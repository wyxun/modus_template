"""Run G431 phase code from a bench ELF on a center-aligned timer model.

uv run --with unicorn --with pyelftools python g431_p10_phase_replay.py A ELF
Use B for top transfer. Ticks advance per emulated instruction; these are
phase/protocol checks, never a measurement of firmware execution time.
"""
import argparse
import struct

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn import UC_HOOK_CODE, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import (UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC,
                               UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_PRIMASK,
                               UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP,
                               UC_CPU_ARM_CORTEX_M4)

from g431_p10_probe_replay import field_offsets

TIM1, TIM2 = 0x40012c00, 0x40000000
CR1, SR, EGR, CCER, CNT, ARR, RCR, CCR1 = 0, 0x10, 0x14, 0x20, 0x24, 0x2c, 0x30, 0x34
ADC1, ADC2, BDTR = 0x50000000, 0x50000100, 0x44
JEOS, MOE = 0x40, 0x8000
PERIOD, MASK = 4250, 0xffffffff
RETURN = 0x0807f000


class Bench:
    def __init__(self, elf, phase, masked=0):
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
        self.cpu.reg_write(UC_ARM_REG_C1_C0_2, 0xf00000)
        self.cpu.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
        for address, size in ((0x08000000, 0x80000), (0x20000000, 0x20000),
                              (0x40000000, 0x30000), (0x50000000, 0x1000),
                              (0xe000e000, 0x1000)):
            self.cpu.mem_map(address, size)
        for section in elf.iter_sections():
            if section['sh_flags'] & 2 and section['sh_type'] == 'SHT_PROGBITS':
                self.cpu.mem_write(section['sh_addr'], section.data())
        self.symbols = {s.name: s['st_value'] for s in
                        elf.get_section_by_name('.symtab').iter_symbols()}
        self.ticks = 0
        self.start = self.capture_start = self.next_extremum = None
        self.pending = int(phase == 'A')
        self.repeat = self.status = self.stamp = 0
        self.events, self.writes = [], []
        self.last_duty = self.enabled_after = None
        self.missing = self.force_reverse = self.bad_ug = False
        self.phase = phase
        self.break_during_write = False
        self.cross_during_write = False
        self.cpu.reg_write(UC_ARM_REG_PRIMASK, masked)
        self.cpu.hook_add(UC_HOOK_CODE, self.step)
        self.cpu.hook_add(UC_HOOK_MEM_READ, self.read, begin=TIM2, end=TIM1+0x50)
        self.cpu.hook_add(UC_HOOK_MEM_WRITE, self.write, begin=TIM2, end=TIM1+0x50)
        self.put(TIM1 + CR1, 0x20)
        self.put(TIM1 + ARR, PERIOD)
        self.put(TIM1 + RCR, self.pending)

    def put(self, address, value):
        self.cpu.mem_write(address, struct.pack('<I', value & MASK))

    def get(self, address):
        return int.from_bytes(self.cpu.mem_read(address, 4), 'little')

    def evidence(self):
        name = ('g_mfoc_g431_pwm_phase' if 'g_mfoc_g431_pwm_phase' in self.symbols
                else 'g_mfoc_p10_phase')
        address = self.symbols[name]
        return struct.unpack('<6I?', self.cpu.mem_read(address, 25))

    def tick(self, delta):
        self.ticks += delta
        while self.next_extremum is not None and self.next_extremum <= self.ticks:
            if self.repeat:
                self.repeat -= 1
            else:
                self.repeat = self.pending
                self.status |= 1
                if not self.missing and self.capture_start is not None:
                    self.stamp = (self.next_extremum - self.capture_start) & MASK
                    self.events.append(self.stamp)
            self.next_extremum += PERIOD

    def step(self, cpu, pc, size, _):
        self.tick(4)

    def read(self, cpu, access, address, size, value, _):
        travel = (self.ticks - self.start) % (2*PERIOD) if self.start is not None else 0
        down = travel >= PERIOD
        if address == TIM1 + CNT:
            self.put(address, (2*PERIOD-travel if down else travel) + int(self.bad_ug))
        elif address == TIM1 + CR1:
            self.put(address, self.get(address) & ~16 | 16*int(down or self.force_reverse))
        elif address == TIM1 + SR:
            self.put(address, self.status)
        elif address == TIM2 + CNT:
            self.put(address, 0 if self.capture_start is None else self.ticks-self.capture_start)
        elif address == TIM2 + CCR1:
            self.put(address, self.stamp)

    def write(self, cpu, access, address, size, value, _):
        self.writes.append((address, value, cpu.reg_read(UC_ARM_REG_PRIMASK)))
        if address == TIM1 + EGR and value & 1:
            self.repeat = self.pending
            self.status |= 1
        elif address == TIM1 + CR1 and value & 1 and self.start is None:
            self.start = self.ticks
            self.next_extremum = self.start + PERIOD
        elif address == TIM1 + RCR:
            self.pending = value
        elif address == TIM1 + SR:
            self.status &= value
        elif address == TIM2 + CR1 and value & 1:
            self.capture_start = self.ticks
        elif address == TIM1 + CCR1 + 8:
            self.last_duty = self.ticks
            if self.break_during_write:
                self.put(TIM1 + BDTR, self.get(TIM1 + BDTR) & ~MOE)
            if self.cross_during_write:
                self.tick(2 * PERIOD)
        elif address == TIM1 + CCER and value & 0x555:
            age = (self.ticks - self.capture_start - self.stamp) & MASK
            self.enabled_after = self.ticks - age
            assert self.last_duty < self.enabled_after <= self.ticks

    def call(self, name):
        self.cpu.reg_write(UC_ARM_REG_SP, 0x2001f000)
        self.cpu.reg_write(UC_ARM_REG_LR, RETURN | 1)
        self.cpu.emu_start(self.symbols[name] | 1, RETURN, count=400000)
        assert self.cpu.reg_read(UC_ARM_REG_PC) == RETURN, name
        return self.cpu.reg_read(UC_ARM_REG_R0)


def neutral_duty(elf, board):
    scalar = next(d for cu in elf.get_dwarf_info().iter_CUs()
                  for d in cu.iter_DIEs() if d.tag == 'DW_TAG_typedef'
                  and d.attributes.get('DW_AT_name')
                  and d.attributes['DW_AT_name'].value == b'mfoc_scalar_t')
    while scalar.tag == 'DW_TAG_typedef':
        scalar = scalar.get_DIE_from_attribute('DW_AT_type')
    floating = scalar.attributes['DW_AT_name'].value == b'float'
    addr = 0x2001d000
    board.cpu.mem_write(addr, struct.pack('<3f', 0.5, 0.5, 0.5) if floating
                        else struct.pack('<3i', 16384, 16384, 16384))
    return addr


def prepare_current_port(elf, board):
    fields = field_offsets(elf, 'mfoc_mdi_port_t')
    addr = board.symbols['s_port']
    board.cpu.mem_write(addr + fields['configured'], b'\x01')
    for name, reg in (('offset_u', ADC1+0x80), ('offset_v', ADC2+0x84),
                      ('offset_w', ADC2+0x80)):
        board.put(addr + fields[name], 8192)
        board.put(reg, 8192)
    if 'gain' in fields:
        board.cpu.mem_write(addr + fields['gain'], struct.pack('<f', 1/476))
    else:
        board.put(addr + fields['gain_q30'], round((1 << 30) / 476))


def sample_at_age(board, *, periods=0, complete=True):
    age = (board.ticks - board.capture_start - board.stamp) & MASK
    board.tick(max(600 - age, 0) + periods * 2 * PERIOD)
    board.put(ADC1, JEOS)
    board.put(ADC2, JEOS if complete else 0)
    board.cpu.reg_write(UC_ARM_REG_R0, 0x2001d100)
    return board.call('mfoc_port_current')


def run_production_frames(elf, cold):
    def start(first_wrap=False):
        board = Bench(elf, 'B')
        board.call(cold)
        if first_wrap:
            # Qualify and open just before TIM2 wraps. The next capture
            # advances the Port sequence through zero without special cases.
            offset = MASK - board.stamp - (2*PERIOD + 4000)
            board.capture_start -= offset
            board.stamp = (board.stamp + offset) & MASK
        prepare_current_port(elf, board)
        duty = neutral_duty(elf, board)
        board.cpu.reg_write(UC_ARM_REG_R0, duty)
        assert board.call('mfoc_port_pwm_start') == 0
        return board, duty

    board, duty = start(first_wrap=True)
    assert sample_at_age(board) == 0
    previous = board.stamp
    board.cpu.reg_write(UC_ARM_REG_R0, duty)
    assert board.call('mfoc_port_pwm') == 0
    assert sample_at_age(board, periods=1) == 0
    assert board.stamp < previous
    assert sample_at_age(board, periods=2) == 7  # Missing one complete frame.
    assert board.call('mfoc_port_pwm_stop') == 0
    board.cpu.reg_write(UC_ARM_REG_R0, duty)
    assert board.call('mfoc_port_pwm_start') == 0
    assert sample_at_age(board, periods=1) == 0  # First IRQ lost while IDLE.
    assert sample_at_age(board, periods=1, complete=False) == 6

    board, duty = start()
    assert sample_at_age(board) == 0
    age = (board.ticks - board.capture_start - board.stamp) & MASK
    board.tick(2 * PERIOD - 64 - age)
    mark = len(board.writes)
    board.cpu.reg_write(UC_ARM_REG_R0, duty)
    assert board.call('mfoc_port_pwm') == 7
    assert not any(TIM1 + CCR1 <= r <= TIM1 + CCR1 + 8
                   for r, _, _ in board.writes[mark:])

    for disruption in ('cross_during_write', 'break_during_write'):
        board, duty = start()
        assert sample_at_age(board) == 0
        setattr(board, disruption, True)
        board.cpu.reg_write(UC_ARM_REG_R0, duty)
        assert board.call('mfoc_port_pwm') == 7
        assert board.call('mfoc_port_pwm_stop') == 0
        assert (board.get(TIM1 + CCER) & 0x555) == 0
    print('B release: first-frame handoff, restart, wrap, missed frame, '
          'incomplete ADC, pre-write deadline, crossover and Break passed')


def run_bus_arming(elf, board, duty):
    # Use the real front-end publication and Port Arm, with a simulated
    # stationary SysTick clock independent of the TIM1/TIM2 phase model.
    board.call('mfoc_board_bus_Reset')
    board.cpu.reg_write(UC_ARM_REG_R0, duty)
    assert board.call('mfoc_port_pwm_start') == 5  # No sample published.
    assert (board.get(TIM1 + CCER) & 0x555) == 0

    stamp = board.call('mfoc_port_tick') | (board.cpu.reg_read(UC_ARM_REG_R1) << 32)
    fields = field_offsets(elf, 'mfoc_mdi_port_t')
    value = struct.pack('<f', 1.0) if 'gain' in fields else struct.pack('<i', 32768)
    sample = duty + 0x200
    board.cpu.mem_write(sample, struct.pack('<Q', stamp) + value + bytes(4))
    board.cpu.reg_write(UC_ARM_REG_R0, sample)
    assert board.call('mfoc_board_bus_Publish') == 0
    board.cpu.reg_write(UC_ARM_REG_R0, duty)
    assert board.call('mfoc_port_pwm_start') == 0
    assert (board.get(TIM1 + CCER) & 0x555) == 0x555
    assert board.call('mfoc_port_pwm_stop') == 0

    epoch = board.symbols['s_lSystemClockCounts']
    board.cpu.mem_write(epoch, struct.pack('<Q', 170000000 * 30 // 1000))
    board.cpu.reg_write(UC_ARM_REG_R0, duty)
    assert board.call('mfoc_port_pwm_start') == 7
    assert (board.get(TIM1 + CCER) & 0x555) == 0
    print('B runtime bus: missing/stale sample keeps gates closed, fresh sample opens')


def run(elf, phase):
    symbols = {s.name for s in elf.get_section_by_name('.symtab').iter_symbols()}
    cold = ('mfoc_g431_pwm_StartCounter' if 'g_mfoc_g431_pwm_phase' in symbols
            else 'mfoc_p10_PhaseStartCounter')
    warm = ('mfoc_g431_pwm_WaitTransfer' if 'g_mfoc_g431_pwm_phase' in symbols
            else 'mfoc_p10_PhaseWaitTransfer')
    for masked in (0, 1):
        board = Bench(elf, phase, masked)
        board.call(cold)
        count, first, second, starts, last, failures, locked = board.evidence()
        assert locked and not failures and count < PERIOD and starts == 0
        assert [first, second] == board.events and second-first == 2*PERIOD
        assert board.cpu.reg_read(UC_ARM_REG_PRIMASK) == masked
        cen = next(i for i, w in enumerate(board.writes) if w[0] == TIM1 + CR1 and w[1] & 1)
        assert board.writes[cen][2] == 1
        if phase == 'B':
            assert board.writes[cen+1] == (TIM1 + RCR, 1, 1)
        begin = len(board.writes)
        for wait in (37, PERIOD-100, PERIOD+19, 2*PERIOD-1):
            board.tick(wait)
            n = len(board.events)
            assert board.call(warm) and len(board.events) > n
            assert board.evidence()[4] == board.events[-1]
        assert not board.writes[begin:]  # No warm UG, CEN or RCR rewrite.
        assert board.evidence()[3] == 4
        # Move both the current and most recent capture timestamps across
        # 32-bit wrap without changing the physical TIM1 phase or period.
        offset = MASK - (board.ticks - board.capture_start) - 40
        board.capture_start -= offset
        board.stamp = (board.stamp + offset) & MASK
        assert board.call(warm) and board.stamp < 2*PERIOD
        board.status |= 0x80             # A latched break survives transfers.
        board.tick(2*PERIOD)
        assert board.call(warm) and board.status & 0x80
        board.missing = True
        assert not board.call(warm) and board.evidence()[5] == 1

    for failure in ('bad_ug', 'force_reverse', 'missing', 'wrong_repetition'):
        board = Bench(elf, phase)
        if failure == 'wrong_repetition':
            board.pending ^= 1
        else:
            setattr(board, failure, True)
        board.call(cold)
        assert board.evidence()[5:] == (1, False)
        if failure in ('bad_ug', 'force_reverse'):
            assert board.start is None
        assert not board.call(warm)
    if 'mfoc_port_pwm_start' in board.symbols:
        board = Bench(elf, phase)
        board.call(cold)
        duty = neutral_duty(elf, board)
        if 'mfoc_board_bus_Arm' in symbols:
            run_bus_arming(elf, board, duty)
        else:
            for channel, old in enumerate((320, 3021, 1721)):
                board.put(TIM1 + CCR1 + 4*channel, old)
            board.cpu.reg_write(UC_ARM_REG_R0, duty)
            assert board.call('mfoc_port_pwm_start') == 0
            assert [board.get(TIM1+CCR1+4*i) for i in range(3)] == [PERIOD//2]*3
            gates = 0x555
            assert board.get(TIM1 + CCER) & gates == gates
            assert board.call('mfoc_port_pwm_stop') == 0
            assert board.get(TIM1 + CCER) & gates == 0
            board.status |= 0x80
            board.cpu.reg_write(UC_ARM_REG_R0, duty)
            assert board.call('mfoc_port_pwm_start') == 7  # MFOC_RESULT_SAFETY
            assert board.get(TIM1 + CCER) & gates == 0
            board.status &= ~0x80
            board.missing = True
            board.cpu.reg_write(UC_ARM_REG_R0, duty)
            assert board.call('mfoc_port_pwm_start') == 6  # MFOC_RESULT_IO
            assert board.get(TIM1 + CCER) & gates == 0
    print(f'{phase}: cold, both PRIMASK states, warm phase sweep/rollover, '
          'break, missing capture, bad UG/direction and reversed phase passed')
    if phase == 'B' and 's_pwm_period' in symbols and 'mfoc_board_bus_Arm' not in symbols:
        run_production_frames(elf, cold)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=['A', 'B'])
    parser.add_argument('elf', type=argparse.FileType('rb'))
    args = parser.parse_args()
    run(ELFFile(args.elf), args.phase)
