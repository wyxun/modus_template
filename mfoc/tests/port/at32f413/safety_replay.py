"""Exercise F413 safety code from a linked ELF using synthetic MMIO."""
import argparse
from struct import pack, unpack

from elftools.elf.elffile import ELFFile
from unicorn import (
    Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB, UC_HOOK_CODE,
    UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE,
)
from unicorn.arm_const import (
    UC_ARM_REG_PC, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0,
    UC_ARM_REG_R1, UC_ARM_REG_R2, UC_CPU_ARM_CORTEX_M4,
)

ADC, TMR, GPIO, DWT = 0x40012400, 0x40012C00, 0x40010C08, 0xE0001004
CTRL, STATUS, COUNT, BRK = TMR, TMR + 0x10, TMR + 0x24, TMR + 0x44
RAW = (ADC + 0x3C, ADC + 0x40, ADC + 0x44)
DOWN, MOE, SAFE_PINS = 1 << 4, 1 << 15, (1 << 12) | (1 << 4)
STACK, RETURN = 0x20010000, 0x0807F000
PHASE, OVERCURRENT, HALL, MISSING, BREAK, GATE = range(1, 7)
OK, RESULT_IO, RESULT_SAFETY = 0, 6, 7


class Target:
    def __init__(self, symbols, segments):
        self.symbols = symbols
        cpu = self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
        for base, size in ((0x08000000, 0x80000), (0x20000000, 0x20000),
                           (0x40000000, 0x40000), (0xE0000000, 0x10000)):
            cpu.mem_map(base, size)
        for base, data in segments:
            cpu.mem_write(base, data)
        cpu.hook_add(UC_HOOK_CODE, self.count)
        for address, value in ((BRK, MOE), (CTRL, DOWN), (COUNT, 4250),
                               (GPIO, SAFE_PINS)):
            self.put(address, value)
        for address in RAW:
            self.put(address, 2048)
        self.arm()

    def put(self, address, value):
        self.cpu.mem_write(address, pack('<I', value))

    def get(self, address):
        return unpack('<I', self.cpu.mem_read(address, 4))[0]

    def count(self, cpu, _pc, _size, _ctx):
        self.instructions += 1
        self.stack = max(self.stack, STACK - cpu.reg_read(UC_ARM_REG_SP))

    def call(self, name, arg0=0, arg1=0, arg2=0):
        self.instructions = self.stack = 0
        cpu = self.cpu
        for reg, value in ((UC_ARM_REG_SP, STACK), (UC_ARM_REG_LR, RETURN | 1),
                           (UC_ARM_REG_R0, arg0), (UC_ARM_REG_R1, arg1),
                           (UC_ARM_REG_R2, arg2)):
            cpu.reg_write(reg, value)
        address = self.symbols['mfoc_at32_safety_' + name]
        cpu.emu_start(address | 1, RETURN, count=20000)
        assert cpu.reg_read(UC_ARM_REG_PC) == RETURN, name
        return cpu.reg_read(UC_ARM_REG_R0), self.instructions, self.stack

    def arm(self):
        self.call('Arm', 2048, 2048, 2048)

    def expect_fault(self, fault):
        assert self.call('GetFault')[0] == fault
        assert bool(self.get(BRK) & MOE) == (fault == 0)


def test_frame(target, hall):
    cases = [
        ('normal', COUNT, 4250, 0),
        ('descending low edge', COUNT, 3900, 0),
        ('descending high edge', COUNT, 4600, 0),
        ('before window', COUNT, 4601, PHASE),
        ('after window', COUNT, 3899, PHASE),
        ('ascending', CTRL, 0, PHASE),
        ('Hall 000', GPIO, 1 << 12, HALL if hall else 0),
        ('Hall 111', GPIO, (1 << 12) | (3 << 4) | 1, HALL if hall else 0),
    ]
    for phase, register in zip('uvw', RAW):
        for delta in (-123, -122, 122, 123):
            cases.append((f'{phase} raw delta {delta}', register, 2048 + delta,
                          OVERCURRENT if abs(delta) == 123 else 0))
    for label, register, value, fault in cases:
        board = target()
        board.put(register, value)
        ok, instructions, stack = board.call('Frame')
        assert bool(ok) == (fault == 0), label
        board.expect_fault(fault)
        print(f'Frame {label}: {instructions} instructions, {stack} B stack')


def test_tick(target):
    board = target()
    for _ in range(4):
        assert board.call('Frame')[0] == 1
        _, instructions, stack = board.call('Tick')
        board.expect_fault(0)
    print(f'Tick with ADC frames: {instructions} instructions, {stack} B stack')

    board = target()
    for _ in range(2):
        board.call('Tick')
        board.expect_fault(0)
    _, instructions, stack = board.call('Tick')
    board.expect_fault(MISSING)
    print(f'Tick missing-frame trip: {instructions} instructions, {stack} B stack')

    for register, value, fault in ((GPIO, 1 << 4, BREAK),
                                    (STATUS, 1 << 7, BREAK), (BRK, 0, GATE)):
        board = target()
        board.put(register, value)
        board.call('Tick')
        board.expect_fault(fault)
        board.call('Disarm')
        assert board.call('GetFault')[0] == fault
        assert board.call('Frame')[0] == 1
        board.put(GPIO, SAFE_PINS)
        board.put(STATUS, 0)
        board.arm()
        board.put(BRK, MOE)
        assert board.call('Frame')[0] == 1
        board.expect_fault(0)

    board = target()
    board.call('Disarm')
    board.put(BRK, 0)
    for _ in range(4):
        board.call('Tick')
        assert board.call('GetFault')[0] == 0
    board.arm()
    board.put(BRK, MOE)
    assert board.call('Frame')[0] == 1
    board.expect_fault(0)

    board = target()
    preempt = False

    def interrupt(_cpu, _access, _address, _size, _value, _ctx):
        if preempt:
            # ADC publishes a fresh sequence just after SysTick's snapshot.
            heartbeat = board.symbols['s_frame']
            board.put(heartbeat, board.get(heartbeat) + 1)

    board.cpu.hook_add(UC_HOOK_MEM_READ, interrupt, begin=GPIO, end=GPIO)
    board.call('Tick')
    board.call('Tick')
    preempt = True
    board.call('Tick')
    board.expect_fault(0)


def test_start(target):
    cases = [('ready', OK), ('no top', RESULT_SAFETY),
             ('upcount', RESULT_SAFETY), ('late top', RESULT_SAFETY),
             ('no ADC', RESULT_IO), ('Break', RESULT_SAFETY),
             ('no DWT', RESULT_SAFETY), ('late ADC', RESULT_SAFETY)]
    for label, expected in cases:
        board = target()
        board.call('Disarm')
        board.put(BRK, 0)
        board.put(DWT - 4, 0 if label == 'no DWT' else 1)
        if label == 'upcount':
            board.put(CTRL, 0)
        if label == 'late top':
            board.put(COUNT, 3499)
        if label == 'Break':
            board.put(GPIO, 1 << 4)
        acknowledged = False
        top_wait = False
        top_seen = False
        software_events = []

        def write(_cpu, _access, address, _size, value, _ctx):
            nonlocal top_wait, acknowledged
            if address == STATUS:
                top_wait = True
            if address == ADC:
                acknowledged = top_seen
            if address == TMR + 0x14:
                software_events.append(value)

        def read(_cpu, _access, address, _size, _value, _ctx):
            nonlocal top_seen
            if address == STATUS:
                top_seen = top_wait and label != 'no top'
                board.put(address, int(top_seen))
            if address == DWT:
                board.put(address, board.get(address) + 100000)
            if address == ADC:
                assert acknowledged, 'ADC polled before discarding old frame'
                board.put(address, 0 if label == 'no ADC' else 4)
                if label == 'late ADC':
                    board.put(COUNT, 3499)

        board.cpu.hook_add(UC_HOOK_MEM_READ, read)
        board.cpu.hook_add(UC_HOOK_MEM_WRITE, write)
        result, _, _ = board.call('WaitStart')
        assert result == expected, (label, result)
        assert not software_events and not board.get(BRK) & MOE, label


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', choices=('hall', 'no-hall'))
    parser.add_argument('elf', type=argparse.FileType('rb'))
    args = parser.parse_args()
    with args.elf as source:
        elf = ELFFile(source)
        symbols = {s.name: s['st_value'] for s in
                   elf.get_section_by_name('.symtab').iter_symbols()}
        segments = [(s['p_vaddr'], s.data()) for s in elf.iter_segments()
                    if s['p_type'] == 'PT_LOAD' and s['p_filesz']]

    def target():
        return Target(symbols, segments)

    test_frame(target, args.profile == 'hall')
    test_tick(target)
    test_start(target)
    print(f'F413 {args.profile}: phase, current, Hall, watchdog and gate checks passed')


if __name__ == '__main__':
    main()
