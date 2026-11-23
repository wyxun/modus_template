"""Replay the compiled A/B probe's frame accounting against synthetic MMIO.

uv run --with unicorn --with pyelftools python .../g431_p10_probe_replay.py B ELF
Timestamps and hardware states here are test inputs. They are not board cycles.
"""
import argparse
from struct import pack
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_SP
from g431_p10_exit import SNAPSHOT_SIZE, decode as decode_exit_witness

TIM1, TIM2, ADC1, ADC2 = 0x40012C00, 0x40000000, 0x50000000, 0x50000100
MASK, PERIOD = 0xffffffff, 8500
RETURN = 0x0807f000


def field_offsets(elf, name):
    for cu in elf.get_dwarf_info().iter_CUs():
        for die in cu.iter_DIEs():
            attr = die.attributes.get('DW_AT_name')
            if die.tag == 'DW_TAG_typedef' and attr and attr.value == name.encode():
                struct = die.get_DIE_from_attribute('DW_AT_type')
                return {member.attributes['DW_AT_name'].value.decode():
                        member.attributes['DW_AT_data_member_location'].value
                        for member in struct.iter_children() if member.tag == 'DW_TAG_member'
                        and 'DW_AT_name' in member.attributes}
    raise ValueError(name)


def replay(elf, phase):
    cpu = Uc(UC_ARCH_ARM, UC_MODE_MCLASS | UC_MODE_THUMB)
    for start, size in [(0x08000000, 0x80000), (0x20000000, 0x20000),
                        (0x40000000, 0x20000), (ADC1, 0x1000),
                        (0xe0000000, 0x10000)]:
        cpu.mem_map(start, size)
    for sec in elf.iter_sections():
        if sec['sh_flags'] & 2 and sec['sh_type'] == 'SHT_PROGBITS':
            cpu.mem_write(sec['sh_addr'], sec.data())
    sym = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    exit_witness = sym.get('g_p10_exit_witness')

    def put(addr, data):
        cpu.mem_write(addr, pack('<I', data & MASK))

    def get(addr):
        return int.from_bytes(cpu.mem_read(addr, 4), 'little')

    def call(name, arg=0):
        cpu.reg_write(UC_ARM_REG_LR, RETURN | 1)
        cpu.reg_write(UC_ARM_REG_SP, 0x2001f000)
        cpu.reg_write(UC_ARM_REG_R0, arg)
        cpu.emu_start(sym[name] | 1, RETURN, count=2000000)
        return cpu.reg_read(UC_ARM_REG_R0)

    # Use public DWARF to locate the Motor without assuming either numeric ABI.
    motor = sym['tMfocApp'] + field_offsets(elf, 'mfoc_app_t')['motor']
    fields = field_offsets(elf, 'mfoc_motor_t')
    if exit_witness:
        core = (motor + fields['previous_core_output'] +
                field_offsets(elf, 'mfoc_core_output_t')['limited'])
    put(motor + fields['state'], 2)
    if 'g_mfoc_g431_pwm_phase' in sym:
        p = sym['g_mfoc_g431_pwm_phase']
        po = field_offsets(elf, 'mfoc_g431_pwm_phase_t')
    else:
        p = sym['g_mfoc_p10_phase']
        po = field_offsets(elf, 'mfoc_p10_phase_evidence_t')
    put(p + po['startup_captures'], 1)
    cpu.mem_write(p + po['locked'], b'\x01')
    put(TIM1 + 0x2c, 4250)
    put(TIM1 + 0x40, 4240)
    call('mfoc_p10_Init')

    # ADC 18, DMA 11, TIM1 break 24, SysTick system handler, USART2 38.
    for irq, pri in ((18, 1), (11, 2), (24, 2), (38, 4)):
        addr = 0xe000e100 + 4 * (irq // 32)
        put(addr, get(addr) | (1 << (irq % 32)))
        cpu.mem_write(0xe000e400 + irq, bytes([pri << 4]))
    put(0xe000ed0c, 3 << 8)
    cpu.mem_write(0xe000ed23, b'\x30')
    put(0xe000e010, 7)
    put(ADC1 + 4, 64)
    put(TIM1 + 0x0c, 128)
    offset = 4240 if phase == 'A' else 8490
    permitted = 3408 if phase == 'A' else 6808
    last = (0xffffffff - 4500 + offset) & MASK

    # Gates-off JEOS calibration. Rank completion and volatile flag
    # acknowledgement are modeled independently for ADC1 and ADC2.
    if 'p10_CalibrateFrame' in sym:
        put(motor + fields['state'], 1)
        put(TIM1, 1)
        put(TIM2, 1)
        put(ADC1 + 4, 0)
        put(TIM1 + 0x20, 1)  # CC1E open: command must refuse any ADC access
        assert call('p10_CalibrateFrame') == 0
        put(TIM1 + 0x20, 0)
        elapsed = [0]
        acknowledged = [-2, -2]
        emitted = [-1, -1]

        def read_cal(_, access, addr, size, val, data):
            if addr == TIM2 + 0x24:
                elapsed[0] += 13
                put(addr, elapsed[0])
            elif addr == TIM2 + 0x34:
                put(addr, elapsed[0] // PERIOD * PERIOD)
            elif addr in (ADC1, ADC2):
                channel = int(addr == ADC2)
                instant = elapsed[0] - offset - (630 if channel else 605)
                emitted[channel] = instant // PERIOD
                put(addr, 0x40 if emitted[channel] > acknowledged[channel] else 0)

        def write_cal(_, access, addr, size, val, data):
            if addr in (ADC1, ADC2):
                assert val == 0x40
                ch = int(addr == ADC2)
                acknowledged[ch] = emitted[ch]

        r = cpu.hook_add(UC_HOOK_MEM_READ, read_cal)
        wr = cpu.hook_add(UC_HOOK_MEM_WRITE, write_cal)
        assert call('p10_CalibrateFrame') == 1, 'stopped paired JEOS polls'
        cpu.hook_del(r)
        cpu.hook_del(wr)
        cal = sym['s_cal']
        assert get(cal) == 16 and get(cal + 4) == 0
        assert get(cal + 8) <= 630 <= get(cal + 12)
        print(f'{phase}: stopped calibration n=16, known joint JEOS +630; '
              f'bounds [{get(cal + 8)},{get(cal + 12)}], '
              f'widest read bracket {get(cal + 16)} ticks')
        put(ADC1 + 4, 64)
        put(motor + fields['state'], 2)

    def frame(k, *, late=False, incomplete=False, fail=False, events=0,
              limited=False, bus=False):
        t = (last + PERIOD * k) & MASK
        cap = (t - offset + ((offset + 620) // PERIOD) * PERIOD) & MASK
        put(TIM2 + 0x34, cap)
        put(sym['g_p10_irq_entry'], t + 620)
        put(TIM2 + 0x24, t + 654)
        put(ADC1, 64)
        put(ADC2, 0 if incomplete else 64)
        call('mfoc_p10_Begin', motor)
        put(sym['g_p10_events'], get(sym['g_p10_events']) | events)
        if exit_witness:
            cpu.mem_write(motor + fields['previous_core_output_valid'],
                          bytes([limited]))
            cpu.mem_write(core, bytes([limited]))
            if bus:
                cpu.mem_write(sym['g_p10_bus_applied'], b'\x01')
        if not fail:
            put(TIM2 + 0x24, t + permitted + int(late))
            call('mfoc_p10_PwmWritten')
        put(motor + fields['irq_result'], 1 if fail else 0)
        call('mfoc_p10_End', motor)
        tail = PERIOD if late else PERIOD - (100 if exit_witness else 1)
        put(get(sym['g_p10_irq_tail']), t + tail)
        if exit_witness:
            put(TIM2 + 0x10, 4)
            put(TIM2 + 0x24, t + tail + 20)
            call('TIM2_IRQHandler')

    frame(0, limited=bool(exit_witness))
    nv = field_offsets(elf, 'p10_nvic_t')
    assert cpu.mem_read(sym['s_nvic'] + nv['valid'], 1) == b'\x01', 'NVIC audit'
    frame(1, late=True, events=3, limited=bool(exit_witness),
          bus=bool(exit_witness and 'g_p10_bus_applied' in sym))
    frame(2, incomplete=True)
    frame(4, fail=True)           # one absent frame, one no-submit error
    frame(5)                      # retires previous tail, this frame still pending
    assert get(sym['s_groups'] + 5 * 12) == 2, 'valid writes'
    assert get(sym['s_groups'] + 9 * 12) == 1, 'heavy + speed'
    for var in ('s_window_fail', 's_tail_fail', 's_incomplete', 's_gaps',
                's_missing_write', 's_errors'):
        assert get(sym[var]) == 1, var
    # A warm start captures current routing again. An unexpected enabled IRQ
    # invalidates the NVIC snapshot. A new start resets the sequence baseline.
    put(0xe000e100, get(0xe000e100) | 1)
    put(p + po['startup_captures'], 2)
    frame(9)
    assert cpu.mem_read(sym['s_nvic'] + nv['valid'], 1) == b'\x00'
    assert get(sym['s_gaps']) == 1
    if exit_witness:
        raw = bytes(cpu.mem_read(exit_witness, SNAPSHOT_SIZE))
        witness = decode_exit_witness(raw, PERIOD)
        assert witness['complete'] and not witness['witness_deadline_pass']
        groups = witness['groups']
        assert groups['all']['count'] == 6 and groups['all']['late'] == 1
        assert groups['limit_only']['count'] == 1
        assert groups['speed_smo_limit']['count'] == 1
        assert groups['bus_speed_smo_limit']['count'] == int('g_p10_bus_applied' in sym)
        assert groups['abnormal']['count'] == 2
        print(f'{phase}: linked ADC tags paired with the modeled lower-priority '
              'TIM2 witness for control, limits and abnormal exits')
    print(f'{phase}: wrapped timestamps, both deadline boundaries, gap, incomplete '
          'ADC2, fault/no-write, collision tags, warm restart and NVIC drift passed')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('phase', choices=['A', 'B'])
    ap.add_argument('elf')
    args = ap.parse_args()
    with open(args.elf, 'rb') as image:
        replay(ELFFile(image), args.phase)
