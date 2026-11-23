"""Count executed Thumb instructions in a pair of linked release control IRQs.

Build both bus settings with HFI primary, SMO enabled and Flux disabled.
Run with uv run --with unicorn --with pyelftools python <script> BOARD NUM OFF_ELF ON_ELF.
The replay supplies a calibrated zero-current frame and seeds observer history
immediately before a selected speed update. The timer registers are stationary;
the replay advances the Port epoch between frames. The measured IRQ, including
perf_counter, PWM and fault paths, runs unmodified.
"""
import argparse
from collections import Counter
import hashlib
import struct

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import (UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC, UC_ARM_REG_S0,
    UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1,
    UC_CPU_ARM_CORTEX_M4)

RETURN = 0x0807f000
TIMER = 0x40012c00


class Image:
    def __init__(self, path):
        with open(path, 'rb') as file:
            elf = ELFFile(file)
            table = list(elf.get_section_by_name('.symtab').iter_symbols())
            self.symbols = {s.name: s['st_value'] for s in table}
            self.sizes = {s.name: s['st_size'] for s in table}
            flash_end = max(s['p_paddr'] + s['p_filesz'] for s in elf.iter_segments()
                            if s['p_type'] == 'PT_LOAD' and s['p_filesz']
                            and 0x08000000 <= s['p_paddr'] < 0x08100000)
            self.flash = flash_end - 0x08000000
            self.ram = sum(elf.get_section_by_name(n)['sh_size'] for n in ('.bss', '.data'))
            file.seek(0)
            self.digest = hashlib.sha256(file.read()).hexdigest()[:12]
            self.segments = [(s['p_vaddr'], s.data()) for s in elf.iter_segments()
                             if s['p_type'] == 'PT_LOAD' and s['p_filesz']]
            self.fields = {}
            needed = {'mfoc_motor_t', 'mfoc_feedback_t', 'mfoc_smo_t',
                      'mfoc_smo_coeff_t', 'mfoc_smo_axis_t', 'mfoc_hfi_t',
                      'mfoc_app_cfg_t', 'mfoc_app_t', 'mfoc_mdi_port_t',
                      'at32_mfoc_port_t', 'mfoc_encoder_t', 'mfoc_encoder_sample_t'}
            for cu in elf.get_dwarf_info().iter_CUs():
                for die in cu.iter_DIEs():
                    name = die.attributes.get('DW_AT_name')
                    if die.tag != 'DW_TAG_typedef' or not name:
                        continue
                    key = name.value.decode()
                    if key not in needed or key in self.fields:
                        continue
                    target = die.get_DIE_from_attribute('DW_AT_type')
                    self.fields[key] = {
                        m.attributes['DW_AT_name'].value.decode():
                        m.attributes['DW_AT_data_member_location'].value
                        for m in target.iter_children() if m.tag == 'DW_TAG_member'
                        and 'DW_AT_name' in m.attributes}


def prepare_g431_delayed_port(write, symbols, capture, period):
    """Place one ready frame 600 timer ticks after a B top transfer."""
    if 's_pwm_period' not in symbols:
        return
    write(symbols['s_pwm_period'], period)
    write(symbols['s_pwm_capture'], capture - period)
    write(0x40000034, capture)        # TIM2 captured TIM1_TRGO
    write(0x40000024, capture + 600)  # TIM2 current count
    write(TIMER + 0x44, 1 << 15)     # TIM1 main output enable


def replay(image, board, numeric, path, sample):
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
    cpu.reg_write(UC_ARM_REG_C1_C0_2, 0xf00000)
    cpu.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
    for base, size in ((0x08000000, 0x80000), (0x20000000, 0x20000),
                       (0x40000000, 0x40000), (0x50000000, 0x1000),
                       (0xe0000000, 0x10000)):
        cpu.mem_map(base, size)
    for base, data in image.segments:
        cpu.mem_write(base, data)
    sym, fields = image.symbols, image.fields
    tim4 = board == 'stm32g431'
    fixed = numeric == 'fixed'
    def write(address, value, fmt='I'):
        cpu.mem_write(address, struct.pack('<' + fmt, value))
    def read(address, fmt='I'):
        return struct.unpack('<' + fmt, cpu.mem_read(address, struct.calcsize(fmt)))[0]
    def pu(address, value):
        write(address, round(value * 32768) if fixed else value, 'i' if fixed else 'f')
    def call(name, arg=0, arg1=0):
        cpu.reg_write(UC_ARM_REG_SP, 0x20010000)
        cpu.reg_write(UC_ARM_REG_LR, RETURN | 1)
        cpu.reg_write(UC_ARM_REG_R0, arg)
        cpu.reg_write(UC_ARM_REG_R1, arg1)
        cpu.emu_start(sym[name] | 1, RETURN, count=200000)
        assert cpu.reg_read(UC_ARM_REG_PC) == RETURN, name
        return cpu.reg_read(UC_ARM_REG_R0)

    motor = sym['tMfocApp'] + fields['mfoc_app_t']['motor']
    cfg = sym['tMfocAppCfg'] + fields['mfoc_app_cfg_t']['motor']
    def port_init(cpu, pc, size, userdata):
        cpu.reg_write(UC_ARM_REG_R0, 0)
        cpu.reg_write(UC_ARM_REG_PC, cpu.reg_read(UC_ARM_REG_LR))
    address = sym['mfoc_port_init'] & ~1
    hook = cpu.hook_add(UC_HOOK_CODE, port_init, begin=address, end=address)
    assert call('mfoc_motor_Init', motor, cfg) == 0
    cpu.hook_del(hook)  # Only the foreground calibration is substituted.
    m = fields['mfoc_motor_t']
    fb = motor + m['feedback']
    hfi = fb + fields['mfoc_feedback_t']['hfi']
    smo = fb + fields['mfoc_feedback_t']['smo']
    h = fields['mfoc_hfi_t']
    s = fields['mfoc_smo_t']
    port = fields['mfoc_mdi_port_t' if tim4 else 'at32_mfoc_port_t']
    write(sym['s_port'] + port['configured'], 1, 'B')
    midpoint = 8192 if tim4 else 2048
    for field in ('offset_u', 'offset_v', 'offset_w'):
        write(sym['s_port'] + port[field], midpoint)
    gain = -1/476 if tim4 else 3300000/(33*4096*3500)
    if fixed:
        write(sym['s_port'] + port['gain_q30'], round(gain * (1 << 30)), 'i')
    else:
        write(sym['s_port'] + port['gain'], gain, 'f')
    write(TIMER + 0x2c, 4250 if tim4 else 5000)
    write(TIMER + 0x44, 1 << 15)
    if tim4:
        for reg in (0x50000080, 0x50000184, 0x50000180):
            write(reg, midpoint)
        write(0x50000000, 0x40)
        write(0x50000100, 0x40)
    else:
        for reg in (0x4001243c, 0x40012440, 0x40012444):
            write(reg, midpoint)
        write(0x40012400, 4)
    write(motor + m['state'], 2)
    write(motor + m['mode'], 1)
    write(motor + m['previous_voltage_ab_valid'], 1, 'B')
    speed_due = path != 'ordinary'
    smo_due = path in ('smo', 'smo+hfi')
    hfi_due = path in ('hfi', 'smo+hfi')
    write(motor + m['speed_count'], 0 if speed_due else 1, 'H')
    pu(motor + m['speed_ref'], 0.03125)
    write(fb + fields['mfoc_feedback_t']['hfi_mode'], 2)
    write(hfi + h['state'], 1)
    write(hfi + h['lock_count'], 64, 'H')
    write(hfi + h['carrier'], 7 if hfi_due else 0, 'B')
    if 'applied_tag' in h:
        write(hfi + h['applied_tag'], 8 if hfi_due else 1, 'B')
        write(hfi + h['pending_tag'], 1 if hfi_due else 2, 'B')
    write(hfi + h['has_current'], 1, 'B')
    write(hfi + h['has_angle'], 1, 'B')
    # Anisotropic positive response: the 8th sample completes a valid axis.
    for offset, value in ((h['delta_u'], .125), (h['delta_u']+12, .125),
                          (h['delta_i'], .0625), (h['delta_i']+12, .03125)):
        pu(hfi + offset, value)
    pu(smo + s['axis'] + fields['mfoc_smo_axis_t']['e_hat'], .0625)
    span = read(smo + s['c'] + fields['mfoc_smo_coeff_t']['speed_span'], 'B')
    assert span > 1
    write(smo + s['has_angle'], 1, 'B')
    write(smo + s['speed_count'], span-1 if smo_due else 0, 'B')

    hz = 170000000 if tim4 else 200000000
    period = hz // 20000
    timeout = hz * 30 // 1000
    epoch = sym['s_lSystemClockCounts']
    # All measured successful IRQs see precisely the same Port time, including
    # the OFF reference. The optional POSITION read also uses this clock.
    measured_at = 4 * period
    if sample in ('stale', 'delayed_start'):
        measured_at = (1 << 32) - hz // 100
    write(epoch, measured_at - (period if sample == 'fresh' else 0), 'Q')
    if tim4:
        prepare_g431_delayed_port(write, sym, 4 * period, period)
        # G431 also evaluates the physical-position diagnostic. Supply one
        # eligible sample and its real timer scale so both images extrapolate.
        e = fields['mfoc_encoder_t']
        last = fields['mfoc_encoder_sample_t']
        encoder = sym['s_port'] + port['encoder']
        pos = encoder + e['samples']
        write(encoder + e['published'], 0, 'B')
        write(encoder + e['tick_hz'], hz)
        write(encoder + e['timeout_ticks'], hz // 100)
        write(encoder + e['pole_pairs'], 7, 'H')
        write(pos + last['tick'], measured_at - period, 'Q')
        pu(pos + last['speed'], 0.125)
        pu(pos + last['speed_pu'], 0.002)
    if sample != 'off':
        core = motor + m['core']
        now = call('mfoc_port_tick') | (cpu.reg_read(UC_ARM_REG_R1) << 32)
        payload = 0x2001a000
        call('mfoc_board_bus_Reset')
        stamp = now - (timeout - 20 * period if sample == 'stale' else 0)
        write(payload, stamp, 'Q')
        pu(payload + 8, 1.0)
        write(payload + 12, 0)
        assert call('mfoc_board_bus_Publish', payload) == 0
        # In a release ELF that discards all foreground start commands the
        # cold Reset already schedules the first check. A companion ELF
        # built from the same objects exercises Arm as well as the cold path.
        if 'mfoc_board_bus_Arm' in sym:
            assert call('mfoc_board_bus_Arm', now & 0xffffffff, now >> 32) == 0
            if sample == 'delayed_start':
                assert call('mfoc_board_bus_Apply', core) == 0
                write(epoch, read(epoch, 'Q') + period, 'Q')
                arm_at = now + period
                assert call('mfoc_board_bus_Arm', arm_at & 0xffffffff,
                            arm_at >> 32) == 0
        if sample not in ('first', 'delayed_start'):
            assert call('mfoc_board_bus_Apply', core) == 0
        if sample == 'fresh':
            write(epoch, read(epoch, 'Q') + period, 'Q')
            write(payload, now + period, 'Q')
            assert call('mfoc_board_bus_Publish', payload) == 0
        elif sample == 'delayed_start':
            write(epoch, read(epoch, 'Q') + timeout, 'Q')
        elif sample == 'stale':
            # Exactly 1 ms of the 30 ms budget remained at startup. Find
            # the first rejected frame and replay that same state through IRQ.
            for frames in range(1, 22):
                write(epoch, read(epoch, 'Q') + period, 'Q')
                before = bytes(cpu.mem_read(0x20000000, 0x2000))
                if call('mfoc_board_bus_Apply', core) == 7:
                    cpu.mem_write(0x20000000, before)
                    break
            else:
                raise AssertionError('frame-age watchdog did not expire')
            assert frames in (19, 20), ('age at startup lost', frames)

    addresses = {sym[name] & ~1: name for name in ('mfoc_pi_Step',
                 'mfoc_smo_Step', 'mfoc_hfi_Step', 'mfoc_angle_atan2',
                 'mfoc_port_pwm', 'mfoc_motor_Trip', 'get_system_ticks')}
    counts = Counter()
    low_sp = 0x20010000
    apply = sym.get('mfoc_board_bus_Apply', 0) & ~1
    def count(cpu, pc, size, userdata):
        nonlocal low_sp
        low_sp = min(low_sp, cpu.reg_read(UC_ARM_REG_SP))
        counts['instructions'] += 1
        if apply <= pc < apply + image.sizes.get('mfoc_board_bus_Apply', 0):
            counts['apply'] += 1
        if pc in addresses:
            counts[addresses[pc]] += 1
        if pc == (sym['mfoc_pi_Step'] & ~1) and cpu.reg_read(UC_ARM_REG_R0) == motor+m['speed']:
            counts['speed'] += 1
    cpu.hook_add(UC_HOOK_CODE, count)
    # Unicorn caches translated helper blocks during foreground setup.
    # Re-translate them with the instruction hook for the measured IRQ.
    cpu.ctl_flush_tb()
    call('mfoc_motor_IRQHandler')
    rc = read(motor + m['irq_result'])
    if sample in ('stale', 'delayed_start'):
        assert (rc, counts['mfoc_motor_Trip'], counts['mfoc_port_pwm']) == (7, 1, 0)
    else:
        assert (rc, counts['speed'], counts['mfoc_port_pwm']) == (0, int(speed_due), 1), (rc, counts)
        assert read(smo+s['speed_count'], 'B') == (0 if smo_due else 1)
        assert read(hfi+h['carrier'], 'B') == (0 if hfi_due else 1)
        assert read(hfi+h['lock_count'], 'H') == 64
    stack = 0x20010000-low_sp
    total = counts['instructions']
    apply_count = counts['apply']
    ticks = counts['get_system_ticks']
    written_voltage = m.get('preloaded_voltage_ab', m['previous_voltage_ab'])
    output = bytes(cpu.mem_read(motor+written_voltage, 8)) + bytes(
        cpu.mem_read(TIMER+0x34, 12))
    direct = None
    if sample == 'fresh':
        counts.clear()
        low_sp = 0x20010000
        cpu.reg_write(UC_ARM_REG_S0, 0x3f800000)
        assert call('mfoc_core_SetBus', motor + m['core'], 32768) == 0
        direct = (counts['instructions'], 0x20010000-low_sp)
    return total, stack, output, apply_count, direct, ticks


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('board', choices=['stm32g431', 'at32f413'])
    parser.add_argument('numeric', choices=['float', 'fixed'])
    parser.add_argument('off')
    parser.add_argument('on')
    parser.add_argument('--armed-on', help='same ON release objects with Arm retained')
    parser.add_argument('--tick-on-new', action='store_true',
                        help='check the separate absolute-on-new comparison ELF')
    args = parser.parse_args()
    off, on = Image(args.off), Image(args.on)
    armed = Image(args.armed_on) if args.armed_on else None
    if armed:
        assert 'mfoc_board_bus_Arm' in armed.symbols
    print('ELF Flash / static RAM / SHA-256 prefix:',
          [(image.flash, image.ram, image.digest) for image in (off, on)])
    frames = ('off', 'first', 'fresh', 'unchanged', 'stale', 'delayed_start')
    print('IRQ instruction and software-stack column order:', *frames)
    for group in ('ordinary', 'smo', 'hfi', 'smo+hfi'):
        values = [replay(off if frame == 'off' else on, args.board, args.numeric,
                         group, frame) for frame in frames]
        assert all(v[2] == values[0][2] for v in values[1:4])
        off_ticks = values[0][5]
        assert values[1][5] == off_ticks + 1, 'first frame must check absolute time'
        assert values[2][5] == off_ticks + int(args.tick_on_new), 'new-frame clock reads'
        assert values[3][5] == off_ticks, 'steady bus path read the clock'
        print(args.board, args.numeric, group, [v[0] for v in values],
              [v[1] for v in values], 'bus body', [v[3] for v in values[1:]],
              'SetBus', values[2][4], 'clock reads', [v[5] for v in values])
        if armed:
            for case in ('first', 'delayed_start'):
                armed_result = replay(armed, args.board, args.numeric, group, case)
                assert armed_result == values[frames.index(case)], ('Arm handoff', group, case)
