"""Replay the linked 24 V AT32F413 Hall debug-rel firmware.

Natural timer updates transfer staged duties and trigger synthetic ADC frames.
The firmware owns initialization, gating, frame checks and fault handling.
Modeled register transitions do not measure physical PWM timing or Hall wiring.
"""
import argparse
from struct import pack, unpack

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn import UC_HOOK_CODE, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import (
    UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC, UC_ARM_REG_PC, UC_ARM_REG_SP,
    UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2,
    UC_ARM_REG_S0, UC_ARM_REG_S1, UC_ARM_REG_PRIMASK, UC_CPU_ARM_CORTEX_M4,
)

from runtime_bus_irq_replay import Image

ADC = 0x40012400
PWM = 0x40012C00
HALL_TIMER = 0x40000400
HALL_CAPTURE_FLAG = 1 << 1
GPIOB_IDT = 0x40010C08
HALL_H1 = 1 << 4
BREAK_INACTIVE = 1 << 12
BREAK_LATCHED = 1 << 7
PWM_OUTPUT_ENABLE = 1 << 15
CORE_HZ = 200_000_000
RETURN = 0x0807F000
DWT = 0xE0001000


class TopSampling:
    """20 kHz center-aligned timer model. Peripheral polls advance simulated time."""

    def __init__(self, cpu, get, put):
        self.get, self.put = get, put
        self.cycles = self.origin = 0
        self.top_events = self.adc_events = self.software_updates = 0
        self.timer_flags = self.adc_flags = 0
        self.active = None
        self.conversion_ticks = 750  # Synthetic latency from top trigger to PCCE.
        self.deliver_adc = True
        self.gate_opens = 0
        for address in (ADC, ADC + 8, PWM + 0x10, DWT + 4):
            cpu.hook_add(UC_HOOK_MEM_READ, self.read, begin=address, end=address)
        for address in (ADC, PWM, PWM + 0x10, PWM + 0x14, PWM + 0x30, PWM + 0x44):
            cpu.hook_add(UC_HOOK_MEM_WRITE, self.write, begin=address, end=address)

    def advance(self, ticks):
        old = self.cycles - self.origin
        self.cycles += ticks
        if not self.get(PWM) & 1:
            return
        arr = self.get(PWM + 0x2C)
        period = 2 * arr
        now = self.cycles - self.origin
        if (now - arr) // period > (old - arr) // period:
            assert self.get(PWM + 0x30) == 1  # One natural UEV per carrier.
            self.top_events += 1
            self.timer_flags |= 1
            self.active = tuple(self.get(PWM + off) for off in (0x34, 0x38, 0x3C))
        conversion = arr + self.conversion_ticks
        if (now - conversion) // period > (old - conversion) // period:
            trigger = self.get(ADC + 8)
            if (self.deliver_adc and (self.get(PWM + 4) >> 4) & 7 == 2
                    and trigger & 0x0100F000 == 0x8000):
                self.adc_events += 1
                self.adc_flags |= 4
        phase = now % period
        self.put(PWM + 0x24, min(phase, period - phase))
        self.put(PWM, (self.get(PWM) & ~16) | (16 if phase >= arr else 0))

    def read(self, cpu, access, address, size, value, data):
        if address in (DWT + 4, ADC, PWM + 0x10):
            self.advance(40)
        if address == DWT + 4:
            self.put(address, self.cycles & 0xFFFFFFFF)
        elif address == ADC:
            self.put(address, self.adc_flags)
        elif address == PWM + 0x10:
            self.put(address, self.timer_flags)
        elif address == ADC + 8:
            # The simulated ADC completes its internal startup calibration.
            self.put(address, self.get(address) & ~12)

    def write(self, cpu, access, address, size, value, data):
        if address == PWM and value & 1 and not self.get(PWM) & 1:
            assert not value & 16 and self.get(PWM + 0x24) == 0
            assert self.get(PWM + 0x30) == 0 and cpu.reg_read(UC_ARM_REG_PRIMASK) == 1
            self.origin = self.cycles
        elif address == PWM + 0x30 and self.get(PWM) & 1:
            assert value == 1 and cpu.reg_read(UC_ARM_REG_PRIMASK) == 1
        elif address == PWM + 0x14 and value & 1:
            self.software_updates += 1
            assert not self.get(ADC + 8) & 0x8000, 'software UEV with ADC armed'
            self.timer_flags |= 1
        elif address == PWM + 0x10:
            self.timer_flags &= value  # W0C
        elif address == ADC:
            self.adc_flags &= value  # W0C
        elif address == PWM + 0x44 and value & PWM_OUTPUT_ENABLE:
            assert self.active == (2500, 2500, 2500), 'neutral not transferred'
            assert not self.adc_flags & 4 and not self.timer_flags & BREAK_LATCHED
            self.gate_opens += 1

    def frame(self):
        arr = self.get(PWM + 0x2C)
        wait = (arr + self.conversion_ticks - (self.cycles - self.origin)) % (2 * arr)
        self.advance(wait or 2 * arr)
        assert self.adc_flags & 4 and self.get(ADC + 4) & (1 << 7)


def replay(image, numeric, measure=False):
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
    for base, size in ((0x08000000, 0x80000), (0x20000000, 0x20000),
                       (0x40000000, 0x40000), (0xE0000000, 0x10000)):
        cpu.mem_map(base, size)
    cpu.reg_write(UC_ARM_REG_C1_C0_2, 0xF00000)
    cpu.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
    for base, data in image.segments:
        cpu.mem_write(base, data)
    symbols = image.symbols
    breakpoint = [None]
    measured_irq = []

    def visit_instruction(_cpu, addr, _size, _data):
        if addr == breakpoint[0]:
            cpu.emu_stop()
        elif measured_irq:
            measured_irq[0] += 1
            measured_irq[1] = min(measured_irq[1], cpu.reg_read(UC_ARM_REG_SP))

    cpu.hook_add(UC_HOOK_CODE, visit_instruction)

    def put(address, value):
        cpu.mem_write(address, pack('<I', value))

    def get(address):
        return unpack('<I', cpu.mem_read(address, 4))[0]

    def call(name, arg0=0, arg1=0, *, float_arg=None, stack=0x20010000, interrupt_start=None):
        cpu.reg_write(UC_ARM_REG_SP, stack)
        cpu.reg_write(UC_ARM_REG_LR, RETURN | 1)
        cpu.reg_write(UC_ARM_REG_R0, arg0)
        cpu.reg_write(UC_ARM_REG_R1, arg1)
        if float_arg is not None:
            cpu.reg_write(UC_ARM_REG_S0, unpack('<I', pack('<f', float_arg))[0])
        entry = symbols[name] | 1
        if interrupt_start:
            # Inject the ADC immediately after the Port has enabled it, before
            # the preempted Motor start routine can execute another instruction.
            port = symbols['mfoc_port_pwm_start'] & ~1
            breakpoint[0] = port
            cpu.emu_start(entry, RETURN, count=300000)
            assert cpu.reg_read(UC_ARM_REG_PC) == port
            resume = cpu.reg_read(UC_ARM_REG_LR) & ~1
            breakpoint[0] = resume
            cpu.emu_start(port | 1, RETURN, count=300000)
            assert cpu.reg_read(UC_ARM_REG_PC) == resume and cpu.reg_read(UC_ARM_REG_R0) == 0
            breakpoint[0] = None
            saved = cpu.context_save()
            interrupt_start()
            cpu.context_restore(saved)
            entry = resume | 1
        cpu.emu_start(entry, RETURN, count=300000)
        assert cpu.reg_read(UC_ARM_REG_PC) == RETURN, name
        return cpu.reg_read(UC_ARM_REG_R0)

    def compare():
        return tuple(get(PWM + offset) for offset in (0x34, 0x38, 0x3C))

    put(symbols['system_core_clock'], CORE_HZ)
    put(0xE000E014, 199999)  # SysTick at 1 kHz.
    put(0xE000E018, 99999)
    put(GPIOB_IDT, BREAK_INACTIVE | HALL_H1)
    put(0x40020000, 2)  # DMA1 regular-group transfer complete.
    put(symbols['g_ahwAt32AdcOrdinary'], 1536)  # 24 V.
    put(PWM + 0x2C, 5000)
    for offset in (0x3C, 0x40, 0x44):
        put(ADC + offset, 2048)
    timer = TopSampling(cpu, get, put)
    call('halpwm_Init')
    call('haladc_Init')
    assert not get(ADC + 8) & 0x8000
    call('halpwm_StartSampling')
    assert timer.top_events and get(ADC + 8) & 0x8000
    assert timer.gate_opens == 0
    initial_updates = timer.software_updates

    def control_frame(label=None, *, irq_lag=0, stack=0x20010000):
        timer.frame()
        timer.advance(irq_lag)
        if measure and label:
            measured_irq[:] = [0, 0x20010000]
        call('ADC1_2_IRQHandler', stack=stack)
        if measured_irq:
            print(f'{label}: {measured_irq[0]} instructions, '
                  f'{0x20010000 - measured_irq[1]} B software stack')
            measured_irq.clear()
        assert not timer.adc_flags & 4

    app = symbols['tMfocApp']
    assert call('mfoc_app_Init', app, symbols['tMfocAppCfg']) == 0
    assert timer.adc_events >= 8
    assert not get(PWM + 0x44) & PWM_OUTPUT_ENABLE

    port_base = image.fields['at32_mfoc_port_t']['voltage_base_millivolt']
    assert get(symbols['s_port'] + port_base) == 24000
    motor = app + image.fields['mfoc_app_t']['motor']
    fields = image.fields['mfoc_motor_t']
    status = motor + fields['irq_result']
    ref = 0x20018000
    fixed = numeric == 'fixed'

    def scalar(data):
        return unpack('<i', data)[0] / 32768 if fixed else unpack('<f', data)[0]

    # Observe the actual speed PI's feedback argument using the target ABI.
    # Install before any ADC frame translates the PI code in Unicorn.
    speed_samples = []
    pi = symbols['mfoc_pi_Step'] & ~1

    def read_speed_pi(_cpu, _address, _size, _data):
        if cpu.reg_read(UC_ARM_REG_R0) == motor + fields['speed']:
            reg = UC_ARM_REG_R2 if fixed else UC_ARM_REG_S1
            speed_samples.append(scalar(pack('<I', cpu.reg_read(reg))))

    cpu.hook_add(UC_HOOK_CODE, read_speed_pi, begin=pi, end=pi)
    command = pack('<ii', 0, 1638) if fixed else pack('<ff', 0.0, 0.05)
    cpu.mem_write(ref, command)
    assert call('mfoc_motor_SetCurrent', motor, ref) == 0
    assert call('mfoc_motor_Start', motor) == 0
    assert get(PWM + 0x44) & PWM_OUTPUT_ENABLE
    assert compare() == (2500, 2500, 2500)
    control_frame('current-first')
    assert get(status) == 0
    commanded = compare()
    assert len(set(commanded)) > 1
    assert timer.active == (2500, 2500, 2500)
    control_frame('current-running')
    assert timer.active == commanded and get(status) == 0
    commanded = compare()

    # The emulator does not model timer Break, so supply its register effects.
    put(GPIOB_IDT, HALL_H1)
    timer.timer_flags |= BREAK_LATCHED
    put(PWM + 0x44, get(PWM + 0x44) & ~PWM_OUTPUT_ENABLE)
    control_frame('break')
    assert get(status) != 0 and compare() == commanded
    assert call('mfoc_motor_ClearFault', motor) == 0
    assert call('mfoc_motor_Start', motor) != 0
    assert not get(PWM + 0x44) & PWM_OUTPUT_ENABLE
    assert timer.timer_flags & BREAK_LATCHED

    put(GPIOB_IDT, BREAK_INACTIVE | HALL_H1)
    assert not get(PWM + 0x44) & PWM_OUTPUT_ENABLE
    assert call('mfoc_motor_ClearFault', motor) == 0
    assert call('mfoc_motor_Start', motor) == 0
    assert not timer.timer_flags & BREAK_LATCHED
    assert get(PWM + 0x44) & PWM_OUTPUT_ENABLE
    assert call('mfoc_motor_Stop', motor) == 0
    assert not get(PWM + 0x44) & PWM_OUTPUT_ENABLE

    # The shell's speed command enters this application function in debug-rel.
    assert call('_mfoc_app_Speed', float_arg=0.125) == 0
    assert get(motor + fields['mode']) == 1  # MFOC_MOTOR_MODE_SPEED.
    assert get(motor + fields['state']) == 2  # MFOC_MOTOR_RUNNING.
    assert get(PWM + 0x44) & PWM_OUTPUT_ENABLE
    assert compare() == (2500, 2500, 2500)

    control_frame('speed-first')
    assert get(status) == 0 and speed_samples == [0.0], (
        get(status), speed_samples)
    stationary = compare()
    assert len(set(stationary)) > 1

    # Follow the next two forward sectors from the linked Hall calibration.
    # mfoc_hall_sector_t has one byte of code and one aligned BAM32 angle.
    table = 's_hall_sectors'
    assert image.sizes[table] == 48
    sectors = [unpack('<B3xI', cpu.mem_read(symbols[table] + i * 8, 8))
               for i in range(6)]
    start = next(i for i, (code, _) in enumerate(sectors) if code == 1)

    def read_capture(_cpu, _access, _address, _size, _value, _data):
        # On hardware, reading C1DT clears C1IF.
        put(HALL_TIMER + 0x10, get(HALL_TIMER + 0x10) & ~HALL_CAPTURE_FLAG)

    capture = HALL_TIMER + 0x34
    cpu.hook_add(UC_HOOK_MEM_READ, read_capture, begin=capture, end=capture)
    assert get(HALL_TIMER + 0x0C) & HALL_CAPTURE_FLAG
    interval_us = 5000
    epoch = symbols['s_lSystemClockCounts']
    for step in (1, 2):
        code, _ = sectors[(start + step) % 6]
        edge = (65532 + (step - 1) * interval_us) & 0xFFFF
        put(GPIOB_IDT, BREAK_INACTIVE | ((code & 3) << 4) | (code >> 2))
        put(capture, edge)
        put(HALL_TIMER + 0x24, (edge + 12) & 0xFFFF)
        put(HALL_TIMER + 0x10, HALL_CAPTURE_FLAG)
        cpu.mem_write(epoch, pack('<Q', step * interval_us * (CORE_HZ // 1_000_000)))
        call('TMR3_GLOBAL_IRQHandler')
        assert not get(HALL_TIMER + 0x10) & HALL_CAPTURE_FLAG
        assert call('mfoc_motor_Run', motor) == 0  # Consume the captured edge.

    assert call('mfoc_port_position', ref, ref + 4) == 0
    _, previous = sectors[(start + 1) % 6]
    _, anchor = sectors[(start + 2) % 6]
    _, next_anchor = sectors[(start + 3) % 6]
    measured_speed = scalar(cpu.mem_read(ref + 4, 4))
    speed_base = scalar(cpu.mem_read(symbols['s_hall_speed_base'], 4))
    width = ((anchor - previous) & 0xFFFFFFFF) / (1 << 32)
    expected_speed = width * 1_000_000 / (interval_us * speed_base)
    assert measured_speed > 0 and abs(measured_speed - expected_speed) < 1 / 32768
    assert ((get(ref) - anchor) & 0xFFFFFFFF) < (
        (next_anchor - anchor) & 0xFFFFFFFF)

    # Exercise a full control-divider period with the gate still enabled.
    divider = unpack('<H', cpu.mem_read(motor + fields['speed_divider'], 2))[0]
    assert divider > 1
    for frame in range(1, divider + 1):
        label = {1: 'speed-hold', divider: 'speed-update'}.get(frame)
        control_frame(label)
        assert get(status) == 0
        assert get(PWM + 0x44) & PWM_OUTPUT_ENABLE
    assert speed_samples == [0.0, measured_speed]
    regulated = compare()
    assert regulated != stationary
    assert len(set(regulated)) > 1 and all(0 < c < 5000 for c in regulated)
    assert call('mfoc_motor_Stop', motor) == 0
    assert not get(PWM + 0x44) & PWM_OUTPUT_ENABLE
    assert timer.software_updates == initial_updates

    # A stale PCCE while stopped cannot authorize a restart. Without a new
    # triggered conversion, no rising gate or first-frame write is permitted.
    opens = timer.gate_opens
    timer.adc_flags |= 4
    timer.deliver_adc = False
    assert call('_mfoc_app_Speed', float_arg=0.125) == 6  # IO
    assert not get(PWM + 0x44) & PWM_OUTPUT_ENABLE and timer.gate_opens == opens
    timer.deliver_adc = True
    assert call('mfoc_motor_ClearFault', motor) == 0

    for fault in ('phase', 'current-high', 'current-low', 'hall', 'missing', 'break', 'gate'):
        put(GPIOB_IDT, BREAK_INACTIVE | HALL_H1)
        put(ADC + 0x3C, 2048)
        assert call('_mfoc_app_Speed', float_arg=0.125) == 0
        original = compare()
        if fault in ('phase', 'current-high', 'current-low', 'hall'):
            if fault.startswith('current'):
                sign = 1 if fault == 'current-high' else -1
                put(ADC + 0x3C, 2048 + sign * 122)
                control_frame()
                assert get(status) == 0 and get(PWM + 0x44) & PWM_OUTPUT_ENABLE
                original = compare()
                put(ADC + 0x3C, 2048 + sign * 123)
            if fault == 'hall':
                put(GPIOB_IDT, BREAK_INACTIVE)  # Invalid Hall code 000.
            control_frame(irq_lag=750 if fault == 'phase' else 0)
            assert get(status) == 7
        else:
            if fault == 'missing':
                for _ in range(2):
                    call('SysTick_Handler')
                    assert get(PWM + 0x44) & PWM_OUTPUT_ENABLE
            elif fault == 'break':
                put(GPIOB_IDT, HALL_H1)
                timer.timer_flags |= BREAK_LATCHED
                put(PWM + 0x44, get(PWM + 0x44) & ~PWM_OUTPUT_ENABLE)
            else:
                put(PWM + 0x44, get(PWM + 0x44) & ~PWM_OUTPUT_ENABLE)
            call('SysTick_Handler')
            assert not get(PWM + 0x44) & PWM_OUTPUT_ENABLE
            assert call('mfoc_app_Run', app) != 0
        reason = {'phase': 1, 'current-high': 2, 'current-low': 2,
                  'hall': 3, 'missing': 4, 'break': 5, 'gate': 6}
        assert call('mfoc_at32_safety_GetFault') == reason[fault], fault
        assert get(motor + fields['state']) == 3 and compare() == original, fault
        assert not get(PWM + 0x44) & PWM_OUTPUT_ENABLE
        assert call('mfoc_motor_ClearFault', motor) == 0
    assert timer.software_updates == initial_updates

    # Resume foreground through the real Port return site after an IRQ. Both
    # the first command and a fault raised in this interval must survive.
    def immediate_frame():
        assert get(motor + fields['state']) == 2 and get(PWM + 0x44) & PWM_OUTPUT_ENABLE
        control_frame(stack=0x2001C000)

    put(GPIOB_IDT, BREAK_INACTIVE | HALL_H1)
    assert call('_mfoc_app_Speed', float_arg=0.125, interrupt_start=immediate_frame) == 0
    assert get(status) == 0 and len(set(compare())) > 1
    assert cpu.mem_read(motor + fields['previous_core_output_valid'], 1) == b'\x01'
    assert call('mfoc_motor_Stop', motor) == 0
    put(ADC + 0x3C, 2048 + 123)
    assert call('_mfoc_app_Speed', float_arg=0.125, interrupt_start=immediate_frame) == 7
    assert get(motor + fields['state']) == 3 and not get(PWM + 0x44) & PWM_OUTPUT_ENABLE
    assert timer.software_updates == initial_updates


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('numeric', choices=('float', 'fixed'))
    parser.add_argument('elf')
    parser.add_argument('--measure', action='store_true',
                        help='count instructions and software stack for selected complete ADC IRQs')
    args = parser.parse_args()
    replay(Image(args.elf), args.numeric, args.measure)
    print(f'AT32 {args.numeric}: natural top update, 24 V Hall control, restart and guards passed')
