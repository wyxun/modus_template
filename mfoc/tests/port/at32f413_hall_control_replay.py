"""Replay the linked 24 V AT32F413 Hall debug-rel firmware.

The model supplies synthetic stationary Hall inputs and ADC register values.
It verifies firmware sequencing, including the software response to a
simulated hardware Break. PWM timing and real Hall mapping need the bench.
"""
import argparse
from struct import pack, unpack

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn import UC_HOOK_MEM_READ
from unicorn.arm_const import (
    UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC, UC_ARM_REG_PC, UC_ARM_REG_SP,
    UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_CPU_ARM_CORTEX_M4,
)

from runtime_bus_irq_replay import Image

ADC = 0x40012400
PWM = 0x40012C00
GPIOB_IDT = 0x40010C08
HALL_H1 = 1 << 4
BREAK_INACTIVE = 1 << 12
BREAK_LATCHED = 1 << 7
PWM_OUTPUT_ENABLE = 1 << 15
RETURN = 0x0807F000


def replay(image, numeric):
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

    def put(address, value):
        cpu.mem_write(address, pack('<I', value))

    def get(address):
        return unpack('<I', cpu.mem_read(address, 4))[0]

    def call(name, arg0=0, arg1=0):
        cpu.reg_write(UC_ARM_REG_SP, 0x20010000)
        cpu.reg_write(UC_ARM_REG_LR, RETURN | 1)
        cpu.reg_write(UC_ARM_REG_R0, arg0)
        cpu.reg_write(UC_ARM_REG_R1, arg1)
        cpu.emu_start(symbols[name] | 1, RETURN, count=100000)
        assert cpu.reg_read(UC_ARM_REG_PC) == RETURN, name
        return cpu.reg_read(UC_ARM_REG_R0)

    def read_adc_status(_cpu, _access, _address, _size, _value, _data):
        put(ADC, 4)  # Synthesized preempted-group frame completion.

    def compare():
        return tuple(get(PWM + offset) for offset in (0x34, 0x38, 0x3C))

    put(symbols['system_core_clock'], 200000000)
    put(0xE000E014, 199999)  # SysTick at 1 kHz.
    put(0xE000E018, 99999)
    put(GPIOB_IDT, BREAK_INACTIVE | HALL_H1)
    put(0x40020000, 2)  # DMA1 regular-group transfer complete.
    put(symbols['g_ahwAt32AdcOrdinary'], 1536)  # 24 V.
    put(PWM + 0x2C, 5000)
    for offset in (0x3C, 0x40, 0x44):
        put(ADC + offset, 2048)
    cpu.hook_add(UC_HOOK_MEM_READ, read_adc_status, begin=ADC, end=ADC)
    app = symbols['tMfocApp']
    assert call('mfoc_app_Init', app, symbols['tMfocAppCfg']) == 0
    assert not get(PWM + 0x44) & PWM_OUTPUT_ENABLE

    port_base = image.fields['at32_mfoc_port_t']['voltage_base_millivolt']
    assert get(symbols['s_port'] + port_base) == 24000
    motor = app + image.fields['mfoc_app_t']['motor']
    status = motor + image.fields['mfoc_motor_t']['irq_result']
    ref = 0x20018000
    command = pack('<ii', 0, 1638) if numeric == 'fixed' else pack(
        '<ff', 0.0, 0.05)
    cpu.mem_write(ref, command)
    assert call('mfoc_motor_SetCurrent', motor, ref) == 0
    assert call('mfoc_motor_Start', motor) == 0
    assert get(PWM + 0x44) & PWM_OUTPUT_ENABLE
    assert compare() == (2500, 2500, 2500)
    call('ADC1_2_IRQHandler')
    assert get(status) == 0
    commanded = compare()
    assert len(set(commanded)) > 1

    # The emulator does not model timer Break, so supply its register effects.
    put(GPIOB_IDT, HALL_H1)
    put(PWM + 0x10, BREAK_LATCHED)
    put(PWM + 0x44, get(PWM + 0x44) & ~PWM_OUTPUT_ENABLE)
    call('ADC1_2_IRQHandler')
    assert get(status) != 0 and compare() == commanded
    assert call('mfoc_motor_ClearFault', motor) == 0
    assert call('mfoc_motor_Start', motor) != 0
    assert not get(PWM + 0x44) & PWM_OUTPUT_ENABLE
    assert get(PWM + 0x10) & BREAK_LATCHED

    put(GPIOB_IDT, BREAK_INACTIVE | HALL_H1)
    assert not get(PWM + 0x44) & PWM_OUTPUT_ENABLE
    assert call('mfoc_motor_ClearFault', motor) == 0
    assert call('mfoc_motor_Start', motor) == 0
    assert not get(PWM + 0x10) & BREAK_LATCHED
    assert get(PWM + 0x44) & PWM_OUTPUT_ENABLE
    assert call('mfoc_motor_Stop', motor) == 0
    assert not get(PWM + 0x44) & PWM_OUTPUT_ENABLE


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('numeric', choices=('float', 'fixed'))
    parser.add_argument('elf')
    args = parser.parse_args()
    replay(Image(args.elf), args.numeric)
    print(f'AT32 {args.numeric}: 24 V Hall control, Break and restart passed')
