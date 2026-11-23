"""Replay an AT32F413 Hall release ELF with pyelftools and Unicorn 2."""
import argparse
from struct import pack, unpack

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn import UC_HOOK_CODE, UC_HOOK_MEM_READ
from unicorn.arm_const import (UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC,
                               UC_ARM_REG_R0, UC_ARM_REG_PRIMASK,
                               UC_CPU_ARM_CORTEX_M4)

TIMER = 0x40000400
GPIOB = 0x40010C00
SYSTICK = 0xE000E010
ICSR = 0xE000ED04
STACK = 0x20007FF0
RETURN = 0x080FF000


def replay(elf, pending=False, preempt=False, overcapture=False):
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
    for address, size in [(0x08000000, 0x100000), (0x20000000, 0x10000),
                          (0x40000000, 0x40000), (0xE000E000, 0x2000)]:
        cpu.mem_map(address, size)
    for seg in elf.iter_segments():
        if seg['p_type'] == 'PT_LOAD' and seg.data():
            cpu.mem_write(seg['p_vaddr'], seg.data())
    symbols = {s.name: s['st_value'] for s in
               elf.get_section_by_name('.symtab').iter_symbols()}

    def put(addr, value):
        cpu.mem_write(addr, pack('<I', value))

    def get(addr):
        return unpack('<I', cpu.mem_read(addr, 4))[0]

    def call(name):
        cpu.reg_write(UC_ARM_REG_LR, RETURN | 1)
        cpu.reg_write(UC_ARM_REG_SP, STACK)
        cpu.emu_start(symbols[name] | 1, RETURN, count=5000)
        assert cpu.reg_read(UC_ARM_REG_PC) == RETURN, (name, hex(cpu.reg_read(UC_ARM_REG_PC)))

    # Exercise the actual Hall pin remap, XOR, prescaler and IRQ priority.
    # Supply clock-ready responses and skip unrelated analogue calibration.
    skip = {symbols[name] & ~1: result for name, result in [
        ('crm_reset', 0), ('crm_flag_get', 1), ('crm_hext_stable_wait', 1),
        ('crm_sysclk_switch', 0), ('crm_sysclk_switch_status_get', 2), ('haladc_Init', 0),
        ('halusart_Init', 0), ('system_core_clock_update', 0)]}

    def board_setup(_, addr, _size, _ctx):
        if addr in skip:
            cpu.reg_write(UC_ARM_REG_R0, skip[addr])
            cpu.reg_write(UC_ARM_REG_PC, cpu.reg_read(UC_ARM_REG_LR))

    hook = cpu.hook_add(UC_HOOK_CODE, board_setup)
    put(symbols['system_core_clock'], 200000000)
    call('peripheral_Init')
    cpu.hook_del(hook)
    assert (get(0x40010024) >> 8) & 0xf == 2  # TMR3 PB4/PB5/PB0
    assert (get(0x40010030) >> 16) & 7 == 2   # JTAG off, SWD retained
    assert get(TIMER + 0x28) == 199 and get(TIMER + 0x2C) == 65535
    assert get(TIMER) & 1 and get(TIMER + 4) & (1 << 7)
    assert get(TIMER + 8) & 0x77 == 0x40  # C1INC, reset disabled
    assert get(TIMER + 0x18) & 0xff == 3 and get(TIMER + 0x20) & 0xf == 1
    assert get(TIMER + 0xC) == 0
    assert cpu.mem_read(0xE000E400 + 29, 1) == b'\xf0'
    assert cpu.mem_read(0xE000ED23, 1) == b'\xf0'

    # The memory model handles C1DT read-clear and a 20 us ADC preemption.
    put(TIMER + 0xC, 2)
    put(TIMER + 0x10, 0x202 if overcapture else 2)
    put(TIMER + 0x24, 8)
    put(TIMER + 0x34, 65532)  # edge -> clock read = 12 us across wrap
    put(GPIOB + 8, 1 << 5)
    put(SYSTICK + 4, 199999)
    put(SYSTICK + 8, 99999)
    put(ICSR, (1 << 26) if pending else 0)
    put(symbols['s_hall_status'], 0)
    count, depth, masked, masked_peak, clock_reads = 0, 0, 0, 0, 0

    def instruction(_, _addr, _size, _ctx):
        nonlocal count, depth, masked, masked_peak
        count += 1
        depth = max(depth, STACK - cpu.reg_read(UC_ARM_REG_SP))
        masked = masked + 1 if cpu.reg_read(UC_ARM_REG_PRIMASK) & 1 else 0
        masked_peak = max(masked_peak, masked)

    def read(_, _kind, addr, _size, _value, _ctx):
        nonlocal clock_reads
        if addr == TIMER + 0x34:
            put(TIMER + 0x10, get(TIMER + 0x10) & ~2)
        elif addr == TIMER + 0x24:
            clock_reads += 1
            if preempt and clock_reads == 2:
                put(TIMER + 0x24, 28)
                put(SYSTICK + 8, 95999)

    cpu.hook_add(UC_HOOK_CODE, instruction)
    cpu.hook_add(UC_HOOK_MEM_READ, read, begin=TIMER, end=TIMER + 0x3f)
    call('TMR3_GLOBAL_IRQHandler')
    if overcapture:
        assert get(symbols['s_hall_status']) == 7
        assert get(TIMER + 0xC) & 2 == 0
    else:
        assert clock_reads == (4 if preempt else 2)
        assert get(symbols['s_hall_status']) == 0
        tick = unpack('<Q', cpu.mem_read(symbols['s_edges'], 8))[0]
        # The active modus SysTick port returns LOAD+1 as top, and perfc
        # adds a tick. Allow the documented one-tick pending epoch bias.
        expected = 100000 + 200000 * pending - 12 * 200
        assert tick - expected in ({0, 1} if pending else {0})
    return count, depth, masked_peak


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=argparse.FileType('rb'))
    image = ELFFile(parser.parse_args().elf)
    for label, kwargs in [('edge', {}), ('SysTick pending', {'pending': True}),
                          ('ADC preemption + pending', {'pending': True, 'preempt': True}),
                          ('overcapture', {'overcapture': True})]:
        count, stack, masked = replay(image, **kwargs)
        print(f'{label}: {count} instructions, {stack} B stack, '
              f'{masked} consecutive instructions with PRIMASK set')
