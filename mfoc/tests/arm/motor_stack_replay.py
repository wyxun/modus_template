"""Measure stack high-water mark and executed instructions in linked ARM smoke tests."""
import argparse

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (
    UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC, UC_ARM_REG_LR, UC_ARM_REG_PC,
    UC_ARM_REG_R0, UC_ARM_REG_SP, UC_CPU_ARM_CORTEX_M0, UC_CPU_ARM_CORTEX_M4,
)


def replay(stream, core, names):
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4 if core == "m4"
                          else UC_CPU_ARM_CORTEX_M0)
    if core == "m4":
        cpu.reg_write(UC_ARM_REG_C1_C0_2, 0xf00000)
        cpu.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
    # lld's bare-metal default script places the smoke ELF in the low block.
    cpu.mem_map(0, 0x100000)
    stack_top = 0x20010000
    cpu.mem_map(0x20000000, 0x20000)
    elf = ELFFile(stream)
    for segment in elf.iter_segments():
        if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
            cpu.mem_write(segment["p_vaddr"], segment.data())
    symbols = {entry.name: entry["st_value"]
               for entry in elf.get_section_by_name(".symtab").iter_symbols()}
    addresses = {symbols[name] & ~1: name for name in names}
    stats = {name: [] for name in names}
    active = {}

    def measure(uc, pc, _size, _context):
        sp = uc.reg_read(UC_ARM_REG_SP)
        for name, (start, return_address) in tuple(active.items()):
            if pc == return_address:
                del active[name]
            else:
                result = stats[name][-1]
                result[0] = max(result[0], start - sp)
                result[1] += 1
        if pc in addresses:
            name = addresses[pc]
            assert name not in active, "recursive measurement is unsupported"
            active[name] = (sp, uc.reg_read(UC_ARM_REG_LR) & ~1)
            stats[name].append([0, 1])

    stop = 0xff000
    cpu.hook_add(UC_HOOK_CODE, measure)
    cpu.reg_write(UC_ARM_REG_SP, stack_top)
    cpu.reg_write(UC_ARM_REG_LR, stop | 1)
    cpu.emu_start(symbols["main"] | 1, stop, count=200000)
    assert cpu.reg_read(UC_ARM_REG_PC) == stop
    assert cpu.reg_read(UC_ARM_REG_R0) == 0, "ARM smoke fixture failed"
    assert not active and all(stats.values())
    return stats


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=argparse.FileType("rb"))
    parser.add_argument("--core", choices=("m0", "m4"), required=True)
    parser.add_argument("--function", action="append",
                        help="Function to measure, repeat for multiple names.")
    options = parser.parse_args()
    results = replay(options.elf, options.core,
                     options.function or ["mfoc_motor_Init"])
    for symbol, invocations in results.items():
        for index, (stack, instructions) in enumerate(invocations, 1):
            print(f"{symbol} ({index}): {stack} B nested stack, "
                  f"{instructions} instructions")
