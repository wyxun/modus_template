"""Exercise the linked witness using modeled timer and NVIC registers.

Real exception scheduling is a bench observation. Here the fixture controls
when the witness runs, which frame pended it, and ADC active state.
"""
import argparse
import struct

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_PRIMASK,
                               UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2,
                               UC_ARM_REG_R3, UC_ARM_REG_SP, UC_CPU_ARM_CORTEX_M4)

from g431_p10_exit import MASK, SNAPSHOT_SIZE, decode

TIM2, NVIC, AIRCR, RETURN = 0x40000000, 0xe000e100, 0xe000ed0c, 0x0807f000
ADC_IRQ, TIM2_IRQ = 18, 28
CNT, PEND, PERIOD = TIM2 + 0x24, NVIC + 0x100, 8500
SPEED, SMO, HFI, LIMIT, BUS, ABNORMAL = (1 << i for i in range(6))


class Fixture:
    def __init__(self, elf):
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_MCLASS | UC_MODE_THUMB)
        self.cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
        for start, size in ((0x08000000, 0x80000), (0x20000000, 0x20000),
                            (TIM2, 0x1000), (0xe000e000, 0x1000)):
            self.cpu.mem_map(start, size)
        for section in elf.iter_sections():
            if section['sh_flags'] & 2 and section['sh_type'] == 'SHT_PROGBITS':
                self.cpu.mem_write(section['sh_addr'], section.data())
        self.symbols = {sym.name: sym['st_value'] for sym in
                        elf.get_section_by_name('.symtab').iter_symbols()}
        self.put(AIRCR, 3 << 8)
        self.put(NVIC + 0x300 + ADC_IRQ, 0x10, 'B')
        self.call('mfoc_p10_exit_Init')
        assert self.get(NVIC + 0x300 + TIM2_IRQ, 'B') == 0x20
        assert self.get(NVIC) & (1 << TIM2_IRQ)

    def put(self, address, value, fmt='I'):
        self.cpu.mem_write(address, struct.pack('<' + fmt, value & MASK))

    def get(self, address, fmt='I'):
        return struct.unpack('<' + fmt, self.cpu.mem_read(
            address, struct.calcsize(fmt)))[0]

    def call(self, name, r0=0, r1=0, r2=0, r3=0):
        for reg, value in ((UC_ARM_REG_SP, 0x2001f000), (UC_ARM_REG_LR, RETURN | 1),
                           (UC_ARM_REG_R0, r0), (UC_ARM_REG_R1, r1),
                           (UC_ARM_REG_R2, r2), (UC_ARM_REG_R3, r3)):
            self.cpu.reg_write(reg, value)
        self.cpu.emu_start(self.symbols[name] | 1, RETURN, count=50000)
        assert self.cpu.reg_read(UC_ARM_REG_PC) == RETURN

    def arm(self, seq, trigger, flags):
        self.put(CNT, trigger + 600)
        self.put(PEND, 0)
        self.call('mfoc_p10_exit_Pend', seq, trigger, PERIOD, flags)
        assert self.get(PEND) == 1 << TIM2_IRQ

    def observe(self, stamp, *, active=False):
        self.put(CNT, stamp)
        self.put(NVIC + 0x200, (1 << ADC_IRQ) if active else 0)
        self.call('TIM2_IRQHandler')

    def sample(self, seq, trigger, flags, age):
        self.arm(seq, trigger, flags)
        self.observe(trigger + age)

    def evidence(self):
        raw = bytes(self.cpu.mem_read(self.symbols['g_p10_exit_witness'],
                                     SNAPSHOT_SIZE))
        return decode(raw, PERIOD)


def replay(elf):
    f = Fixture(elf)
    f.sample(1, 10000, LIMIT, 3500)
    f.sample(2, 18500, SPEED | SMO | LIMIT, 3900)
    f.sample(3, 27000, BUS, 2100)
    assert f.evidence()['groups']['bus_speed_smo_limit']['count'] == 0
    f.sample(4, 35500, BUS | SPEED | SMO | LIMIT, 4600)
    f.sample(5, 44000, ABNORMAL, 1300)
    result = f.evidence()
    assert result['complete'] and result['witness_deadline_pass']
    groups = result['groups']
    assert groups['all']['count'] == 5 and groups['all']['worst']['seq'] == 4
    assert groups['limit_only']['count'] == 1
    assert groups['speed_smo_limit']['count'] == 2
    assert groups['bus_speed_smo_limit']['count'] == 1
    assert groups['abnormal']['count'] == 1

    f.cpu.reg_write(UC_ARM_REG_PRIMASK, 1)
    f.call('mfoc_p10_exit_Reset')
    assert f.cpu.reg_read(UC_ARM_REG_PRIMASK) == 1
    f.sample(1, 0xfffffff0, HFI | SPEED | BUS, PERIOD - 65)
    f.sample(2, 8484, SPEED | SMO | LIMIT, PERIOD - 64)
    result = f.evidence()
    assert result['complete'] and not result['witness_deadline_pass']
    assert result['groups']['all']['late'] == 1
    assert result['groups']['bus_speed_hfi_eighth']['guarded_trigger_margin_ticks'] == 1
    assert result['groups']['all']['guarded_trigger_margin_ticks'] == 0
    assert f.cpu.reg_read(UC_ARM_REG_PRIMASK) == 1

    f = Fixture(elf)
    f.observe(10700)  # Stray interrupt without a request.
    assert f.evidence()['groups']['all']['count'] == 0
    f.arm(1, 10000, SPEED)
    assert f.evidence()['outstanding'] == 1
    f.arm(2, 18500, LIMIT)  # Next ADC claimed this slot before the old witness.
    f.observe(18500 + 665)
    f.arm(3, 27000, BUS)
    f.observe(27000 + 599)  # Invalid timestamp before the pending request.
    f.arm(4, 35500, SMO)
    f.observe(35500 + 2000, active=True)
    f.put(NVIC + 0x300 + TIM2_IRQ, 0x00, 'B')
    f.sample(5, 44000, HFI, 2000)
    result = f.evidence()
    assert not result['complete'] and not result['witness_deadline_pass']
    for key in ('replaced', 'bad_compare', 'adc_active', 'bad_priority'):
        assert result[key] == 1, (key, result[key])
    assert result['armed'] == 5 and result['outstanding'] == 0
    assert result['groups']['all']['count'] == 1
    print('TIM2 witness, exact collision groups, guarded boundary, wrap, '
          'pending replacement, active ADC, invalid stamp and NVIC checks passed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=argparse.FileType('rb'))
    args = parser.parse_args()
    replay(ELFFile(args.elf))
