"""Count executed G431 Motor.Step Thumb instructions from endpoint ELFs.

Input is a build_g431_p10_bench.ps1 package directory. No physical cycles.
"""
import argparse
from collections import Counter
from functools import lru_cache
from pathlib import Path
import struct
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB, UC_HOOK_CODE, UC_HOOK_MEM_WRITE
from unicorn.arm_const import (UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC, UC_ARM_REG_SP,
    UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_PC, UC_CPU_ARM_CORTEX_M4)
import sys
sys.dont_write_bytecode = True
from g431_p10_probe_replay import field_offsets
from runtime_bus_irq_replay import Image, RETURN, prepare_g431_delayed_port

TIM, ADC = 0x40012c00, 0x50000000


def replay(image, fields, numeric, case):
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
    cpu.reg_write(UC_ARM_REG_C1_C0_2, 0xf00000)
    cpu.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
    for addr, size in [(0x08000000, 0x80000), (0x20000000, 0x20000),
                       (0x40000000, 0x40000), (ADC, 0x1000), (0xe0000000, 0x10000)]:
        cpu.mem_map(addr, size)
    for addr, data in image.segments:
        cpu.mem_write(addr, data)
    sym = image.symbols

    def put(addr, value, fmt='I'):
        cpu.mem_write(addr, struct.pack('<' + fmt, value))

    def pu(addr, x):
        if numeric == 'fixed':
            put(addr, round(x * 32768), 'i')
        else:
            put(addr, x, 'f')

    def call(name, a=0, b=0):
        for reg, value in [(UC_ARM_REG_SP, 0x20010000), (UC_ARM_REG_LR, RETURN | 1),
                           (UC_ARM_REG_R0, a), (UC_ARM_REG_R1, b)]:
            cpu.reg_write(reg, value)
        cpu.emu_start(sym[name] | 1, RETURN, count=100000)
        assert cpu.reg_read(UC_ARM_REG_PC) == RETURN
        return cpu.reg_read(UC_ARM_REG_R0)

    m = fields('mfoc_motor_t')
    motor = sym['tMfocApp'] + fields('mfoc_app_t')['motor']
    cfg = sym['tMfocAppCfg'] + fields('mfoc_app_cfg_t')['motor']
    def calibration(cpu, pc, size, data):
        cpu.reg_write(UC_ARM_REG_R0, 0)
        cpu.reg_write(UC_ARM_REG_PC, cpu.reg_read(UC_ARM_REG_LR))
    addr = sym['mfoc_port_init'] & ~1
    hk = cpu.hook_add(UC_HOOK_CODE, calibration, begin=addr, end=addr)
    assert call('mfoc_motor_Init', motor, cfg) == 0
    cpu.hook_del(hk)
    f = fields('mfoc_mdi_port_t')
    port = sym['s_port']
    put(port + f['configured'], 1, 'B')
    for key, reg in zip(('offset_u','offset_v','offset_w'), (0x80,0x184,0x180)):
        put(port + f[key], 8192)
        put(ADC + reg, 8192)
    if numeric == 'fixed':
        put(port + f['gain_q30'], round(-(1 << 30) / 476), 'i')
    else:
        put(port + f['gain'], -1 / 476, 'f')
    put(TIM + 0x2c, 4250)
    prepare_g431_delayed_port(put, sym, 34000, 8500)
    put(ADC, 64)
    put(ADC + 0x100, 64)
    now = call('mfoc_port_tick')
    enc = port + f['encoder']
    fe = fields('mfoc_encoder_t')
    put(enc + fe['tick_hz'], 170000000)
    put(enc + fe['timeout_ticks'], 1700000)
    put(enc + fe['pole_pairs'], 7, 'H')
    snap = enc + fe['samples']
    fs = fields('mfoc_encoder_sample_t')
    pu(snap + fs['speed'], .5)
    # Place sample with age 170 ticks, in the same modular timestamp domain.
    put(snap + fs['tick'], (now - 170) % (1 << 64), 'Q')
    put(motor + m['state'], 2)
    put(motor + m['mode'], 1)
    put(motor + m['previous_voltage_ab_valid'], 1, 'B')
    put(motor + m['speed_count'], 0 if 'speed' in case else 1, 'H')
    pu(motor + m['speed_ref'], .03125)
    fb = motor + m['feedback']
    ff = fields('mfoc_feedback_t')
    if case.startswith('flux'):
        flux = fb + ff['flux']
        pu(flux + fields('mfoc_flux_t')['psi_s'], .125)
    elif case.startswith('smo'):
        smo = fb + ff['smo']
        sm = fields('mfoc_smo_t')
        span = cpu.mem_read(smo + sm['c'] + fields('mfoc_smo_coeff_t')['speed_span'], 1)[0]
        assert span > 1
        pu(smo + sm['axis'] + fields('mfoc_smo_axis_t')['e_hat'], .0625)
        pu(smo + sm['held_speed'], .03125)
        put(smo + sm['has_angle'], 1, 'B')
        put(smo + sm['speed_count'], span-1 if 'window' in case else 0, 'B')
    elif case.startswith('hfi'):
        hfi = fb + ff['hfi']
        h = fields('mfoc_hfi_t')
        put(fb + ff['hfi_mode'], 2)
        put(hfi + h['state'], 1)
        put(hfi + h['lock_count'], 64, 'H')
        put(hfi + h['carrier'], 7, 'B')
        if 'applied_tag' in h:
            put(hfi + h['applied_tag'], 8, 'B')
            put(hfi + h['pending_tag'], 1, 'B')
        put(hfi + h['has_current'], 1, 'B')
        put(hfi + h['has_angle'], 1, 'B')
        for off, value in ((h['delta_u'], .125), (h['delta_u']+12, .125),
                           (h['delta_i'], .0625), (h['delta_i']+12, .03125)):
            pu(hfi + off, value)
    else:
        idp = motor + m['identification']
        ident = fields('mfoc_identification_t')
        put(idp + ident['state'], 1)
        put(idp + ident['stage'], 0x22)
        pu(idp + ident['current_limit'], .12)
        put(idp + ident['work'] + fields('mfoc_identification_rs_t')['count'], 4500)
    if case == 'flux_limit':
        put(motor + m['mode'], 0)
        pu(motor + m['current_cmd'], 1.0)
        pu(motor + m['current_cmd'] + 4, 1.0)
    put(sym['s_active'], 1, 'B')

    durations, calls = Counter(), Counter()
    stack = []
    total = [0]
    low = [0x20010000]
    funcs = ('mfoc_port_current','mfoc_port_position','mfoc_feedback_Step','mfoc_flux_Step',
        'mfoc_smo_Step','mfoc_hfi_Step','mfoc_core_Step','mfoc_port_pwm',
        'mfoc_identification_SampleStage','mfoc_encoder_Step','mfoc_pi_Step')
    tracked = {sym[name] & ~1: name for name in funcs if name in sym}
    def count(cpu, pc, size, arg):
        while stack and pc == stack[-1][1]:
            name, _, start = stack.pop()
            durations[name] += total[0] - start
            if name == 'mfoc_encoder_Step':
                assert cpu.reg_read(UC_ARM_REG_R0) == 0
        if pc in tracked:
            name = tracked[pc]
            if name == 'mfoc_pi_Step' and cpu.reg_read(UC_ARM_REG_R0) == motor + m['speed']:
                name = 'speed_pi'
            calls[name] += 1
            stack.append((name, cpu.reg_read(UC_ARM_REG_LR) & ~1, total[0]))
        total[0] += 1
        low[0] = min(low[0], cpu.reg_read(UC_ARM_REG_SP))
    def store(cpu, access, addr, size, value, data):
        durations['to_CCR3'] = total[0]
    cpu.hook_add(UC_HOOK_CODE, count)
    cpu.hook_add(UC_HOOK_MEM_WRITE, store, begin=TIM+0x3c, end=TIM+0x3c)
    cpu.ctl_flush_tb()
    rc = call('mfoc_motor_Step', motor)
    assert rc == 0 and calls['mfoc_port_pwm'] == 1, (case, rc, calls)
    assert ('speed' in case) == bool(calls['speed_pi'])
    flags = int.from_bytes(cpu.mem_read(sym['g_p10_events'], 4), 'little')
    assert bool(flags & 2) == ('window' in case), (case, flags)
    assert bool(flags & 4) == case.startswith('hfi'), (case, flags)
    if case == 'flux_limit':
        lo = fields('mfoc_core_output_t')['limited']
        assert cpu.mem_read(motor + m['previous_core_output'] + lo, 1) == b'\x01'
    obs = sum(durations[n] for n in ('mfoc_smo_Step','mfoc_flux_Step','mfoc_hfi_Step',
                                     'mfoc_identification_SampleStage'))
    return [numeric, case, total[0], durations['to_CCR3'], durations['mfoc_port_current'],
            durations['mfoc_port_position'], obs, durations['speed_pi'],
            durations['mfoc_core_Step'], durations['mfoc_port_pwm'], 0x20010000-low[0]]


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('package', type=Path)
    p.add_argument('--phase', choices=['A','B'], default='A')
    args = p.parse_args()
    print('numeric,case,MotorStep,until_CCR3,current,position,observer_or_id,speedPI,core,portPWM,sw_stack')
    for numeric in ('float','fixed'):
        for profile, cases in [('flux', ('flux','flux_speed','flux_limit')),
                               ('smo', ('smo','smo_window','smo_window_speed')),
                               ('hfi', ('hfi_eighth_speed',)), ('identification', ('rs',))]:
            path = args.package / f'{args.phase}-endpoints-{numeric}-{profile}' / 'template.elf'
            image = Image(path)
            with path.open('rb') as file:
                elf = ELFFile(file)
                fields = lru_cache(None)(lambda name: field_offsets(elf, name))
                for case in cases:
                    print(*replay(image, fields, numeric, case), sep=',', flush=True)
