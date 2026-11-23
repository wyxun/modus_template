"""Count true algorithm branch hits using the existing numerical test plants.

Run with python and matching native gcc/gcov on PATH. Counts represent host
replay coverage. The software counter build is never used for timing.
"""
import json
import os
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import tempfile

LIB = Path(__file__).resolve().parents[2]
INC = [LIB, LIB / 'include', *(d / 'include' for d in LIB.iterdir() if
                              (d / 'include').is_dir())]
ALG = sorted((LIB / 'math/src').glob('*.c')) + sorted((LIB / 'control/src').glob('*.c'))
SOURCES = {
    'flux': ('position/flux_algorithm', ['position/src/mfoc_flux.c']),
    'smo': ('position/smo', ['position/src/mfoc_smo.c']),
    'hfi': ('position/hfi_algorithm', ['position/src/mfoc_hfi.c']),
    'limit': ('core/core', ['core/src/mfoc_core.c']),
    'ident': ('motor/motor_identification_plant', [
        'tests/motor/motor_port_fake.c', 'core/src/mfoc_core.c',
        'motor/src/*.c', 'position/src/mfoc_flux.c',
        'identification/src/*.c', 'identification/src/inductance/*.c']),
}
# Find source lines by operation, not text offsets or assembler addresses.
MARKS = {
    'flux': [('position/src/mfoc_flux.c', 'out->valid = true;', 'valid')],
    'smo': [('position/src/mfoc_smo.c', 'out->angle = mfoc_angle_atan2(', 'angle'),
            ('position/src/mfoc_smo.c', 'smo->held_speed = mfoc_mul_wide(', 'window')],
    'hfi': [('position/src/mfoc_hfi.c', '_mfoc_hfi_Track(hfi, angle);', 'eighth_ok'),
            ('position/src/mfoc_hfi.c', 'if (hfi->state == MFOC_HFI_TRACK) {', 'eighth_weak')],
    'limit': [('core/src/mfoc_core.c', 'out->limited = _mfoc_core_limit(', 'steps')],
    'ident': [('identification/src/mfoc_identification.c', 'progress = mfoc_identification_rs_Step(', 'rs'),
              ('identification/src/mfoc_identification.c', 'mfoc_identification_inductance_SeriesStep(', 'ld_lq'),
              ('identification/src/mfoc_identification.c', 'progress = mfoc_identification_rotation_Step(', 'rotating')],
}


def run_case(case):
    profile, numeric = case
    test, globs = SOURCES[profile]
    marks = list(MARKS[profile])
    if profile == 'limit':
        expr = 'mfoc_scalar_t k = sqrtf(' if numeric == 'FLOAT' else 'uint32_t n = _mfoc_core_CeilSqrt('
        marks.append(('core/src/mfoc_core.c', expr, 'limited'))
    with tempfile.TemporaryDirectory(prefix='mfoc-p10-paths-') as tmp:
        work = Path(tmp)
        exe = work / ('probe.exe' if os.name == 'nt' else 'probe')
        sources = [LIB / 'tests' / (test + '_test.c'), *ALG]
        for pattern in globs:
            sources.extend(sorted(LIB.glob(pattern)))
        args = ['gcc', '-std=gnu11', '-O0', '--coverage', '-ffp-contract=off',
                '-DMFOC_NUMERIC_' + numeric + '=1', '-DMFOC_TRIG_BACKEND=1',
                '-DMFOC_FEEDBACK_BACKEND=0', '-DMFOC_FEEDBACK_ENABLE_FLUX=0', '-DMFOC_FEEDBACK_ENABLE_HFI=0',
                '-DMFOC_TEST_REAL_FEEDBACK=1', '-DMFOC_FEEDBACK_ENABLE_POSITION=0',
                '-DMFOC_IDENTIFICATION_ENABLE=' + str(int(profile == 'ident'))]
        subprocess.run([*args, *['-I' + str(p) for p in INC], *sources, '-lm',
                        '-o', exe], cwd=tmp, check=True)
        subprocess.run([exe], cwd=tmp, check=True, stdout=subprocess.DEVNULL)
        counts = {}
        cache = {}
        for source, operation, label in marks:
            path = LIB / source
            if source not in cache:
                notes = next(work.glob('*-' + path.stem + '.gcno'))
                data = subprocess.check_output(['gcov', '-j', '-t', notes], cwd=tmp)
                record = next(f for f in json.loads(data)['files']
                              if Path(f['file']).name == path.name)
                cache[source] = {l['line_number']: l['count'] for l in record['lines']}
            matches = [i for i, line in enumerate(path.read_text(encoding='utf8').splitlines(), 1)
                       if operation in line]
            assert len(matches) == 1, (operation, matches)
            counts[label] = cache[source][matches[0]]
            assert counts[label] > 0, (numeric, label)
        return numeric, profile, counts


if __name__ == '__main__':
    with ThreadPoolExecutor(max_workers=2) as pool:
        for numeric, profile, counts in pool.map(run_case, [
                (p, n) for n in ('FLOAT', 'FIXED') for p in SOURCES]):
            print(numeric, profile, counts, flush=True)
