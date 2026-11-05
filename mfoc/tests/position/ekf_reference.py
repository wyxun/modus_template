"""Independent float64 EKF oracle and FLOAT C numerical verification.

Run: python mfoc/tests/position/ekf_reference.py [--long-steps 1000000]
Requires NumPy and gcc. Builds only a host DLL in a temporary directory.
Noise values below are synthetic fixtures, not board characterization.
"""
import argparse
import ctypes as ct
import json
import math
from pathlib import Path
import subprocess
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
TAU = 2 * math.pi
DT = 1 / 20000
BASE = 400.0
RS = 2.55 * 3.5 / 12
LS = 0.00086 * TAU * BASE * 3.5 / 12
FLUX = 0.0035 * TAU * BASE / 12
KI = 1 - TAU * BASE * DT * RS / LS
KU = TAU * BASE * DT / LS
KE = FLUX * KU
KT = BASE * DT
UPPER = np.triu_indices(4)


def predict(x, u):
    a, b, w, th = x
    sine, cosine = np.sin(TAU * th), np.cos(TAU * th)
    return np.array([KI*a + KU*u[0] + KE*w*sine,
                     KI*b + KU*u[1] - KE*w*cosine, w, th + KT*w])


def jacobian(x):
    w, th = x[2:]
    s, c = np.sin(TAU * th), np.cos(TAU * th)
    return np.array([[KI, 0, KE*s, TAU*KE*w*c],
                     [0, KI, -KE*c, TAU*KE*w*s],
                     [0, 0, 1, 0], [0, 0, KT, 1]])


def batch(x, p, z, r):
    h = np.eye(2, 4)
    k = np.linalg.solve(h @ p @ h.T + r, h @ p).T
    x = x + k @ (z - h @ x)
    b = np.eye(4) - k @ h
    return x, b @ p @ b.T + k @ r @ k.T


def sequential(x, p, z, r, stale_second=False):
    prior = x.copy()
    rho = r[0, 1] / r[0, 0]
    for h, value, noise in [
            (np.array([1, 0, 0, 0]), z[0], r[0, 0]),
            (np.array([-rho, 1, 0, 0]), z[1]-rho*z[0],
             r[1, 1]-rho*r[0, 1])]:
        v = p @ h
        k = v / (h @ v + noise)
        measured_state = prior if stale_second and h[1] else x
        x = x + k * (value - h @ measured_state)
        b = np.eye(4) - np.outer(k, h)
        p = b @ p @ b.T + noise * np.outer(k, k)
    return x, p


def math_checks():
    rng = np.random.default_rng(20261031)
    fd_error, mean_error, cov_error = 0.0, 0.0, 0.0
    stale_error = 0.0
    for th in [0, 0.25, 0.5, 0.75, 1-1e-8, 1e-8]:
        for w in [-0.6, -0.1, 0, 0.1, 0.6]:
            x = np.array([0.04, -0.02, w, th])
            eps = np.eye(4) * 1e-6
            fd = np.array([(predict(x+e, [0.1, 0.2]) -
                            predict(x-e, [0.1, 0.2])) / 2e-6
                           for e in eps]).T
            fd_error = max(fd_error, np.max(np.abs(fd-jacobian(x))))
    for _ in range(200):
        a = rng.normal(size=(4, 4))
        p = a @ a.T + np.eye(4)*0.01
        a = rng.normal(size=(2, 2))
        r = a @ a.T + np.eye(2)*0.001
        x, z = rng.normal(size=4), rng.normal(size=2)
        xb, pb = batch(x, p, z, r)
        xs, ps = sequential(x, p, z, r)
        stale, _ = sequential(x, p, z, r, stale_second=True)
        stale_error = max(stale_error, np.max(np.abs(stale-xb)))
        mean_error = max(mean_error, np.max(np.abs(xb-xs)))
        cov_error = max(cov_error, np.max(np.abs(pb-ps)))
        assert np.linalg.eigvalsh(ps).min() >= -1e-12
    assert fd_error < 1e-7
    assert mean_error < 1e-10 and cov_error < 1e-10
    assert stale_error > 0.01  # Negative control catches a stale residual.
    return dict(jacobian_max=fd_error, sequential_mean_max=mean_error,
                sequential_cov_max=cov_error, stale_second_error=stale_error)


class AB(ct.Structure):
    _fields_ = [('a', ct.c_float), ('b', ct.c_float)]


class Cfg(ct.Structure):
    _fields_ = [('fs', ct.c_uint32), ('base', ct.c_float),
                ('q', ct.c_float*4), ('r', ct.c_float*3),
                ('p0', ct.c_float*4), ('min_speed', ct.c_float),
                ('max_angle_var', ct.c_float), ('max_speed_var', ct.c_float)]


class Coeff(ct.Structure):
    _fields_ = [(v, ct.c_float) for v in ('ki', 'ku', 'ke', 'kt')] + [
        ('q', ct.c_float*4), ('p0', ct.c_float*4)] + [
        (v, ct.c_float) for v in ('r1', 'r2', 'rho', 'min_speed',
                                 'max_angle_var', 'max_speed_var')]


class Model(ct.Structure):
    _fields_ = [('poles', ct.c_uint16)] + [(v, ct.c_float) for v in
        ('rs', 'ld', 'lq', 'flux', 'inertia')]


class Ekf(ct.Structure):
    _fields_ = [('c', Coeff), ('i', AB), ('speed', ct.c_float),
                ('angle', ct.c_uint32), ('p', ct.c_float*10),
                ('has_sample', ct.c_bool)]


class Output(ct.Structure):
    _fields_ = [('angle', ct.c_uint32), ('speed', ct.c_float),
                ('valid', ct.c_bool)]


CFG = Cfg(20000, BASE, (1e-6, 1e-6, 1e-7, 1e-8),
          (1e-6, 0.0, 1e-6), (0.01, 0.01, 0.25, 1/12),
          0.05, 0.000625, 0.0025)
MODEL = Model(7, RS, LS, LS, FLUX, 0)


def covariance(e):
    p = np.zeros((4, 4))
    p[UPPER] = e.p
    return p + np.triu(p, 1).T


def state(e):
    return np.array([e.i.a, e.i.b, e.speed, e.angle / 2**32])


def wrap(th):
    return (th + 0.5) % 1 - 0.5


def init(lib, cfg=CFG):
    e = Ekf()
    assert lib.mfoc_ekf_Init(ct.byref(e), ct.byref(cfg),
                             ct.byref(MODEL)) == 0
    return e


def load_library(directory):
    dll = Path(directory) / 'ekf.dll'
    sources = list((ROOT/'mfoc/math/src').glob('*.c')) + \
        list((ROOT/'mfoc/control/src').glob('*.c')) + [
        ROOT/'mfoc/position/src/mfoc_ekf.c']
    includes = ['mfoc', 'mfoc/math/include', 'mfoc/control/include',
                'mfoc/motor/include',
                'mfoc/position/include']
    subprocess.run(['gcc', '-shared', '-std=gnu11', '-O2', '-Wall',
                    '-Wextra', '-Werror', '-ffp-contract=off',
                    '-DMFOC_NUMERIC_FLOAT=1', '-DMFOC_FEEDBACK_ENABLE_EKF=1',
                    '-DMFOC_TRIG_BACKEND=0'] +
                   ['-I'+str(ROOT/p) for p in includes] +
                   [str(p) for p in sources] + ['-lm', '-o', str(dll)],
                   check=True)
    lib = ct.CDLL(str(dll))
    lib.mfoc_ekf_Init.argtypes = [ct.POINTER(Ekf), ct.POINTER(Cfg),
                                  ct.POINTER(Model)]
    lib.mfoc_ekf_Step.argtypes = [ct.POINTER(Ekf), ct.POINTER(AB),
                                  ct.POINTER(AB), ct.POINTER(Output)]
    lib.mfoc_ekf_Step.restype = None
    return lib


def c_oracle_checks(lib):
    rng = np.random.default_rng(42)
    mean_max, cov_max = 0.0, 0.0
    for _ in range(1000):
        cfg = Cfg.from_buffer_copy(CFG)
        cfg.r[:] = [1e-4, 4e-5, 2e-4]
        e = init(lib, cfg)
        a = rng.normal(size=(4, 4))*0.03
        p = a @ a.T + np.eye(4)*1e-4
        e.p[:] = p[UPPER]
        e.i = AB(*rng.normal(size=2)*0.05)
        e.speed = rng.uniform(-0.8, 0.8)
        e.angle = int(rng.uniform(0, 1)*2**32)
        e.has_sample = True
        u, z = AB(*rng.normal(size=2)*0.2), AB(*rng.normal(size=2)*0.05)
        x, p = state(e), covariance(e)
        f = jacobian(x)
        xp, pp = batch(predict(x, [u.a, u.b]),
                       f @ p @ f.T + np.diag(cfg.q), [z.a, z.b],
                       np.array([[cfg.r[0], cfg.r[1]],
                                 [cfg.r[1], cfg.r[2]]]))
        out = Output()
        lib.mfoc_ekf_Step(ct.byref(e), ct.byref(z), ct.byref(u), ct.byref(out))
        delta = state(e)-xp
        delta[3] = wrap(delta[3])
        mean_max = max(mean_max, np.abs(delta).max())
        cov_max = max(cov_max, np.abs(covariance(e)-pp).max())
        assert np.linalg.eigvalsh(covariance(e)).min() >= -1e-9
    assert mean_max < 5e-6 and cov_max < 1e-7, (mean_max, cov_max)
    return dict(c_mean_max=mean_max, c_cov_max=cov_max, sizeof_ekf=ct.sizeof(Ekf))


def plant_checks(lib):
    # Concurrent independent plants; voltage held each control interval.
    cases = [(f'w{w}_th{th}', w, th, 1, 1, 1, 0, 1, 0)
             for w in (0.4, -0.4) for th in (0, 0.125, 0.5, 0.875)]
    cases += [(name, 0.4, 0.125, r, l, flux, bias, voltage, delay)
              for name, r, l, flux, bias, voltage, delay in [
                  ('Rs-20%', .8, 1, 1, 0, 1, 0),
                  ('Rs+20%', 1.2, 1, 1, 0, 1, 0),
                  ('L-20%', 1, .8, 1, 0, 1, 0),
                  ('L+20%', 1, 1.2, 1, 0, 1, 0),
                  ('flux-10%', 1, 1, .9, 0, 1, 0),
                  ('flux+10%', 1, 1, 1.1, 0, 1, 0),
                  ('current-bias', 1, 1, 1, .005, 1, 0),
                  ('voltage+5%', 1, 1, 1, 0, 1.05, 0),
                  ('voltage-delay', 1, 1, 1, 0, 1, 1),
                  ('ramp-load', 1, 1, 1, 0, 1, 0)]]
    n = len(cases)
    data = np.array([c[1:] for c in cases])
    speed, angle = data[:, 0].copy(), data[:, 1].copy()
    r, l, flux = RS*data[:, 2], LS*data[:, 3], FLUX*data[:, 4]
    currents = np.zeros((n, 2))
    states, out = [init(lib) for _ in cases], Output()
    noise = np.random.default_rng(20261031).normal(0, .001, (6000, n, 2))
    last_u = np.zeros((n, 2))
    errors, speed_errors, valid = [], [], []
    min_eigen = math.inf
    for k in range(6000):
        speed[-1] = 0.05 + 0.35*min(k/2500, 1)
        if k > 4000:
            speed[-1] = 0.4 - 0.2*min((k-4000)/1500, 1)
        iq = 0.04 if k < 3500 else 0.08
        s, c = np.sin(TAU*angle), np.cos(TAU*angle)
        desired = iq*np.stack([-s, c], axis=1)
        rotation = iq*np.stack([-c, -s], axis=1)
        # Feedforward uses true parameters only in the independent plant.
        u = r[:, None]*desired + (l*speed)[:, None]*rotation + (
            flux*speed)[:, None]*np.stack([-s, c], axis=1)

        def derivative(i, th):
            emf = (flux*speed)[:, None]*np.stack(
                [np.sin(TAU*th), -np.cos(TAU*th)], axis=1)
            return TAU*BASE*(u-r[:, None]*i+emf)/l[:, None]

        h = DT/20
        for _ in range(20):
            half = angle + BASE*speed*h/2
            end = angle + BASE*speed*h
            a = derivative(currents, angle)
            b = derivative(currents+h*a/2, half)
            c1 = derivative(currents+h*b/2, half)
            d = derivative(currents+h*c1, end)
            currents += h*(a+2*b+2*c1+d)/6
            angle = end % 1
        sample = currents + noise[k] + data[:, 5, None]
        errors_now, speeds_now, valid_now = [], [], []
        for j, e in enumerate(states):
            supplied = (last_u[j] if data[j, 7] else u[j])*data[j, 6]
            measurement, voltage = AB(*sample[j]), AB(*supplied)
            lib.mfoc_ekf_Step(ct.byref(e), ct.byref(measurement),
                              ct.byref(voltage), ct.byref(out))
            assert e.has_sample
            assert np.isfinite(state(e)).all()
            eig = float(np.linalg.eigvalsh(covariance(e)).min())
            min_eigen = min(min_eigen, eig)
            assert eig >= -1e-9
            errors_now.append(wrap(e.angle/2**32-angle[j])*360)
            speeds_now.append(e.speed-speed[j])
            valid_now.append(out.valid)
        errors.append(errors_now)
        speed_errors.append(speeds_now)
        valid.append(valid_now)
        last_u = u.copy()
    tail = np.array(errors)[-1000:]
    speed_tail = np.array(speed_errors)[-1000:]
    validity = np.array(valid)[-1000:].mean(axis=0)
    result = []
    for j, case in enumerate(cases):
        item = dict(case=case[0], bias_deg=float(tail[:, j].mean()),
                    std_deg=float(tail[:, j].std()),
                    p95_deg=float(np.quantile(abs(tail[:, j]), .95)),
                    speed_bias_pu=float(speed_tail[:, j].mean()),
                    valid_fraction=float(validity[j]))
        result.append(item)
        # Nominal plants: must converge without supplying true angle/speed.
        if j < 8:
            assert item['p95_deg'] < 10 and abs(item['speed_bias_pu']) < .01
            assert item['valid_fraction'] > .99
    return dict(rk4_substeps=20, plant_min_eigen=min_eigen, cases=result)


def long_check(lib, steps):
    e, out = init(lib), Output()
    i = AB(0, 0)
    angle, w = .17, .4
    min_eigen = math.inf
    valid_count = 0
    for k in range(steps):
        # Exact zero-current rotating solution of the discrete test model.
        u = AB(-FLUX*w*math.sin(TAU*angle), FLUX*w*math.cos(TAU*angle))
        angle = (angle+KT*w) % 1
        lib.mfoc_ekf_Step(ct.byref(e), ct.byref(i), ct.byref(u), ct.byref(out))
        p = covariance(e)
        eig = float(np.linalg.eigvalsh(p).min())
        min_eigen = min(min_eigen, eig)
        assert np.isfinite(p).all() and eig >= -1e-9
        assert e.has_sample
        if k > 1000:
            assert abs(wrap(e.angle/2**32-angle)) < .01
            assert abs(e.speed-w) < .005
            valid_count += int(out.valid)
    # Standstill has no angle information; never claim validity at zero w.
    zero = init(lib)
    for _ in range(2000):
        lib.mfoc_ekf_Step(ct.byref(zero), ct.byref(i), ct.byref(i), ct.byref(out))
        assert not out.valid
    return dict(long_steps=steps, min_eigen=min_eigen,
                long_valid=valid_count, standstill_invalid=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--long-steps', type=int, default=1000000)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = math_checks()
    print('Float64 Jacobian / correlated sequential Joseph passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='mfoc-ekf-reference-') as directory:
        lib = load_library(directory)
        result.update(c_oracle_checks(lib))
        print('C posterior vs float64 batch Joseph passed', flush=True)
        result.update(plant_checks(lib))
        print('RK4 plants / parameter scans completed', flush=True)
        result.update(long_check(lib, args.long_steps))
        # Windows keeps a loaded DLL locked; release before removing temp dir.
        import _ctypes
        _ctypes.FreeLibrary(lib._handle)
    encoded = json.dumps(result, indent=2)
    print(encoded)
    if args.output:
        args.output.write_text(encoded+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
