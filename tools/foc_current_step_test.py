#!/usr/bin/env python3
"""Capture and analyze a firmware-timed FOC q-current step over AITrace RTT."""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _resolve_aitrace(repo_root: Path) -> Path:
    """Prefer the sibling build that writes sample-index metadata."""
    sibling_build = (
        repo_root.parent / "mstudio" / "aitrace" / "aitrace.exe"
    )
    if sibling_build.is_file():
        return sibling_build
    return repo_root / "tools" / "aitrace.exe"


AITRACE = _resolve_aitrace(ROOT)
MOTOR_IDLE = 2
MOTOR_RUNNING = 4


@dataclass(frozen=True)
class StepAnalysis:
    sample_count: int
    step_index: int
    end_index: int | None
    actual_pulse_ms: float | None
    baseline_iq: float
    steady_iq: float
    target_iq: float
    target_error_iq: float
    measured_sample_rate_hz: float
    rise_10_ms: float | None
    rise_90_ms: float | None
    rise_time_ms: float | None
    peak_iq: float
    overshoot_percent: float
    peak_above_steady_percent: float
    speed_start_pu: float | None
    speed_end_pu: float | None


def validate_test_parameters(iq: float, pulse_ms: int) -> None:
    """Keep this first test within the requested low-current envelope."""
    if not math.isfinite(iq) or not 0.0 < iq <= 0.05:
        raise ValueError("Iq must be greater than 0 and at most 0.05 pu")
    if not 25 <= pulse_ms <= 150:
        raise ValueError("pulse duration must be between 25 and 150 ms")


def analyze_csv(
    path: Path,
    sample_rate_hz: int,
    target_iq: float,
    smoothing_samples: int = 64,
) -> StepAnalysis:
    """Measure a current pulse from CSV timestamps and sample indices."""
    rows: list[tuple[float, int, float, float, float | None]] = []
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        speed_field = next(
            (name for name in ("ElecSpeed", "ElecSpe") if name in fields),
            None,
        )
        required = {"time", "sample_index", "IqRef", "Iq"}
        if not required.issubset(fields):
            raise ValueError(
                "CSV must contain time, sample_index, IqRef, and Iq"
            )
        signal_fields = ["IqRef", "Iq"]
        if speed_field is not None:
            signal_fields.append(speed_field)
        for row in reader:
            values = [row.get(name) for name in signal_fields]
            if all(value is None or not value.strip() for value in values):
                continue
            if any(value is None or not value.strip() for value in values):
                raise ValueError("CSV contains a partially empty sample")
            try:
                timestamp = float(row["time"])
                sample_index = int(row["sample_index"])
                reference = float(row["IqRef"])
                current = float(row["Iq"])
                speed = float(row[speed_field]) if speed_field else None
            except (TypeError, ValueError) as exc:
                raise ValueError("CSV contains a non-numeric sample") from exc
            sample_values = (timestamp, reference, current)
            if (not all(math.isfinite(value) for value in sample_values) or
                    (speed is not None and not math.isfinite(speed))):
                raise ValueError("CSV contains a non-finite sample")
            rows.append((timestamp, sample_index, reference, current, speed))

    if sample_rate_hz <= 0 or smoothing_samples <= 0:
        raise ValueError("sample rate and smoothing window must be positive")
    if len(rows) < 2:
        raise ValueError("CSV does not contain enough waveform samples")
    for previous, current in zip(rows, rows[1:]):
        if current[0] <= previous[0] or current[1] <= previous[1]:
            raise ValueError("CSV time and sample_index must increase")

    threshold = target_iq * 0.5
    step_index = next(
        (i for i, row in enumerate(rows) if row[2] >= threshold), None
    )
    if step_index is None:
        raise ValueError("CSV does not contain the requested IqRef step")
    end_index = next(
        (i for i in range(step_index + 1, len(rows))
         if rows[i][2] < threshold),
        None,
    )
    pulse_end = len(rows) if end_index is None else end_index
    baseline_start = max(0, step_index - min(500, step_index))
    for index in range(baseline_start + 1, pulse_end):
        if rows[index][1] != rows[index - 1][1] + 1:
            raise ValueError(
                f"sample gap inside baseline or step window at row {index}"
            )
    baseline_rows = rows[baseline_start:step_index]
    baseline_iq = (
        statistics.median(row[3] for row in baseline_rows)
        if baseline_rows else 0.0
    )
    pulse_rows = rows[step_index:pulse_end]
    pulse_iq = [row[3] for row in pulse_rows]
    if not pulse_iq:
        raise ValueError("IqRef pulse contains no current samples")

    window = min(smoothing_samples, len(pulse_iq))
    smooth: list[tuple[float, float]] = []
    running_sum = sum(pulse_iq[:window])
    smooth.append((
        running_sum / window,
        sum(row[0] for row in pulse_rows[:window]) / window,
    ))
    for index in range(window, len(pulse_iq)):
        running_sum += pulse_iq[index] - pulse_iq[index - window]
        smooth.append((
            running_sum / window,
            sum(row[0] for row in pulse_rows[index - window + 1:index + 1]) /
            window,
        ))

    pulse_tail_count = max(1, math.ceil(len(pulse_iq) * 0.1))
    steady_iq = statistics.median(pulse_iq[-pulse_tail_count:])
    delta = steady_iq - baseline_iq
    if delta <= 0.0:
        raise ValueError("measured current response did not rise above baseline")
    index_10 = next(
        (i for i, item in enumerate(smooth)
         if item[0] >= baseline_iq + delta * 0.1), None
    )
    index_90 = next(
        (i for i, item in enumerate(smooth)
         if item[0] >= baseline_iq + delta * 0.9), None
    )
    rise_10_ms = (
        (smooth[index_10][1] - rows[step_index][0]) * 1000.0
        if index_10 is not None else None
    )
    rise_90_ms = (
        (smooth[index_90][1] - rows[step_index][0]) * 1000.0
        if index_90 is not None else None
    )
    rise_time_ms = (
        rise_90_ms - rise_10_ms
        if rise_10_ms is not None and rise_90_ms is not None else None
    )
    peak_iq = max(item[0] for item in smooth)
    target_delta = target_iq - baseline_iq
    if target_delta <= 0.0:
        raise ValueError("target current must exceed baseline current")
    overshoot = max(0.0, (peak_iq - target_iq) / target_delta * 100.0)
    peak_above_steady = max(
        0.0, (peak_iq - steady_iq) / delta * 100.0,
    )
    local_times = [row[0] for row in rows[baseline_start:pulse_end]]
    local_periods = [
        current - previous
        for previous, current in zip(local_times, local_times[1:])
    ]
    measured_sample_rate_hz = 1.0 / statistics.median(local_periods)
    return StepAnalysis(
        sample_count=len(rows), step_index=step_index, end_index=end_index,
        actual_pulse_ms=(
            (rows[end_index][0] - rows[step_index][0]) * 1000.0
            if end_index is not None else None
        ),
        baseline_iq=baseline_iq, steady_iq=steady_iq,
        target_iq=target_iq, target_error_iq=target_iq - steady_iq,
        measured_sample_rate_hz=measured_sample_rate_hz,
        rise_10_ms=rise_10_ms, rise_90_ms=rise_90_ms,
        rise_time_ms=rise_time_ms, peak_iq=peak_iq,
        overshoot_percent=overshoot,
        peak_above_steady_percent=peak_above_steady,
        speed_start_pu=rows[step_index][4],
        speed_end_pu=rows[pulse_end - 1][4],
    )


def _aitrace(args: list[str], timeout: float = 8.0) -> str:
    result = subprocess.run(
        [str(AITRACE), *args], cwd=ROOT, capture_output=True,
        text=True, timeout=timeout, check=False,
    )
    output = result.stdout + result.stderr
    if result.returncode != 0:
        raise RuntimeError(f"aitrace {' '.join(args)} failed: {output.strip()}")
    if "motor command rejected" in output or "Unknown command" in output:
        raise RuntimeError(output.strip())
    return output


def _motor_status() -> dict[str, int]:
    output = _aitrace(["shell", "--raw", "motor status"])
    match = re.search(
        r"motor state=(\d+) fault=0x([0-9A-Fa-f]+) mode=(\d+) "
        r"pwm=(\d+).*?zero=(\d+) angle=(\d+)", output,
    )
    if match is None:
        raise RuntimeError(f"cannot parse motor status: {output.strip()}")
    state, faults, mode, pwm, zero, angle = match.groups()
    return {
        "state": int(state), "fault": int(faults, 16),
        "mode": int(mode), "pwm": int(pwm),
        "zero": int(zero), "angle": int(angle),
    }


def _wait_for_status(predicate, timeout_s: float, description: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = _motor_status()
        if status["fault"] != 0:
            raise RuntimeError(f"motor fault during {description}: {status}")
        if predicate(status):
            return
        time.sleep(0.05)
    raise TimeoutError(f"timed out waiting for {description}")


def _csv_data_rows(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = ("time", "sample_index", "IqRef", "Iq")
        if reader.fieldnames is None or any(
            name not in reader.fieldnames for name in required
        ):
            return 0
        return sum(
            1 for row in reader
            if all((row.get(name) or "").strip() for name in required)
        )


def _wait_for_baseline(path: Path, rows_needed: int,
                       capture: subprocess.Popen, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if capture.poll() is not None:
            raise RuntimeError("wave capture ended before baseline was ready")
        if _csv_data_rows(path) >= rows_needed:
            return
        time.sleep(0.05)
    raise TimeoutError("wave CSV did not accumulate enough baseline samples")


def _confirm_free_rotation(skip_prompt: bool) -> None:
    message = (
        "This test energizes the motor and permits free rotation. "
        "Mount the motor securely, remove the load, and keep the shaft clear; "
        "do not hold or clamp the rotor."
    )
    print(message)
    if skip_prompt:
        return
    answer = input("Type YES to align and run the current step: ").strip()
    if answer != "YES":
        raise RuntimeError("test cancelled")


def _choose_output(path: str | None) -> Path:
    if path:
        output = Path(path)
        if not output.is_absolute():
            output = ROOT / output
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        output = ROOT / "build" / f"current_step_{stamp}.csv"
    if output.exists():
        raise FileExistsError(f"refusing to overwrite {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    return output


def run_test(args: argparse.Namespace) -> Path:
    validate_test_parameters(args.iq, args.pulse_ms)
    if args.sample_rate_hz <= 0 or args.smoothing_samples <= 0:
        raise ValueError("sample rate and smoothing window must be positive")
    if not 25 <= args.baseline_ms <= 500:
        raise ValueError("baseline duration must be between 25 and 500 ms")
    if not AITRACE.is_file():
        raise FileNotFoundError(f"aitrace executable not found: {AITRACE}")
    _confirm_free_rotation(args.yes)
    output = _choose_output(args.output)
    status = _motor_status()
    if status["state"] != MOTOR_IDLE or status["pwm"] != 0:
        raise RuntimeError(f"motor must be idle before alignment: {status}")
    if status["fault"] != 0:
        raise RuntimeError(f"clear motor fault before testing: {status}")

    capture: subprocess.Popen | None = None
    motor_started = False
    try:
        motor_started = True
        _aitrace(["shell", "--raw", "motor align"])
        _wait_for_status(
            lambda state: state["state"] == MOTOR_IDLE and
            state["pwm"] == 0 and state["zero"] == 1 and
            state["angle"] == 1,
            args.align_timeout_s, "electrical alignment",
        )
        motor_started = False
        motor_started = True
        _aitrace(["shell", "--raw", "motor current 0 0"])
        _wait_for_status(
            lambda state: state["state"] == MOTOR_RUNNING and
            state["pwm"] == 1,
            3.0, "zero-reference current mode",
        )

        capture_seconds = max(
            3, math.ceil((args.baseline_ms + 2 * args.pulse_ms) / 1000) + 2
        )
        capture = subprocess.Popen(
            [str(AITRACE), "wave", "capture", str(capture_seconds),
             "--output", str(output)],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        rows_needed = max(100, int(
            args.sample_rate_hz * args.baseline_ms / 1000
        ))
        _wait_for_baseline(output, rows_needed, capture, 5.0)
        _aitrace([
            "shell", "--raw",
            f"motor step {args.iq:.6f} {args.pulse_ms}",
        ])
        _wait_for_status(
            lambda state: state["state"] == MOTOR_IDLE and
            state["pwm"] == 0,
            args.pulse_ms / 1000.0 + 3.0,
            "firmware-timed current pulse and automatic stop",
        )
        motor_started = False
        try:
            capture.wait(timeout=capture_seconds + 5)
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError("wave capture did not finish") from exc
        if capture.returncode != 0:
            raise RuntimeError(f"wave capture exited with {capture.returncode}")
    finally:
        if motor_started:
            try:
                _aitrace(["shell", "--raw", "motor stop"])
            except (RuntimeError, subprocess.SubprocessError):
                pass
        if capture is not None and capture.poll() is None:
            capture.terminate()
            try:
                capture.wait(timeout=2)
            except subprocess.TimeoutExpired:
                capture.kill()
                capture.wait(timeout=2)

    status = _motor_status()
    if (status["state"] != MOTOR_IDLE or status["pwm"] != 0 or
            status["fault"] != 0):
        raise RuntimeError(f"motor did not return to idle: {status}")
    result = analyze_csv(
        output, args.sample_rate_hz, args.iq, args.smoothing_samples,
    )
    if result.end_index is None:
        raise RuntimeError(
            "CSV ended before IqRef returned to zero; pulse data is incomplete"
        )
    print(f"CSV: {output}")
    print(f"samples={result.sample_count} step_index={result.step_index} "
          f"end_index={result.end_index}")
    print(f"baseline Iq={result.baseline_iq:.4f} pu, "
          f"steady={result.steady_iq:.4f} pu, "
          f"target={result.target_iq:.4f} pu, "
          f"steady-state error={result.target_error_iq:+.4f} pu")
    print(f"10-90 rise={result.rise_time_ms!s} ms, "
          f"smoothed peak={result.peak_iq:.4f} pu, "
          f"overshoot vs target={result.overshoot_percent:.1f}%, "
          f"peak above measured steady="
          f"{result.peak_above_steady_percent:.1f}%")
    if result.actual_pulse_ms is None:
        print("IqRef falling edge was not captured; pulse width unavailable")
    else:
        print(f"IqRef high time={result.actual_pulse_ms:.1f} ms "
              f"(requested {args.pulse_ms} ms)")
    print(f"measured local sample rate="
          f"{result.measured_sample_rate_hz:.1f} Hz "
          f"(configured nominal {args.sample_rate_hz} Hz)")
    if result.speed_start_pu is None or result.speed_end_pu is None:
        print("speed during pulse unavailable in this waveform capture")
    else:
        print(f"speed during pulse={result.speed_start_pu:.3f} -> "
              f"{result.speed_end_pu:.3f} pu")
    print(f"final motor state=idle pwm=0 fault=0x{status['fault']:08X}")
    return output


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iq", type=float, default=0.05,
                        help="q-axis reference in pu (0 < Iq <= 0.05)")
    parser.add_argument("--pulse-ms", type=int, default=100,
                        help="reference pulse duration, 25..150 ms")
    parser.add_argument("--baseline-ms", type=int, default=100,
                        help="zero-reference baseline before the step")
    parser.add_argument("--sample-rate-hz", type=int, default=10000,
                        help="measured waveform rate used for timing estimates")
    parser.add_argument("--smoothing-samples", type=int, default=64,
                        help="moving-average window for rise/overshoot metrics")
    parser.add_argument("--align-timeout-s", type=float, default=20.0)
    parser.add_argument("--output", help="CSV output path; must not exist")
    parser.add_argument("--yes", action="store_true",
                        help="skip the free-rotation confirmation prompt")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    try:
        run_test(args)
    except (FileExistsError, FileNotFoundError, RuntimeError,
            TimeoutError, ValueError, subprocess.SubprocessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
