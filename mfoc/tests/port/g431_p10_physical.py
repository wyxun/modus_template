"""Decode a stopped G431 B physical experiment RAM capture.

Input is the little-endian g_p10_physical symbol from the matching ELF.
The duty triplets describe commands relative to the captured update event.
They are not PWM pin measurements.
"""

import argparse
import json
import math
import statistics
import struct
from pathlib import Path


MAGIC = 0x50313042
HEADER = struct.Struct("<3Ii4Iff2I")
RECORD = struct.Struct("<10I4f10If")
STATUS = {
    0: "idle", 1: "active", 2: "done",
    10: "overcurrent", 11: "frame_loss", 12: "timer_sequence", 13: "pwm",
    14: "host_abort", 15: "break", 16: "watchdog",
}


def decode(data):
    (magic, status, mode, mpu, electrical_hz, period_frames, frames,
     last_trigger, max_current, bus, arr, rows) = HEADER.unpack_from(data)
    if magic != MAGIC or (mode, rows) not in ((1, 8), (2, 32)):
        raise ValueError("unrecognized G431 P10 experiment capture")
    if status not in STATUS or status < 2 or arr == 0:
        raise ValueError("capture has no terminal result")
    if len(data) < HEADER.size + rows * RECORD.size:
        raise ValueError("truncated G431 P10 experiment capture")

    result = {
        "kind": "pulse" if mode == 1 else "sweep",
        "status": STATUS[status],
        "command_mpu": mpu,
        "electrical_hz": electrical_hz,
        "period_frames": period_frames,
        "frames": frames,
        "last_trigger": last_trigger,
        "max_abs_current_pu": max_current,
        "bus_pu": bus,
        "arr": arr,
        "records": [],
    }
    for index in range(rows):
        row = RECORD.unpack_from(data, HEADER.size + index * RECORD.size)
        n, seq, trigger, capture, entry, write, angle = row[:7]
        if n == 0:
            continue
        result["records"].append({
            "index": index,
            "n": n,
            "seq": seq,
            "trigger": trigger,
            "capture": capture,
            "irq_entry": entry,
            "write": write,
            "angle_after_bam32": angle,
            "raw_uvw": row[7:10],
            "i_uvw_pu": row[10:13],
            "kcl_pu": row[13],
            "before_ccr": row[14:17],
            "after_ccr": row[17:20],
            "command_ccr": row[20:23],
            "max_v_ccr": row[23],
            "sum_kcl_squared": row[24],
        })
    return result


def summarize(result):
    records = result["records"]
    if result["kind"] == "pulse":
        if [r["index"] for r in records] != list(range(8)):
            return {"complete": False}
        delta = {
            f"k{offset:+d}": tuple(y - x for x, y in zip(
                records[i - 1]["i_uvw_pu"], records[i]["i_uvw_pu"]))
            for i, offset in enumerate(range(-2, 6)) if i > 0
        }
        return {"complete": result["status"] == "done",
                "current_increments_uvw_pu": delta}

    n = sum(row["n"] for row in records)
    if not n:
        return {"complete": False, "bins": 0, "samples": 0}
    peak = max(records, key=lambda row: abs(row["kcl_pu"]))
    arr = result["arr"]
    return {
        "complete": (
            result["status"] == "done" and len(records) == 32
            and n >= result["period_frames"]
        ),
        "bins": len(records),
        "samples": n,
        "max_abs_kcl_pu": abs(peak["kcl_pu"]),
        "peak_bin": peak["index"],
        "median_bin_peak_pu": statistics.median(
            abs(row["kcl_pu"]) for row in records),
        "rms_kcl_pu": math.sqrt(sum(row["sum_kcl_squared"]
                                    for row in records) / n),
        "max_v_duty": max(row["max_v_ccr"] for row in records) / arr,
        "peak_bin_after_v_duty": peak["after_ccr"][1] / arr,
    }


def pulse_stats(captures):
    """Estimate the part of each current increment that changes with polarity."""
    groups = {}
    for result in captures:
        if result["kind"] != "pulse" or result["status"] != "done":
            raise ValueError("pulse statistics require completed pulse captures")
        increments = summarize(result)["current_increments_uvw_pu"]
        group = groups.setdefault(result["command_mpu"], {})
        for offset, (u, v, w) in increments.items():
            group.setdefault(offset, []).append((2 * u - v - w) / 3)

    output = {}
    for mpu in sorted(m for m in groups if m > 0 and -m in groups):
        positive, negative = groups[mpu], groups[-mpu]
        rows = {}
        for offset in positive:
            pos, neg = positive[offset], negative[offset]
            if min(len(pos), len(neg)) < 2:
                raise ValueError("need at least two captures per polarity")
            variance = statistics.variance(pos) / len(pos) + (
                statistics.variance(neg) / len(neg))
            zero = groups.get(0, {}).get(offset, [])
            rows[offset] = {
                "positive_mpu": 1000 * statistics.mean(pos),
                "negative_mpu": 1000 * statistics.mean(neg),
                "zero_mpu": 1000 * statistics.mean(zero) if zero else None,
                "signed_mpu": 500 * (statistics.mean(pos) - statistics.mean(neg)),
                "standard_error_mpu": 500 * math.sqrt(variance),
            }
        output[mpu] = {
            "counts": [len(next(iter(positive.values()))),
                       len(next(iter(negative.values()))),
                       len(next(iter(groups.get(0, {}).values()), []))],
            "delta_i_alpha": rows,
        }
    if not output:
        raise ValueError("no matched pulse amplitudes of both polarities")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path, nargs="+")
    parser.add_argument("--pulse-stats", action="store_true")
    args = parser.parse_args()
    captures = [decode(path.read_bytes()) for path in args.capture]
    if args.pulse_stats:
        result = pulse_stats(captures)
    else:
        if len(captures) != 1:
            parser.error("decode one capture or use --pulse-stats")
        result = captures[0]
        result["summary"] = summarize(result)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
