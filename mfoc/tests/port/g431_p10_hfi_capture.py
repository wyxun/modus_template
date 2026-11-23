"""Decode the stopped B HFI capture symbol from the matching G431 ELF."""

import argparse
import json
import math
from pathlib import Path
import statistics
import struct


MAGIC = 0x31494648
MAGIC_2 = 0x32494648
PREFIX, TAIL, STRIDE = 32, 224, 8
PREFIX_2, TAIL_2 = 128, 128
AXIS_OK, AXIS_WEAK = 1 << 2, 1 << 3
LOCK = 1 << 14
ACQUIRE, TRACK = 1 << 28, 1 << 29
HEADER = struct.Struct("<4I")
ENCODING = {0: struct.Struct("<2I8f"), 15: struct.Struct("<2I8i")}
ENCODING_2 = {0: struct.Struct("<3I8f"), 15: struct.Struct("<3I8i")}


def slots(groups, prefix=PREFIX, tail=TAIL):
    for index in range(min(groups, prefix)):
        yield index, index
    tail_groups = max(0, (groups - prefix + STRIDE - 1) // STRIDE)
    for index in range(max(0, tail_groups - tail), tail_groups):
        yield prefix + index % tail, prefix + index * STRIDE


def row_metrics(values, voltage, min_response, threshold):
    iaa, iba, iab, ibb, uaa, uba, uab, ubb = values
    det = uaa * ubb - uab * uba
    response = sum(map(abs, values[:4]))
    aa, bb = iaa * ubb - iab * uba, ibb * uaa - iba * uab
    x = aa - bb
    y = iab * uaa - iaa * uab + iba * ubb - ibb * uba
    trace, magnitude = aa + bb, abs(x) + abs(y)
    ratio = None
    if det < voltage * voltage:
        reason = "excitation"
    elif response < min_response:
        reason = "response"
    elif aa <= 0 or bb <= 0 or magnitude > 2 * trace:
        reason = "conditioning"
    else:
        ratio = magnitude / trace
        reason = "below_saliency" if ratio < threshold else "accepted"
    return ratio, reason, det, response, x, y


def decode(data):
    magic, bits, groups, tail_next = HEADER.unpack_from(data)
    if magic not in (MAGIC, MAGIC_2) or bits not in ENCODING:
        raise ValueError("unrecognized G431 B HFI capture")
    version_2 = magic == MAGIC_2
    prefix, tail = ((PREFIX_2, TAIL_2) if version_2 else (PREFIX, TAIL))
    record = (ENCODING_2 if version_2 else ENCODING)[bits]
    header_size = HEADER.size + (4 if version_2 else 0)
    if len(data) != header_size + (prefix + tail) * record.size:
        raise ValueError("capture does not match ELF symbol size")
    injection_mpu = struct.unpack_from("<I", data, HEADER.size)[0] if version_2 else 40
    if injection_mpu not in (40, 80, 120):
        raise ValueError("unrecognized HFI injection setting")
    tail_groups = max(0, (groups - prefix + STRIDE - 1) // STRIDE)
    if tail_next != tail_groups % tail:
        raise ValueError("inconsistent HFI capture; read with bridge stopped")
    scale = 1 << bits
    amplitude = injection_mpu / 1000
    voltage, response, threshold = (
        (round(amplitude * scale), round(0.002 * scale), round(0.04 * scale) / scale)
        if bits else (amplitude, 0.002, 0.04)
    )
    rows = []
    last_seq = 0
    for slot, group in slots(groups, prefix, tail):
        seq, events, *values = record.unpack_from(data,
            header_size + slot * record.size)
        tracker_turns = values.pop(0) / 2**32 if version_2 else None
        axis_event = events & (AXIS_OK | AXIS_WEAK)
        if (seq <= last_seq or axis_event not in
                (AXIS_OK, AXIS_WEAK) or not all(map(math.isfinite, values))):
            raise ValueError("incoherent HFI record; read with bridge stopped")
        last_seq = seq
        ratio, reason, det, strength, x, y = row_metrics(
            values, voltage, response, threshold)
        # An accepted axis is ambiguous by pi. The observer's PLL can report a
        # different angle after correction and prediction; this is raw axis data.
        axis = ((math.atan2(y, x) / (4 * math.pi)) % 0.5
                if axis_event == AXIS_OK else None)
        pu = [value / scale for value in values]
        rows.append({"group": group + 1, "frame": seq, "event_mask": events,
                     "axis_ok": bool(events & AXIS_OK), "lock": bool(events & LOCK),
                     "track": bool(events & TRACK), "acquire": bool(events & ACQUIRE),
                     "delta_i_pu": [pu[:2], pu[2:4]],
                     "delta_u_pu": [pu[4:6], pu[6:]],
                     "det_u_pu2": det / (scale * scale), "response_pu": strength / scale,
                     "saliency_l1": ratio, "offline_axis_gate": reason,
                     "axis_turns_mod_half": axis, "tracker_turns": tracker_turns})
    return {"numeric": "FIXED" if bits else "FLOAT", "groups_total": groups,
            "injection_mpu": injection_mpu, "voltage_threshold_pu": voltage / scale,
            "response_threshold_pu": response / scale, "saliency_gate": threshold,
            "consecutive_prefix_size": prefix, "tail_stride": STRIDE,
            "tail_overwritten": max(0, tail_groups - tail), "records": rows}


def percentile_95(values):
    ordered = sorted(values)
    return ordered[math.ceil(0.95 * len(ordered)) - 1] if ordered else None


def axis_spread(rows, angle_key="axis_turns_mod_half"):
    angles = [row[angle_key] for row in rows if row[angle_key] is not None]
    if not angles:
        return None
    s = statistics.mean(math.sin(4 * math.pi * a) for a in angles)
    c = statistics.mean(math.cos(4 * math.pi * a) for a in angles)
    mean = (math.atan2(s, c) / (4 * math.pi)) % 0.5
    distance_deg = [360 * abs((a - mean + 0.25) % 0.5 - 0.25) for a in angles]
    return {"valid_groups": len(angles), "center_turns_mod_half": mean,
            "concentration": math.hypot(s, c),
            "rms_deg": math.sqrt(statistics.mean(d * d for d in distance_deg)),
            "p95_deg": percentile_95(distance_deg), "max_deg": max(distance_deg)}


def summarize(rows):
    ratios = sorted(row["saliency_l1"] for row in rows
                    if row["saliency_l1"] is not None)
    gate_counts = {name: sum(row["offline_axis_gate"] == name for row in rows)
                   for name in ("excitation", "response", "conditioning",
                                "below_saliency", "accepted")}
    return {"records": len(rows), "axis_ok": sum(row["axis_ok"] for row in rows),
            "lock_frames_in_sample": [row["frame"] for row in rows if row["lock"]],
            "track_groups_in_sample": sum(row["track"] for row in rows),
            "offline_gates": gate_counts,
            "axis_spread_modulo_pi": axis_spread(rows),
            "saliency_l1": {"p50": statistics.median(ratios),
                            "p95": percentile_95(ratios),
                            "max": ratios[-1]} if ratios else None}


def coherent_averages(result):
    """Average matched signed Di/Du before calculating the two-axis solve."""
    blocks, chunk = [], []
    for row in result["records"]:
        if chunk and (row["group"] != chunk[-1]["group"] + 1 or
                      row["frame"] != chunk[-1]["frame"] + 8):
            chunk.clear()
        chunk.append(row)
        if len(chunk) != 32:
            continue
        vectors = [r["delta_i_pu"][0] + r["delta_i_pu"][1] +
                   r["delta_u_pu"][0] + r["delta_u_pu"][1] for r in chunk]
        averaged = [statistics.mean(axis) for axis in zip(*vectors)]
        ratio, gate, det, response, x, y = row_metrics(
            averaged, result["voltage_threshold_pu"], result["response_threshold_pu"],
            result["saliency_gate"])
        axis = ((math.atan2(y, x) / (4 * math.pi)) % 0.5
                if gate == "accepted" else None)
        blocks.append({"first_frame": chunk[0]["frame"],
                       "last_frame": chunk[-1]["frame"],
                       "native_axis_ok_groups": sum(r["axis_ok"] for r in chunk),
                       "avg_delta_i_pu": [averaged[:2], averaged[2:4]],
                       "avg_delta_u_pu": [averaged[4:6], averaged[6:]],
                       "det_u_pu2": det, "response_pu": response,
                       "saliency_l1": ratio, "offline_axis_gate": gate,
                       "axis_turns_mod_half": axis})
        chunk.clear()
    return blocks


def decode_guard(data, fixed):
    fmt = struct.Struct("<2I4i" if fixed else "<2I4f")
    samples, trip, peak, u, v, w = fmt.unpack(data)
    scale = 32768 if fixed else 1
    limit = round(0.12 * scale) / scale if fixed else 0.12
    if trip > samples or (trip and max(map(abs, (u, v, w))) / scale <= limit):
        raise ValueError("incoherent stopped phase guard capture")
    return {"samples": samples, "current_limit_pu": limit,
            "first_trip_sample": trip if trip else None,
            "peak_abs_phase_pu": peak / scale,
            "trip_current_pu": [x / scale for x in (u, v, w)] if trip else None}


def encoder_reference(before, after, pole_pairs):
    delta = (after - before + 2048) % 4096 - 2048
    mechanical_turns = delta / 4096
    return {"before_count": before, "after_count": after,
            "pole_pairs": pole_pairs, "mechanical_delta_deg": 360 * mechanical_turns,
            "electrical_delta_deg_mod_half": 360 * (
                (pole_pairs * mechanical_turns + 0.25) % 0.5 - 0.25)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path, help="binary g_p10_hfi_capture bytes")
    parser.add_argument("--guard", type=Path, help="binary g_p10_hfi_guard bytes")
    parser.add_argument("--encoder-before", type=int, help="stopped raw AS5600 count")
    parser.add_argument("--encoder-after", type=int, help="stopped raw AS5600 count")
    parser.add_argument("--pole-pairs", type=int, default=7)
    args = parser.parse_args()
    ref = (args.encoder_before, args.encoder_after)
    if ref != (None, None) and (any(x is None or not 0 <= x < 4096 for x in ref)
                                or args.pole_pairs <= 0):
        parser.error("provide both stopped encoder counts in 0..4095 and positive pole pairs")
    result = decode(args.capture.read_bytes())
    rows = result["records"]
    prefix = result["consecutive_prefix_size"]
    result[f"first_{prefix}_groups"] = summarize(rows[:prefix])
    result["sparse_tail"] = summarize(rows[prefix:])
    result["acquisition_axis_spread_modulo_pi"] = axis_spread(
        [row for row in rows if row["acquire"]])
    tracking = [row for row in rows if row["track"] and row["axis_ok"]]
    result["tracking_axis_spread_modulo_pi"] = axis_spread(tracking)
    result["tracking_pll_spread_modulo_pi"] = axis_spread(tracking, "tracker_turns")
    result["complete_32_group_averages"] = coherent_averages(result)
    if args.guard:
        result["phase_guard"] = decode_guard(args.guard.read_bytes(),
                                             result["numeric"] == "FIXED")
    if ref != (None, None):
        result["stopped_encoder_reference"] = encoder_reference(*ref, args.pole_pairs)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
