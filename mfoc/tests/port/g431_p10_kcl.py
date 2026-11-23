"""Summarize the binary g_p10_kcl snapshot read with the bridge closed."""
import argparse
import json
import math
from pathlib import Path
import struct

HEADER = struct.Struct('<5I2f')
BIN = struct.Struct('<I7f6H3f6H')
SECTORS = 32
STATES = ('idle', 'recording', 'done', 'aborted', 'timeout',
          'phase_current', 'control', 'speed', 'shutdown')


def decode(data):
    if len(data) != HEADER.size + BIN.size * SECTORS:
        raise ValueError('wrong KCL snapshot size')
    status, frames, arr, first, last, peak_phase, bus = HEADER.unpack_from(data)
    if status >= len(STATES) or status < 2 or arr == 0:
        raise ValueError('read a terminal capture after the bridge closes')
    observed = []
    total_sq = 0.0
    for index, values in enumerate(BIN.iter_unpack(data[HEADER.size:])):
        count, squared, residual = values[:3]
        if not count:
            continue
        total_sq += squared
        observed.append({
            'sector': index,
            'frames': count,
            'kcl_rms_pu': math.sqrt(squared / count),
            'kcl_peak_abs_pu': residual,
            'peak_phase_abs_pu': values[3:6],
            'max_modulation': math.sqrt(values[6]) / (3 * arr),
            'kcl_at_peak_v_pu': values[7],
            'max_duty': [x / arr for x in values[8:11]],
            'raw_at_peak_kcl': values[11:14],
            'phase_at_peak_kcl_pu': values[14:17],
            'before_top_compare_at_peak_kcl': values[17:20],
            'after_top_compare_at_peak_kcl': values[20:23],
        })
    if sum(row['frames'] for row in observed) != frames:
        raise ValueError('inconsistent frame and sector counts')
    if frames and (last - first) % (1 << 32) != (frames - 1) * 2 * arr:
        raise ValueError('nonconsecutive TIM2 captures')
    return {
        'status': STATES[status],
        'frames': frames,
        'arr_ticks': arr,
        'first_capture': first,
        'last_capture': last,
        'bus_pu': bus,
        'peak_phase_abs_pu': peak_phase,
        'kcl_rms_pu': math.sqrt(total_sq / frames) if frames else None,
        'max_modulation': max((row['max_modulation'] for row in observed),
                              default=None),
        'max_v_duty': max((row['max_duty'][1] for row in observed), default=None),
        'sectors': observed,
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    args = parser.parse_args()
    print(json.dumps(decode(args.capture.read_bytes()), indent=2))
