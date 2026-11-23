"""Decode a stopped G431 exit-witness capture read from g_p10_exit_witness."""
import argparse
import json
from pathlib import Path
import struct

MAGIC = 0x50314558
MASK = 0xffffffff
HEADER = struct.Struct('<9I')
GROUP = struct.Struct('<8I')
GROUPS = (
    'all', 'abnormal', 'smo_window', 'hfi_eighth', 'limit', 'bus',
    'speed_smo', 'speed_hfi_eighth', 'bus_speed_smo', 'bus_speed_hfi_eighth',
    'bus_speed_smo_limit', 'bus_speed_hfi_eighth_limit',
    'bus_speed_smo_hfi_eighth', 'bus_speed_smo_hfi_eighth_limit',
    'limit_only', 'speed_smo_limit',
)
FLAGS = ('speed_pi', 'smo_window', 'hfi_eighth', 'limit', 'bus', 'abnormal')
SNAPSHOT_SIZE = HEADER.size + GROUP.size * len(GROUPS)


def decode(data, period):
    if len(data) != SNAPSHOT_SIZE:
        raise ValueError('wrong exit-witness snapshot size')
    (magic, guard, armed, replaced, untagged, bad_compare, adc_active,
     bad_priority, outstanding) = HEADER.unpack_from(data)
    if magic != MAGIC or period <= guard:
        raise ValueError('invalid exit-witness snapshot or period')
    groups = {}
    for name, row in zip(GROUPS, GROUP.iter_unpack(data[HEADER.size:])):
        count, late, age, seq, trigger, compare, observed, flags = row
        if late > count or (count and age != ((observed - trigger) & MASK)):
            raise ValueError(f'invalid row for {name}')
        groups[name] = {
            'count': count,
            'late': late,
            'max_age_ticks': age if count else None,
            'guarded_trigger_margin_ticks': period - guard - age if count else None,
            'worst': {
                'seq': seq,
                'trigger': trigger,
                'compare': compare,
                'observed': observed,
                'flags': [flag for bit, flag in enumerate(FLAGS) if flags & (1 << bit)],
            } if count else None,
        }
    total = groups['all']['count']
    unresolved = replaced + bad_compare + adc_active + bad_priority + outstanding
    complete = total > 0 and total == armed and unresolved == 0 and untagged == 0
    return {
        'period_ticks': period,
        'trigger_guard_ticks': guard,
        'armed': armed,
        'replaced': replaced,
        'untagged': untagged,
        'bad_compare': bad_compare,
        'adc_active': adc_active,
        'bad_priority': bad_priority,
        'outstanding': outstanding,
        'complete': complete,
        'witness_deadline_pass': complete and groups['all']['late'] == 0,
        'groups': groups,
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--period', type=int, default=8500)
    args = parser.parse_args()
    print(json.dumps(decode(args.snapshot.read_bytes(), args.period), indent=2))
