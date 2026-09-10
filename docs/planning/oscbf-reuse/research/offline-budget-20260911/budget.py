"""Offline arithmetic draft only; never grants robot admission or sets thresholds."""
import argparse
import json
import math
from pathlib import Path


class Invalid(ValueError):
    pass


def number(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Invalid(f"{name}: missing or not numeric")
    if not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise Invalid(f"{name}: must be finite and {'positive' if positive else 'nonnegative'}")
    return value


def bound(obj, name):
    item = obj.get(name)
    if not isinstance(item, dict):
        raise Invalid(f"{name}: missing bound object")
    if item.get('basis') not in ('synthetic_assumption', 'documented_bound'):
        raise Invalid(f"{name}: sample statistics are not bounds")
    if not isinstance(item.get('evidence'), str) or not item['evidence'].strip():
        raise Invalid(f"{name}: missing evidence reference")
    return number(item.get('value'), name)


def evaluate(data):
    """Constant relative-speed envelope; acquisition times already mapped to consumer domain."""
    if data.get('purpose') != 'offline_draft':
        raise Invalid('purpose must be offline_draft')
    for key in ('clock_domain', 'session'):
        if not isinstance(data.get(key), str) or not data[key]:
            raise Invalid(f'missing {key}')
    consume = number(data.get('consume_s'), 'consume_s', positive=True)
    phases = data.get('intervals')
    names = ['remaining_control', 'final_command', 'transport', 'actuator_reaction']
    if not isinstance(phases, list) or [p.get('name') for p in phases] != names:
        raise Invalid('provide all four ordered consumption-to-brake intervals')
    cursor = consume
    for phase in phases:
        start = number(phase.get('start_s'), 'interval.start_s', positive=True)
        end = number(phase.get('end_s'), 'interval.end_s', positive=True)
        if start != cursor or end < start:
            raise Invalid('interval overlap, uncovered gap, or reversed endpoints')
        if phase.get('basis') not in ('synthetic_assumption', 'documented_bound'):
            raise Invalid('interval: sample statistics are not bounds')
        if not isinstance(phase.get('evidence'), str) or not phase['evidence']:
            raise Invalid('interval: missing evidence')
        cursor = end
    remaining = cursor - consume
    d0 = bound(data, 'minimum_clearance_m')
    required = data.get('required_sources')
    sources = data.get('sources')
    if not isinstance(required, list) or not required or any(not isinstance(x, str) or not x for x in required):
        raise Invalid('missing required sources')
    if len(set(required)) != len(required) or not isinstance(sources, list):
        raise Invalid('duplicate source requirement or missing sources')
    ids = [s.get('id') for s in sources]
    if len(set(ids)) != len(ids) or set(ids) != set(required):
        raise Invalid('source coverage mismatch or duplicate source')
    rows = []
    for source in sources:
        if source.get('clock_trusted') is not True or source.get('session_trusted') is not True:
            raise Invalid('untrusted clock or session')
        if source.get('clock_domain') != data['clock_domain'] or source.get('session') != data['session']:
            raise Invalid('clock domain or session mismatch')
        start = number(source.get('acquisition_start_s'), 'acquisition_start_s', positive=True)
        end = number(source.get('acquisition_end_s'), 'acquisition_end_s', positive=True)
        if start > end or end > consume:
            raise Invalid('future or reversed acquisition interval; no clamping')
        if source.get('geometry_reference') != 'acquisition_without_motion_inflation':
            raise Invalid('unsupported geometry baseline; potential double motion compensation')
        age = consume - start + bound(source, 'clock_error_s')
        speed = bound(source, 'relative_speed_m_s')
        space = bound(source, 'available_separation_m') - d0 - bound(source, 'geometry_error_m') - bound(source, 'stop_relative_displacement_m')
        # Stop displacement includes robot braking AND obstacle approach after brake onset.
        residual = space - speed * age
        allowed = residual / speed if speed > 0 else None
        margin = residual - speed * remaining
        if not all(math.isfinite(x) for x in (age, space, residual, margin)) or (allowed is not None and not math.isfinite(allowed)):
            raise Invalid('arithmetic overflow')
        rows.append({'id': source['id'], 'age_s': age, 'available_remaining_time_s': allowed,
                     'zero_speed_note': 'no finite time limit from this spatial formula' if speed == 0 else None,
                     'space_after_age_m': residual, 'margin_m': margin,
                     'arithmetic_feasible': margin > 0})
    limits = [r['available_remaining_time_s'] for r in rows if r['available_remaining_time_s'] is not None]
    return {'status': 'offline_arithmetic_only', 'robot_admission': False,
            'remaining_time_s': remaining, 'most_restrictive_remaining_time_s': min(limits) if limits else None,
            'arithmetic_feasible': all(r['arithmetic_feasible'] for r in rows), 'sources': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    args = parser.parse_args()
    try:
        result = evaluate(json.loads(args.input.read_text()))
    except (Invalid, KeyError, TypeError, AttributeError, ValueError) as exc:
        print(json.dumps({'status': 'rejected', 'reason': str(exc)}, ensure_ascii=False))
        return 2
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['arithmetic_feasible'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
