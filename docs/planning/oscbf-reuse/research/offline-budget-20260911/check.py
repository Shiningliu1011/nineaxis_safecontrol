"""Synthetic fault replay; standard library only, no ROS, CAN, or device imports."""
from copy import deepcopy
import json
from pathlib import Path
from budget import evaluate, Invalid

ROOT = Path(__file__).resolve().parent


def b(value):
    return {'value': value, 'basis': 'synthetic_assumption', 'evidence': 'pure synthetic example; not a production threshold'}


def fixture():
    data = {'purpose': 'offline_draft', 'clock_domain': 'synthetic_monotonic', 'session': 'example-1',
            'consume_s': 100.0, 'minimum_clearance_m': b(.1), 'required_sources': ['camera', 'lidar'], 'sources': [], 'intervals': []}
    times = [100., 100.01, 100.012, 100.017, 100.02]
    for index, name in enumerate(['remaining_control', 'final_command', 'transport', 'actuator_reaction']):
        data['intervals'].append({'name': name, 'start_s': times[index], 'end_s': times[index+1],
                                  'basis': 'synthetic_assumption', 'evidence': 'synthetic only'})
    for name, start in [('camera', 99.95), ('lidar', 99.90)]:
        data['sources'].append({'id': name, 'clock_domain': data['clock_domain'], 'session': data['session'],
             'clock_trusted': True, 'session_trusted': True, 'acquisition_start_s': start,
             'acquisition_end_s': start + .01, 'clock_error_s': b(.002),
             'geometry_reference': 'acquisition_without_motion_inflation',
             'relative_speed_m_s': b(.5), 'available_separation_m': b(.4),
             'geometry_error_m': b(.02), 'stop_relative_displacement_m': b(.1)})
    return data


def run():
    base = fixture()
    (ROOT / 'synthetic-example.json').write_text(json.dumps(base, indent=2) + '\n')
    result = evaluate(base)
    # Independent hand calculation: (.4-.1-.02-.1)/.5 - .102 = .258 s.
    assert abs(result['most_restrictive_remaining_time_s'] - .258) < 1e-12
    assert abs(result['sources'][0]['age_s'] - .052) < 1e-12
    assert abs(result['sources'][1]['margin_m'] - .119) < 1e-12
    assert result['arithmetic_feasible'] and result['robot_admission'] is False
    outcomes = [{'case': 'two_sources_independent_age', 'outcome': 'passed'}]

    def reject(name, mutate):
        data = deepcopy(base)
        mutate(data)
        try:
            evaluate(data)
        except Invalid as exc:
            outcomes.append({'case': name, 'outcome': 'rejected_as_expected', 'reason': str(exc)})
        else:
            raise AssertionError(name + ' accepted')

    for field in ('relative_speed_m_s', 'clock_error_s', 'available_separation_m', 'geometry_error_m', 'stop_relative_displacement_m'):
        reject('missing_' + field, lambda d, f=field: d['sources'][0].pop(f))
    for value, name in [(float('nan'), 'nan'), (float('inf'), 'infinity'), (-.1, 'negative'), (True, 'boolean')]:
        reject(name, lambda d, v=value: d['sources'][0]['relative_speed_m_s'].update(value=v))
    reject('future', lambda d: d['sources'][0].update(acquisition_end_s=101.))
    reject('zero_stamp', lambda d: d['sources'][0].update(acquisition_start_s=0.))
    reject('reversed_scan', lambda d: d['sources'][0].update(acquisition_start_s=99.99))
    reject('clock_untrusted', lambda d: d['sources'][0].update(clock_trusted=False))
    reject('session_untrusted', lambda d: d['sources'][0].update(session_trusted=False))
    reject('session_changed', lambda d: d['sources'][0].update(session='rebooted'))
    reject('clock_domain_mismatch', lambda d: d['sources'][0].update(clock_domain='wall'))
    reject('source_missing', lambda d: d['sources'].pop())
    reject('source_duplicate', lambda d: d['sources'][1].update(id='camera'))
    reject('overlap', lambda d: d['intervals'][1].update(start_s=100.005))
    reject('uncovered_gap', lambda d: d['intervals'][1].update(start_s=100.011))
    reject('missing_phase', lambda d: d['intervals'].pop())
    reject('double_compensation', lambda d: d['sources'][0].update(geometry_reference='predicted_inflated'))
    for statistic in ('p95', 'p99', 'sample_max'):
        reject('reject_' + statistic, lambda d, s=statistic: d['intervals'][0].update(basis=s))
    exhausted = deepcopy(base)
    exhausted['sources'][1]['available_separation_m'] = b(.22)
    assert not evaluate(exhausted)['arithmetic_feasible']
    outcomes.append({'case': 'exhausted_space', 'outcome': 'infeasible_as_expected'})
    (ROOT / 'synthetic-infeasible.json').write_text(json.dumps(exhausted, indent=2) + '\n')
    invalid = deepcopy(base)
    invalid['sources'][0]['clock_trusted'] = False
    (ROOT / 'synthetic-invalid.json').write_text(json.dumps(invalid, indent=2) + '\n')
    zero = deepcopy(base)
    for source in zero['sources']:
        source['relative_speed_m_s'] = b(0.)
    assert evaluate(zero)['most_restrictive_remaining_time_s'] is None
    zero['sources'][0]['available_separation_m'] = b(.1)
    assert not evaluate(zero)['arithmetic_feasible']
    outcomes.append({'case': 'zero_speed_keeps_spatial_check', 'outcome': 'passed'})
    report = {'passed_cases': len(outcomes), 'example_result': result, 'cases': outcomes}
    (ROOT / 'results.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'passed_cases': len(outcomes), 'most_restrictive_remaining_time_s': result['most_restrictive_remaining_time_s']}))


if __name__ == '__main__':
    run()
