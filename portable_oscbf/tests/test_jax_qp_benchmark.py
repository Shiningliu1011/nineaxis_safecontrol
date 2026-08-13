"""Unit tests for frozen QP benchmark evidence helpers."""

import csv

import pytest

pytestmark = pytest.mark.skip(
    reason="depends on work.jax_qp_benchmark, excluded from the portable core "
           "(not in OSCBF_PORTING_GUIDE.md Appendix A); performance "
           "benchmarking lands in M6"
)


def test_percentile_summary_includes_p99():
    from work.jax_qp_benchmark import percentile_summary_ms

    result = percentile_summary_ms([0.001, 0.002, 0.003, 0.004, 0.005])

    assert result['count'] == 5
    assert result['p50_ms'] == 3.0
    assert result['p95_ms'] == 5.0
    assert result['p99_ms'] == 5.0


def test_recorded_qp_sample_uses_previous_actuator_command(tmp_path):
    from work.jax_qp_benchmark import load_recorded_qp_sample

    path = tmp_path / 'metrics.csv'
    fields = ['t', 'rate_limit_slack', 'qp_ok'] + [
        f'{prefix}{index}' for prefix in ('q', 'u_nom', 'u_safe') for index in range(9)]
    rows = []
    for row_index in range(2):
        row = {'t': str(row_index * 0.01), 'rate_limit_slack': '0.0', 'qp_ok': '1.0'}
        for index in range(9):
            row[f'q{index}'] = str(index + row_index)
            row[f'u_nom{index}'] = str(10 + index + row_index)
            row[f'u_safe{index}'] = str(20 + index + row_index)
        rows.append(row)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    sample = load_recorded_qp_sample(path, sample_index=1)

    assert sample['sample_index'] == 1
    assert sample['q'][0] == 1.0
    assert sample['u_nom'][8] == 19.0
    assert sample['u_safe_previous'][0] == 20.0


def test_recorded_qp_sample_rejects_invalid_index(tmp_path):
    from work.jax_qp_benchmark import load_recorded_qp_sample

    path = tmp_path / 'metrics.csv'
    path.write_text('t\n0.0\n', encoding='utf-8')

    with pytest.raises(ValueError, match='sample_index'):
        load_recorded_qp_sample(path, sample_index=2)
