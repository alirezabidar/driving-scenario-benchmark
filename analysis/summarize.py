"""Summarise benchmark runs into a table, an outcome matrix and headline numbers.

    python analysis/summarize.py narrow [wide ...]

Writes results/<sensor>/summary.json, runs.csv and outcome-matrix.svg.
"""
import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUTCOMES = ['completed', 'collision_front', 'collision_side', 'kerb_strike', 'off_route', 'timeout']


def wilson(k, n, z=1.96):
    """95% Wilson score interval for a success rate (honest error bars for small n)."""
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def time_to_conflict(r):
    """Seconds the car had when the pedestrian entered its path (None if it never did)."""
    if r['lane_entry_gap_m'] is None or not r['lane_entry_speed']:
        return None
    return r['lane_entry_gap_m'] / max(0.1, r['lane_entry_speed'])


def encounter(r, car_half_width=1.025, ped_r=0.25, car_len=4.9):
    """Classify the scenario as set up, independent of how the controller reacted.

    Compares when the pedestrian would reach the car's path with when the car, holding
    its target speed, would reach and clear the crossing.
    """
    p, c = r['scenario_params'], r['controls']
    w, sw, v = c['road_width'], c['sidewalk'], p['target_speed_mps']
    start_lat = (w / 4 + sw / 2) if p['from_right'] else (3 * w / 4 + sw / 2)
    t_ped = (start_lat - car_half_width - ped_r) / p['walk_speed_mps']
    t_car = p['trigger_distance_m'] / v
    t_clear = (p['trigger_distance_m'] + car_len + 0.5) / v
    if t_ped < t_car - 1.0:
        return 'pedestrian first'
    if t_ped <= t_clear:
        return 'simultaneous'
    return 'car first'


def band(ttc):
    if ttc is None:
        return 'no conflict'
    if ttc < 0:
        return 'entered beside car'
    if ttc < 1.5:
        return '< 1.5 s'
    if ttc < 3.0:
        return '1.5-3 s'
    return '> 3 s'


def summarize(sensor):
    base = ROOT / 'results' / sensor
    runs = [json.loads(p.read_text()) for p in sorted((base / 'runs').glob('*.json'))]
    if not runs:
        raise SystemExit(f'no runs in {base}')
    for r in runs:
        r['ttc_s'] = time_to_conflict(r)
        r['ttc_band'] = band(r['ttc_s'])
        r['encounter'] = encounter(r)
    n = len(runs)
    ok = sum(r['success'] for r in runs)
    by_band = {}
    for r in runs:
        b = by_band.setdefault(r['encounter'], [0, 0])
        b[0] += r['success']
        b[1] += 1
    rtf = sorted(r['real_time_factor'] for r in runs if r['real_time_factor'])
    summary = dict(
        sensor=sensor, runs=n, layouts=len({r['layout'] for r in runs}), scenarios=len({r['scenario'] for r in runs}),
        success=ok, success_rate=ok / n, success_ci95=wilson(ok, n),
        outcomes=dict(Counter(r['outcome'] for r in runs)),
        success_by_encounter={k: dict(success=v[0], runs=v[1]) for k, v in sorted(by_band.items())},
        completed_rms_cte_m=_mean([r['rms_cte_m'] for r in runs if r['success']]),
        max_cte_m=max(r['max_cte_m'] for r in runs),
        kerb_overlap_runs=sum(r['road_check']['sidewalk_overlap_samples'] > 0 for r in runs),
        stopped_for_pedestrian=sum(bool(r['stopped_for_ped']) for r in runs),
        median_min_gap_when_stopped_m=_median([r['min_gap_while_stopped_m'] for r in runs if r['min_gap_while_stopped_m'] is not None]),
        median_resume_delay_s=_median([r['resume_delay_s'] for r in runs if r['resume_delay_s'] is not None]),
        real_time_factor_median=_median(rtf),
        total_sim_s=sum(r['sim_time_s'] for r in runs), total_wall_s=sum(r['wall_time_s'] for r in runs),
    )
    (base / 'summary.json').write_text(json.dumps(summary, indent=2))
    cols = ['layout', 'scenario', 'outcome', 'encounter', 'ttc_s', 'ttc_band', 'min_ped_gap_m', 'lane_entry_gap_m',
            'lane_entry_speed', 'first_detect_m', 'stopped_for_ped', 'min_gap_while_stopped_m', 'resume_delay_s',
            'rms_cte_m', 'max_cte_m', 'progress', 'sim_time_s', 'wall_time_s', 'real_time_factor']
    with open(base / 'runs.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
        w.writeheader()
        w.writerows(runs)
    matrix_svg(runs, base / 'outcome-matrix.svg')
    print(json.dumps(summary, indent=2))
    return summary


def _mean(xs):
    return sum(xs) / len(xs) if xs else None


def _median(xs):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    m = len(xs) // 2
    return xs[m] if len(xs) % 2 else (xs[m - 1] + xs[m]) / 2


def matrix_svg(runs, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    layouts = sorted({r['layout'] for r in runs})
    scenarios = sorted({r['scenario'] for r in runs})
    colors = ['#3fae8c', '#d9534f', '#e8a33d', '#8e6bbf', '#5b8fd1', '#7f8a93']
    grid = [[None] * len(scenarios) for _ in layouts]
    for r in runs:
        grid[layouts.index(r['layout'])][scenarios.index(r['scenario'])] = OUTCOMES.index(r['outcome'])
    fig, ax = plt.subplots(figsize=(8, 6), facecolor='#11181e')
    ax.set_facecolor('#11181e')
    data = [[-1 if v is None else v for v in row] for row in grid]
    cmap = ListedColormap(['#20282e'] + colors)
    ax.imshow([[v + 1 for v in row] for row in data], cmap=cmap, vmin=0, vmax=len(OUTCOMES), aspect='auto')
    ax.set_xticks(range(len(scenarios)), scenarios, color='#c3cbd0', fontsize=9)
    ax.set_yticks(range(len(layouts)), layouts, color='#c3cbd0', fontsize=9)
    ax.set_xlabel('Crossing scenario', color='#a6b0b7')
    ax.set_ylabel('Houdini layout', color='#a6b0b7')
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_xticks([x - .5 for x in range(1, len(scenarios))], minor=True)
    ax.set_yticks([y - .5 for y in range(1, len(layouts))], minor=True)
    ax.grid(which='minor', color='#11181e', linewidth=2)
    ax.tick_params(which='both', length=0)
    used = sorted({v for row in data for v in row if v >= 0})
    handles = [plt.Rectangle((0, 0), 1, 1, color=colors[i]) for i in used]
    ax.legend(handles, [OUTCOMES[i].replace('_', ' ') for i in used], loc='upper center', bbox_to_anchor=(.5, -.1),
              ncol=len(used), frameon=False, labelcolor='#c3cbd0', fontsize=9)
    fig.tight_layout()
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == '__main__':
    for sensor in sys.argv[1:] or ['narrow']:
        summarize(sensor)
