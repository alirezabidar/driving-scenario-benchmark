"""Compare sensing conditions on the same 100 layout x scenario pairs.

    python analysis/compare.py narrow wide   ->  results/comparison.json, comparison.svg
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'analysis'))
from summarize import OUTCOMES, band, encounter, summarize, time_to_conflict, wilson  # noqa: E402

COLORS = {'completed': '#3fae8c', 'collision_front': '#d9534f', 'collision_side': '#e8a33d',
          'kerb_strike': '#8e6bbf', 'off_route': '#5b8fd1', 'timeout': '#7f8a93'}
LABELS = {'narrow': 'Original: 3 forward rays', 'wide': 'Revised: full-width rays + side watch'}


def load(sensor):
    runs = {}
    for p in sorted((ROOT / 'results' / sensor / 'runs').glob('*.json')):
        r = json.loads(p.read_text())
        r['ttc_band'] = band(time_to_conflict(r))
        r['encounter'] = encounter(r)
        runs[(r['layout'], r['scenario'])] = r
    return runs


def main(a='narrow', b='wide'):
    A, B = load(a), load(b)
    pairs = sorted(set(A) & set(B))
    fixed = [k for k in pairs if not A[k]['success'] and B[k]['success']]
    broke = [k for k in pairs if A[k]['success'] and not B[k]['success']]
    sa, sb = summarize(a), summarize(b)
    out = dict(
        paired_runs=len(pairs),
        **{f'{s}_success': sum(R[k]['success'] for k in pairs) for s, R in ((a, A), (b, B))},
        **{f'{s}_ci95': wilson(sum(R[k]['success'] for k in pairs), len(pairs)) for s, R in ((a, A), (b, B))},
        fixed_by_revision=len(fixed), broken_by_revision=len(broke),
        broken_examples=[f'{l}_{s}' for l, s in broke[:10]],
        **{f'{s}_outcomes': d['outcomes'] for s, d in ((a, sa), (b, sb))},
        **{f'{s}_median_rtf': d['real_time_factor_median'] for s, d in ((a, sa), (b, sb))},
        mean_completion_time_s={s: sum(R[k]['sim_time_s'] for k in pairs if A[k]['success'] and B[k]['success']) /
                                max(1, sum(A[k]['success'] and B[k]['success'] for k in pairs)) for s, R in ((a, A), (b, B))},
    )
    (ROOT / 'results' / 'comparison.json').write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))

    fig, ax = plt.subplots(figsize=(9, 2.8), facecolor='#11181e')
    ax.set_facecolor('#11181e')
    for row, (s, R) in enumerate(((b, B), (a, A))):
        left = 0
        for o in OUTCOMES:
            n = sum(R[k]['outcome'] == o for k in pairs)
            if n:
                ax.barh(row, n, left=left, color=COLORS[o], height=.6)
                if n >= 3:
                    ax.text(left + n / 2, row, str(n), ha='center', va='center', color='#0b1014', fontsize=10, fontweight='bold')
                left += n
    ax.set_yticks([0, 1], [LABELS.get(b, b), LABELS.get(a, a)], color='#c3cbd0', fontsize=10)
    ax.set_xlim(0, len(pairs))
    ax.set_xlabel(f'Runs (same {len(pairs)} layout × scenario pairs)', color='#a6b0b7')
    ax.tick_params(colors='#a6b0b7', length=0)
    for sp in ax.spines.values():
        sp.set_visible(False)
    used = [o for o in OUTCOMES if any(R[k]['outcome'] == o for R in (A, B) for k in pairs)]
    ax.legend([plt.Rectangle((0, 0), 1, 1, color=COLORS[o]) for o in used], [o.replace('_', ' ') for o in used],
              loc='upper center', bbox_to_anchor=(.5, -.35), ncol=len(used), frameon=False, labelcolor='#c3cbd0', fontsize=9)
    fig.tight_layout()
    fig.savefig(ROOT / 'results' / 'comparison.svg', facecolor=fig.get_facecolor())
    return out


if __name__ == '__main__':
    main(*sys.argv[1:3])
