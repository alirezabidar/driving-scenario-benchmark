"""Speed and pedestrian gap over time for one run under both sensing conditions.

    python analysis/plot_trace.py L00_S03   ->  results/trace-L00_S03.svg
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
STYLE = {'narrow': ('#e8a33d', 'Original: 3 forward rays'), 'wide': ('#79d1dc', 'Revised: full-width rays + side watch')}


def main(name):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True, facecolor='#11181e')
    for sensor, (color, label) in STYLE.items():
        tr = json.loads((ROOT / 'results' / sensor / 'traces' / f'{name}.json').read_text())
        run = json.loads((ROOT / 'results' / sensor / 'runs' / f'{name}.json').read_text())
        t = [p['t'] for p in tr]
        a1.plot(t, [p['speed'] for p in tr], color=color, lw=1.8, label=f"{label} — {run['outcome'].replace('_', ' ')}")
        a2.plot(t, [max(-0.2, min(12, p['ped_gap'])) for p in tr], color=color, lw=1.8)
        if run['collision']:
            a2.plot(t[-1], 0, 'x', color='#d9534f', ms=10, mew=2.5)
    a2.axhline(0, color='#d9534f', lw=.8, ls='--')
    a1.set_ylabel('Car speed (m/s)', color='#a6b0b7')
    a2.set_ylabel('Gap to pedestrian (m)', color='#a6b0b7')
    a2.set_xlabel('Simulation time (s)', color='#a6b0b7')
    a2.set_ylim(-0.4, 12)
    for ax in (a1, a2):
        ax.set_facecolor('#11181e')
        ax.tick_params(colors='#a6b0b7')
        ax.grid(color='#26313a', lw=.6)
        for s in ax.spines.values():
            s.set_color('#2c3840')
    a1.legend(frameon=False, labelcolor='#c3cbd0', fontsize=9, loc='lower left', bbox_to_anchor=(0, 1.0), ncol=2)
    fig.tight_layout()
    out = ROOT / 'results' / f'trace-{name}.svg'
    fig.savefig(out, facecolor=fig.get_facecolor())
    print('wrote', out)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'L00_S03')
