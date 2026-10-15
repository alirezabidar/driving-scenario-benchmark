"""One-panel version of the trace chart: distance to the pedestrian over time, both controllers.

    python analysis/plot_trace_simple.py L00_S03   ->  results/trace-L00_S03.svg
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
BG, INK, MUTED, GRID = '#11181e', '#e6ebee', '#a6b0b7', '#26313a'
AMBER, CYAN, RED = '#e8a33d', '#79d1dc', '#d9534f'
CLIP = 14.0          # draw gaps above this at the top edge
T0, T1 = 8.0, 26.0   # time window that contains the encounter


def load(sensor, name):
    tr = json.loads((ROOT / 'results' / sensor / 'traces' / f'{name}.json').read_text())
    run = json.loads((ROOT / 'results' / sensor / 'runs' / f'{name}.json').read_text())
    t = [p['t'] for p in tr if T0 <= p['t'] <= T1]
    gap = [min(CLIP, max(0.0, p['ped_gap'])) for p in tr if T0 <= p['t'] <= T1]
    return t, gap, run


def main(name):
    tn, gn, rn = load('narrow', name)
    tw, gw, rw = load('wide', name)

    fig, ax = plt.subplots(figsize=(9, 5.25), facecolor=BG)
    ax.set_facecolor(BG)
    ax.plot(tn, gn, color=AMBER, lw=2.4, solid_capstyle='round', zorder=3)
    ax.plot(tw, gw, color=CYAN, lw=2.4, solid_capstyle='round', zorder=3)

    # first detection (same moment in both runs)
    td = rw['first_detect_t']
    ax.axvline(td, color=MUTED, lw=0.8, ls=(0, (3, 3)), zorder=1)
    ax.text(td + 0.15, 13.7, f'Pedestrian first detected ({td:.1f} s)', color=MUTED, fontsize=9, ha='left', va='top')

    # original: contact
    if rn['collision']:
        tc = rn['sim_time_s']
        ax.scatter([tc], [0], s=120, marker='X', color=RED, edgecolor=BG, linewidth=1.5, zorder=5)
        ax.text(tc + 0.35, 0.55, f'Contact at {tc:.1f} s', color=INK, fontsize=10.5, fontweight='bold')

    # direct labels
    ax.text(8.6, 2.6, 'Original\n3 forward rays', color=INK, fontsize=10.5, ha='left', va='center')
    ax.plot([10.4, 13.55], [2.6, 2.3], color=AMBER, lw=1)
    ax.text(19.6, 0.9, 'Revised\nfull-width rays + side watch', color=INK, fontsize=10.5, va='center')
    ax.plot([19.55, 18.4], [1.3, 2.3], color=CYAN, lw=1)

    # what the revised car does
    box = dict(boxstyle='round,pad=0.35', fc='#1a242c', ec=GRID)
    ax.text(15.0, 4.6, f"Stops {rw['min_gap_while_stopped_m']:.1f} m short and waits", color=INK, fontsize=10, bbox=box)
    ax.text(19.6, 9.4, f"Drives on once they've crossed\n(closest: {rw['min_ped_gap_m']:.1f} m)", color=INK,
            fontsize=10, bbox=box)

    ax.set_xlim(T0, T1)
    ax.set_ylim(-0.5, CLIP)
    ax.set_yticks([0, 3, 6, 9, 12])
    ax.set_xlabel('Simulation time (s)', color=MUTED, fontsize=10.5)
    ax.set_ylabel('Distance to pedestrian (m)', color=MUTED, fontsize=10.5)
    ax.tick_params(colors=MUTED, labelsize=9.5, length=0)
    ax.grid(axis='y', color=GRID, lw=0.7)
    ax.set_axisbelow(True)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.axhline(0, color=MUTED, lw=0.9)

    fig.suptitle('Same crossing, two controllers: contact vs. a safe stop', color=INK, fontsize=14,
                 fontweight='bold', x=0.075, ha='left', y=0.965)
    fig.text(0.075, 0.885, f'Layout {rw["layout"]}, scenario {rw["scenario"]}: pedestrian steps off the right curb',
             color=MUTED, fontsize=10.5, ha='left')
    fig.tight_layout(rect=[0, 0, 1, 0.9])
    out = ROOT / 'results' / f'trace-{name}.svg'
    fig.savefig(out, facecolor=BG)
    fig.savefig(out.with_suffix('.png'), facecolor=BG, dpi=150)
    print('wrote', out)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'L00_S03')
