"""Top-down view (as seen from above: the simulation is Y-up, so world +Z points down the page) of one encounter, before and after the revision.

Blue = car, red = pedestrian. Dots mark where each one was at the same second,
so you can see where they meet.

    python analysis/plot_encounter_map.py L00_S03  ->  results/encounter-L00_S03.svg
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
BG, INK, MUTED, GRID = '#11181e', '#e6ebee', '#a6b0b7', '#26313a'
ROAD, WALK = '#1b252d', '#232f38'
CAR, PED, HIT = '#4f9be8', '#e5534b', '#ffffff'
X0, X1, Z0, Z1 = -17.0, 2.5, -6.6, 7.6


def load(sensor, name):
    tr = json.loads((ROOT / 'results' / sensor / 'traces' / f'{name}.json').read_text())
    run = json.loads((ROOT / 'results' / sensor / 'runs' / f'{name}.json').read_text())
    return tr, run


def at(tr, t):
    return min(tr, key=lambda p: abs(p['t'] - t))


def car_box(ax, p, length, width, alpha):
    # car drives along +x here (heading ~0), so draw an axis-aligned box centred on its pose
    ax.add_patch(Rectangle((p['x'] - length / 2, -p['z'] - width / 2), length, width, fc=CAR, ec=BG,
                           lw=1.2, alpha=alpha, zorder=4))


def panel(ax, tr, run, title, t_end, car_times, ped_times, box_t, note, car_extra=None):
    road_w = run['controls']['road_width']
    side = run['controls']['sidewalk']
    length, width = run['road_check']['vehicle_envelope_m'][1], run['road_check']['vehicle_envelope_m'][0]
    ax.set_facecolor(BG)
    ax.axhspan(-road_w / 2, road_w / 2, color=ROAD, zorder=0)
    ax.axhspan(road_w / 2, road_w / 2 + side, color=WALK, zorder=0)
    ax.axhspan(-road_w / 2 - side, -road_w / 2, color=WALK, zorder=0)
    ax.axhline(0, color=GRID, lw=1.2, ls=(0, (6, 6)), zorder=1)

    pts = [p for p in tr if 10.5 <= p['t'] <= t_end]
    ax.plot([p['x'] for p in pts], [-p['z'] for p in pts], color=CAR, lw=2, alpha=.55, zorder=2)
    ax.plot([p['ped_x'] for p in pts], [-p['ped_z'] for p in pts], color=PED, lw=2, alpha=.55, zorder=2)

    for t in car_times:
        p = at(tr, t)
        ax.scatter([p['x']], [-p['z']], s=46, color=CAR, edgecolor=BG, lw=1.2, zorder=5)
        ax.text(p['x'], -p['z'] - 1.35, f'{t:g}s', color=CAR, fontsize=10, ha='center', va='top', zorder=6)
    if car_extra:
        t, label = car_extra
        p = at(tr, t)
        ax.text(p['x'], -p['z'] - 1.35, label, color=CAR, fontsize=10, ha='center', va='top', zorder=6)
    start = tr[0]
    ax.scatter([start['ped_x']], [-start['ped_z']], s=46, color=PED, edgecolor=BG, lw=1.2, zorder=5)
    ax.text(start['ped_x'] + 0.5, -start['ped_z'] - 0.2, 'waits on curb until 13s', color=PED, fontsize=10,
            ha='left', va='center', zorder=6)
    for t in ped_times:
        p = at(tr, t)
        ax.scatter([p['ped_x']], [-p['ped_z']], s=46, color=PED, edgecolor=BG, lw=1.2, zorder=5)
        ax.text(p['ped_x'] + 0.5, -p['ped_z'], f'{t:g}s', color=PED, fontsize=10, ha='left', va='center', zorder=6)

    end = at(tr, t_end)
    car_box(ax, at(tr, box_t), length, width, 0.35)
    ax.set_title(title, color=INK, fontsize=12.5, fontweight='bold', loc='left', pad=8)
    ax.text(X0 + 0.3, Z1 - 1.3, note, color=INK, fontsize=10.5, ha='left', va='top',
            bbox=dict(boxstyle='round,pad=0.4', fc='#1a242c', ec=GRID), zorder=7)
    ax.set_xlim(X0, X1)
    ax.set_ylim(Z0, Z1)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    return end


def main(name):
    trn, rn = load('narrow', name)
    trw, rw = load('wide', name)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 6.6), facecolor=BG)

    tc = round(rn['sim_time_s'], 1)
    end = panel(a1, trn, rn, '1. Before: car only looks straight ahead', tc, [11, 12, 13, 14],
                [14, tc], tc, f'At {tc:g} s they meet: collision', car_extra=(tc, f'{tc:g}s'))
    a1.scatter([end['ped_x']], [-end['ped_z']], s=260, marker='*', color=HIT, edgecolor=PED, lw=1.5, zorder=8)

    panel(a2, trw, rw, '2. After: car also watches the sides', 21, [11, 12, 13, 19, 20, 21],
          [14, 15, 16, 17, 18, 19, 20], 16, f"Car stops {rw['min_gap_while_stopped_m']:.0f} m short and waits;\n"
          "pedestrian crosses in front, then the car drives on", car_extra=(16, 'stopped\n15–17s'))

    # legend once, at the top
    a1.text(X0 + 0.2, Z1 - 0.4, '●', color=CAR, fontsize=12, va='top')
    a1.text(X0 + 0.9, Z1 - 0.45, 'Car', color=INK, fontsize=10, va='top')
    a1.text(X0 + 4.2, Z1 - 0.4, '●', color=PED, fontsize=12, va='top')
    a1.text(X0 + 4.9, Z1 - 0.45, 'Pedestrian', color=INK, fontsize=10, va='top')

    fig.suptitle(f'Same street, same pedestrian: where each one is, second by second',
                 color=INK, fontsize=14, fontweight='bold', x=0.04, ha='left', y=0.975)
    fig.text(0.04, 0.905, f'Top-down view of layout {rw["layout"]}, scenario {rw["scenario"]}. '
             'Car drives left to right; pedestrian crosses bottom to top. Same label (e.g. 14s) = same moment.', color=MUTED, fontsize=10)
    fig.tight_layout(rect=[0, 0, 1, 0.9])
    out = ROOT / 'results' / f'encounter-{name}.svg'
    fig.savefig(out, facecolor=BG)
    fig.savefig(out.with_suffix('.png'), facecolor=BG, dpi=150)
    print('wrote', out)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'L00_S03')
