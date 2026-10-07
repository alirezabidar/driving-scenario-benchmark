"""Benchmark layouts, crossing scenarios and pedestrian geometry.

Plain Python + NumPy, so it can be tested without Houdini or Isaac Sim.
All randomness comes from one recorded master seed.
"""
import json
import math
from pathlib import Path

import numpy as np

# Forward ray layouts (lateral offsets in metres from the car's centre line).
# The car is 2.05 m wide; 'narrow' leaves its outer 0.4 m on each side unseen.
RAY_OFFSETS = {
    'narrow': (-0.65, 0.0, 0.65),
    # 'wide' widens the in-path rays to the car's full width and adds the side
    # watch in perception.py.
    'wide': (-1.0, -0.5, 0.0, 0.5, 1.0),
}


# Constraints enforced by the linked HDA export (see linked_export_module.py).
MIN_ROAD_WIDTH = 10.0


def min_pitch_x(w):
    return 2.6 * w + 2.0


def min_pitch_z(w):
    return 2.0 * w + 10.0


def sample_layouts(rng, count):
    """Layout controls for the linked Street District HDA."""
    layouts = []
    for i in range(count):
        if i == 0:
            # Layout 0 is always the portfolio default, for comparison with earlier runs.
            c = dict(blocks_x=3, blocks_z=2, pitch_x=42.0, pitch_z=38.0,
                     road_width=10.0, sidewalk=2.2, seed=17)
        else:
            w = float(rng.choice([10.0, 11.0, 12.0]))
            c = dict(
                blocks_x=int(rng.choice([2, 3, 4])),
                blocks_z=int(rng.choice([2, 3])),
                pitch_x=round(float(rng.uniform(min_pitch_x(w), 50.0)), 1),
                pitch_z=round(float(rng.uniform(min_pitch_z(w), 48.0)), 1),
                road_width=w,
                sidewalk=round(float(rng.uniform(1.8, 3.0)), 2),
                seed=int(rng.integers(1, 10_000)),
            )
        assert c['road_width'] >= MIN_ROAD_WIDTH
        assert c['pitch_x'] >= min_pitch_x(c['road_width'])
        assert c['pitch_z'] >= min_pitch_z(c['road_width'])
        layouts.append(dict(id=f'L{i:02d}', controls=c))
    return layouts


def sample_scenarios(rng, count):
    """Pedestrian crossings. Geometry-independent; mapped onto each route at run time."""
    out = []
    for i in range(count):
        out.append(dict(
            id=f'S{i:02d}',
            crossing_u=round(float(rng.uniform(0, 1)), 4),     # where on the eligible straights
            trigger_distance_m=round(float(rng.uniform(5.0, 30.0)), 2),  # car distance when walking starts
            walk_speed_mps=round(float(rng.uniform(0.8, 1.6)), 2),
            from_right=bool(rng.integers(0, 2)),              # starts on the car's right-hand sidewalk
            target_speed_mps=float(rng.choice([3.0, 3.5, 4.0])),
        ))
    return out


def make_config(master_seed=2026, n_layouts=10, n_scenarios=10):
    rng = np.random.default_rng(master_seed)
    return dict(
        name='driving-benchmark-v1',
        master_seed=master_seed,
        layouts=sample_layouts(rng, n_layouts),
        scenarios=sample_scenarios(rng, n_scenarios),
        notes='Every layout runs every scenario. Scenario values are mapped onto each route at run time.',
    )


# ----------------------------------------------------------------- route geometry

def route_arclength(route):
    return np.r_[0, np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]


def straight_mask(route, window=4, tol_deg=1.0):
    """True where the route heading is constant over +-window samples."""
    d = np.diff(route, axis=0)
    heading = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    heading = np.r_[heading, heading[-1]]
    mask = np.zeros(len(route), bool)
    for i in range(len(route)):
        a, b = max(0, i - window), min(len(route) - 1, i + window)
        mask[i] = np.ptp(heading[a:b + 1]) < math.radians(tol_deg)
    return mask


def eligible_indices(route, start_margin=14.0, end_margin=10.0):
    """Straight route samples where a crossing makes sense.

    The start margin leaves room to accelerate; the end margin keeps the crossing
    separate from the end-of-route stop.
    """
    s = route_arclength(route)
    ok = straight_mask(route) & (s > start_margin) & (s < s[-1] - end_margin)
    idx = np.flatnonzero(ok)
    if len(idx) == 0:
        raise ValueError('Route has no eligible straight section for a crossing')
    return idx


def crossing_geometry(route, scenario, road_width, sidewalk):
    """Return the crossing point, the route tangent and the pedestrian's start/end.

    The route sits in the right-hand lane, a quarter of the road width right of
    the road centre. The pedestrian walks from the middle of one sidewalk to the
    middle of the other, perpendicular to the route.
    """
    idx = eligible_indices(route)
    i = int(idx[min(len(idx) - 1, int(scenario['crossing_u'] * len(idx)))])
    s = route_arclength(route)
    a, b = route[max(0, i - 2)], route[min(len(route) - 1, i + 2)]
    t = (b - a) / np.linalg.norm(b - a)
    # Y-up: right = forward x up. Facing +Z gives -X; facing +X gives +Z.
    # In planar (x, z) components that is (-t_z, t_x).
    right = np.array([-t[1], t[0]])
    p = route[i]
    to_right_kerb = road_width / 4.0          # lane centre -> right road edge
    to_left_kerb = 3.0 * road_width / 4.0     # lane centre -> left road edge
    right_walk = p + right * (to_right_kerb + sidewalk / 2.0)
    left_walk = p - right * (to_left_kerb + sidewalk / 2.0)
    start, end = (right_walk, left_walk) if scenario['from_right'] else (left_walk, right_walk)
    return dict(index=i, s=float(s[i]), point=p, tangent=t, right=right, start=start, end=end)


def pedestrian_position(geom, walk_time, speed):
    """Kinematic pedestrian: straight line at constant speed, then stands still."""
    span = geom['end'] - geom['start']
    length = float(np.linalg.norm(span))
    d = min(length, max(0.0, walk_time * speed))
    return geom['start'] + span * (d / length)


def braking_distance(speed, decel=1.0, margin=3.0):
    """Distance the controller needs to stop: its speed cap is sqrt(2*decel*(d - margin))."""
    return speed * speed / (2.0 * decel) + margin


def write_config(path, **kw):
    cfg = make_config(**kw)
    Path(path).write_text(json.dumps(cfg, indent=2))
    return cfg


if __name__ == '__main__':
    import sys
    out = sys.argv[1] if len(sys.argv) > 1 else 'configs/benchmark_v1.json'
    cfg = write_config(out)
    print(f"wrote {out}: {len(cfg['layouts'])} layouts x {len(cfg['scenarios'])} scenarios")
