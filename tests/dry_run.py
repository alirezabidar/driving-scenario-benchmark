"""Offline dry run: the same controller, scenarios and metrics with a kinematic bicycle
model instead of PhysX. It catches logic errors before spending GPU time. It is NOT
evidence of vehicle behaviour, and its numbers never go on the website.

    python tests/dry_run.py
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sim'))
from controller import RouteController, district_route  # noqa: E402
from road_validation import separation, vehicle_polygon  # noqa: E402
from perception import FAN_ANGLES_DEG, FAN_RANGE_M, SideWatch  # noqa: E402
from scenarios import (RAY_OFFSETS, crossing_geometry, make_config, pedestrian_position)  # noqa: E402

HZ, WHEELBASE, MAX_STEER = 120, 2.6, 0.554264
PED_R, HALF_LEN, HALF_W = 0.25, 2.45, 1.025


def ray_hit(origin, d, centre, r, max_range=30.0):
    oc = origin - centre
    b = float(np.dot(oc, d))
    c = float(np.dot(oc, oc)) - r * r
    disc = b * b - c
    if disc < 0:
        return max_range
    t = -b - math.sqrt(disc)
    return t if 0 <= t < max_range else max_range


def run(scn, route=None, w=10.0, sidewalk=2.2, sensor='narrow'):
    route = district_route() if route is None else route
    ctl = RouteController(scn['target_speed_mps'], route)
    geom = crossing_geometry(route, scn, w, sidewalk)
    L = ctl.distance[-1]
    pos = route[0].astype(float).copy()
    yaw = math.atan2(*(route[1] - route[0])[::-1])
    speed, walk0, min_gap, held = 0.0, None, 1e9, 0
    watch = SideWatch(1 / HZ)
    for step in range(int(HZ * 120)):
        t = step / HZ
        fwd = np.array([math.cos(yaw), math.sin(yaw)])
        pp = pedestrian_position(geom, 0 if walk0 is None else t - walk0, scn['walk_speed_mps'])
        right = np.array([-fwd[1], fwd[0]])
        dist = min(ray_hit(pos + fwd * 2.65 + right * off, fwd, pp, PED_R) for off in RAY_OFFSETS[sensor])
        if sensor == 'wide':
            hits = []
            for a in np.radians(FAN_ANGLES_DEG):
                # positive angle = towards the driver's right
                d = fwd * math.cos(a) + right * math.sin(a)
                t_hit = ray_hit(pos + fwd * HALF_LEN, d, pp, PED_R, FAN_RANGE_M)
                if t_hit < FAN_RANGE_M:
                    hits.append((t_hit * math.cos(a), t_hit * math.sin(a)))
            dist = min(dist, watch.update(hits, speed))
        cmd = ctl.update(pos, fwd, speed, dist)
        if walk0 is None and geom['s'] - (cmd['progress'] * L + HALF_LEN) <= scn['trigger_distance_m']:
            walk0 = t
        acc = 3.0 * cmd['throttle'] - 7.0 * cmd['brake']
        speed = max(0.0, speed + acc / HZ)
        # controller steer +1 = +X when facing +Z, i.e. towards the driver's left
        delta = -cmd['steer'] * MAX_STEER
        yaw += speed / WHEELBASE * math.tan(delta) / HZ
        pos = pos + speed * np.array([math.cos(yaw), math.sin(yaw)]) / HZ
        gap = separation(vehicle_polygon(pos, fwd), np.array(
            [[pp[0] - PED_R, pp[1] - PED_R], [pp[0] + PED_R, pp[1] - PED_R],
             [pp[0] + PED_R, pp[1] + PED_R], [pp[0] - PED_R, pp[1] + PED_R]]))
        min_gap = min(min_gap, gap)
        if gap < 0:
            zone = 'front' if float(np.dot(pp - pos, fwd)) > HALF_LEN - 0.5 else 'side'
            return 'collision_' + zone, min_gap, cmd['cte']
        if cmd['cte'] > 2.0:
            return 'off_route', min_gap, cmd['cte']
        held = held + 1 if (cmd['remaining'] < 1.2 and speed < 0.15) else 0
        if held >= 2 * HZ:
            return 'completed', min_gap, cmd['cte']
    return 'timeout', min_gap, cmd['cte']


if __name__ == '__main__':
    sensor = sys.argv[1] if len(sys.argv) > 1 else 'narrow'
    for scn in make_config()['scenarios']:
        out, gap, cte = run(scn, sensor=sensor)
        print(scn['id'], f"{out:10s} min_gap={gap:5.2f} m  trigger={scn['trigger_distance_m']:5.1f} m  "
              f"v={scn['target_speed_mps']}  walk={scn['walk_speed_mps']}  from_right={scn['from_right']}")
