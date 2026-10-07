"""Fast checks that need neither Houdini nor Isaac Sim:  python -m pytest tests"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sim'))
from controller import district_route  # noqa: E402
from scenarios import (crossing_geometry, eligible_indices, make_config,  # noqa: E402
                       min_pitch_x, min_pitch_z, pedestrian_position, route_arclength)

ROUTE = district_route()          # the default-layout route, built in plain Python
W, SIDEWALK = 10.0, 2.2


def test_config_is_reproducible():
    assert make_config(2026) == make_config(2026)
    assert make_config(2026) != make_config(2027)


def test_layouts_respect_hda_export_limits():
    for layout in make_config()['layouts']:
        c = layout['controls']
        assert c['road_width'] >= 10
        assert c['pitch_x'] >= min_pitch_x(c['road_width'])
        assert c['pitch_z'] >= min_pitch_z(c['road_width'])


def test_layout_zero_is_the_portfolio_default():
    c = make_config()['layouts'][0]['controls']
    assert (c['blocks_x'], c['blocks_z'], c['pitch_x'], c['pitch_z'], c['road_width']) == (3, 2, 42.0, 38.0, 10.0)


def test_crossings_are_on_straights_away_from_the_ends():
    s = route_arclength(ROUTE)
    for i in eligible_indices(ROUTE):
        assert 14.0 < s[i] < s[-1] - 10.0


def test_right_is_minus_x_when_driving_plus_z():
    scn = dict(crossing_u=0.0, from_right=True)
    g = crossing_geometry(ROUTE, scn, W, SIDEWALK)
    assert np.allclose(g['tangent'], [0, 1], atol=1e-6)
    assert np.allclose(g['right'], [-1, 0], atol=1e-6)


def test_pedestrian_crosses_both_kerbs_perpendicular_to_route():
    for u in np.linspace(0, 1, 11):
        for from_right in (True, False):
            g = crossing_geometry(ROUTE, dict(crossing_u=u, from_right=from_right), W, SIDEWALK)
            lat_start = float(np.dot(g['start'] - g['point'], g['right']))
            lat_end = float(np.dot(g['end'] - g['point'], g['right']))
            # Lane centre is W/4 right of the road centre: right kerb W/4 away, left kerb 3W/4 away.
            right_mid, left_mid = W / 4 + SIDEWALK / 2, -(3 * W / 4 + SIDEWALK / 2)
            expected = (right_mid, left_mid) if from_right else (left_mid, right_mid)
            assert np.allclose([lat_start, lat_end], expected, atol=1e-6)
            assert abs(float(np.dot(g['end'] - g['start'], g['tangent']))) < 1e-6


def test_pedestrian_walks_then_stops_at_far_kerb():
    g = crossing_geometry(ROUTE, dict(crossing_u=0.5, from_right=True), W, SIDEWALK)
    assert np.allclose(pedestrian_position(g, 0.0, 1.2), g['start'])
    assert np.allclose(pedestrian_position(g, 1e6, 1.2), g['end'])
    half = np.linalg.norm(g['end'] - g['start']) / 2 / 1.2
    assert np.allclose(pedestrian_position(g, half, 1.2), (g['start'] + g['end']) / 2)
