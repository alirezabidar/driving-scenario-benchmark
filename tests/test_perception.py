"""SideWatch behaviour on hand-made detections."""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sim'))
from perception import CAUTION_SPEED_MPS, STOP_MARGIN_M, SideWatch  # noqa: E402

DT = 1 / 120


def feed(watch, track, speed):
    """track: list of (forward, lateral) per step; returns the last output."""
    out = None
    for f, l in track:
        out = watch.update([(f, l)], speed)
    return out


def test_nothing_seen_means_no_limit():
    assert SideWatch(DT).update([], 3.0) == math.inf


def test_far_static_object_is_ignored():
    w = SideWatch(DT)
    assert feed(w, [(10.0, 5.0)] * 60, 3.0) == math.inf
    assert w.state == 'watch'


def test_person_waiting_on_kerb_slows_the_car():
    w = SideWatch(DT)
    out = feed(w, [(10.0, 3.6)] * 60, 3.0)
    assert w.state == 'caution'
    floor = STOP_MARGIN_M + CAUTION_SPEED_MPS ** 2 / 2
    assert out == 10.0
    assert feed(w, [(1.0, 3.6)] * 5, 1.2) == floor


def test_person_stepping_out_ahead_makes_the_car_yield():
    w = SideWatch(DT)
    track = [(8.0, 4.5 - 1.2 * i * DT) for i in range(60)]   # walking in at 1.2 m/s
    out = feed(w, track, 3.0)
    assert w.state == 'yield'
    assert out == 8.0


def test_person_walking_away_is_not_a_conflict():
    w = SideWatch(DT)
    feed(w, [(8.0, 2.0 + 1.2 * i * DT) for i in range(120)], 3.0)
    assert w.state in ('caution', 'watch')


def test_detection_dropout_is_bridged_then_forgotten():
    w = SideWatch(DT)
    feed(w, [(10.0, 3.6)] * 30, 3.0)
    held = w.update([], 3.0)
    assert held == 10.0
    for _ in range(70):
        out = w.update([], 3.0)
    assert out == math.inf and w.state == 'clear'
