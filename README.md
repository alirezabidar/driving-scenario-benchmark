# Driving Scenario Benchmark

Does a pure-pursuit driving controller hold up when the city and the pedestrian
change? This project generates **10 street layouts** from the Houdini HDA in
[procedural-streets-isaac-sim](https://github.com/alirezabidar/procedural-streets-isaac-sim) and runs **10 pedestrian-crossing scenarios** on each, headless in Isaac Sim,
with every run logged and classified.

Status: **200 runs in Isaac Sim 6.0.1 on an RTX 3080**: 10 layouts × 10
scenarios, each under two sensing conditions.

## Setup

`houdini/` holds a copy of the Street District HDA from
[procedural-streets-isaac-sim](https://github.com/alirezabidar/procedural-streets-isaac-sim)
and the TOP network that exports the layouts, so this repo runs on its own.

## Results

| | Original: 3 forward rays | Revised: full-width rays + side watch |
|---|---:|---:|
| Completed without contact | **65 / 100** (95% CI 55–74%) | **96 / 100** (95% CI 90–98%) |
| Car drove into pedestrian (front) | 2 | 0 |
| Pedestrian walked into side of car | 33 | 4 |
| Curb strikes / off-route / timeouts | 0 / 0 / 0 | 0 / 0 / 0 |
| Max path error, all runs | 0.30 m | 0.30 m |
| Mean time to finish (runs both completed) | 32.8 s | 34.6 s |
| Median simulation speed | 1.6× real time | 1.7× real time |

Classifying each scenario by its setup (before looking at outcomes) explains the
original controller completely: it succeeded in **all 65** scenarios where the
pedestrian crossed clearly before or after the car, and failed in **all 35** where
they stepped out as the car arrived. Forward rays cannot see someone approaching
from the curb.

The revision fixed 33 runs and broke 2 (L00_S06, L06_S01). In both, slowing for a
person on the curb changed the timing so that they walked into the car's side. The
remaining failures, L05_S05 and L05_S08, are late step-outs that also fail in the
original. Figures: `results/comparison.svg`, `results/*/outcome-matrix.svg`,
`results/trace-L00_S03.svg`, `results/layouts-grid.png`.

### What the revision does

`sim/perception.py` adds eight angled rays (±15–60°) from the front bumper. The
nearest object within 6 m of the centre line is tracked; if it is moving toward
the lane fast enough to arrive before the car has passed (constant-velocity
prediction), the car yields as if to an obstacle at that distance. A person within
4 m of the lane but not approaching caps speed at 1.2 m/s. The in-path rays are
widened to the car's full width.

## Pipeline

1. `sim/scenarios.py` writes `configs/benchmark_v1.json` from one master seed
   (2026): layout controls for the HDA and crossing parameters.
2. In `houdini/procedural_driving.hip`, the TOP network `/obj/export_layouts`
   reads each layout from `configs/benchmark_v1.json` and exports
   `layouts/<id>/city.usdc`:

   | Node | What it does |
   |---|---|
   | `read_benchmark_config` (JSON Input) | One work item per layout; the layout controls become work item attributes. |
   | `read_layout_id` (JSON Input) | Adds the layout id (`L00` to `L09`). |
   | `export_city_usd` (ROP Fetch) | Runs `USD_EXPORT` once per layout. |

   The export runs on `/obj/export_district`, a second copy of the HDA whose layout
   controls are `pdgattrib()` expressions, so the main HDA keeps normal sliders.
   The car here is built at run time by NVIDIA's BasicSetup, so the HDA's
   **Vehicle** tab uses the typed-in car size (2.15 × 4.0 m) for the curb check. The HDA's
   `layout_rules` node checks the route against the sidewalks before anything is
   written.
3. `sim/benchmark.py` (Isaac Sim Python) builds one stage per layout and runs
   each scenario as stop → reset → play. A kinematic capsule stands in for the
   pedestrian and starts walking when the car is a set distance away.
4. `analysis/summarize.py` turns `results/<sensor>/runs/*.json` into tables and
   figures.

## Run it

```powershell
# 1. layouts: in Houdini, cook /obj/export_layouts/export_city_usd
# 2. one smoke test (add --sensor wide for the revised controller)
C:\isaacsim\python.bat sim\run.py sim\benchmark.py --layout L00 --scenarios S00
# 3. everything
powershell -ExecutionPolicy Bypass -File run_all.ps1 -Sensor narrow
powershell -ExecutionPolicy Bypass -File run_all.ps1 -Sensor wide
# 4. tables and figures
python analysis\summarize.py narrow
python analysis\summarize.py wide
python analysis\compare.py narrow wide
```

### Route check (car only)

`sim/route_check.py` drives every layout with no pedestrian at the three
benchmark speeds (3.0, 3.5 and 4.0 m/s) and checks the car body against the
raised sidewalks at every physics step:

```powershell
foreach ($id in 'L00','L01','L02','L03','L04','L05','L06','L07','L08','L09') {
    C:\isaacsim\python.bat sim\run.py sim\route_check.py --layout $id }
```

Result: **30 / 30 runs completed, 0 sidewalk overlaps,
0 wrong-lane samples**. Smallest body-to-curb gap 0.11 m (10 m roads, right
turn); maximum path error 0.44 m. Results: `results/route_check/`.

Offline checks that need neither Houdini nor Isaac Sim:
`python -m pytest tests` and `python tests/dry_run.py` (a bicycle-model mock used
only to catch logic errors; its numbers are not results).

## What is measured

| Metric | Definition |
|---|---|
| Outcome | `completed`, `collision_front`, `collision_side`, `curb_strike`, `off_route` (>2 m from route), `timeout` |
| Min pedestrian gap | Separating distance between a 2.05 × 4.90 m car rectangle and the 0.5 m pedestrian square |
| Lane-entry gap | Distance from car front to the crossing when the pedestrian enters the car's path |
| Path error | RMS and max distance from the route |
| Resume delay | Time from the pedestrian leaving the path to the car exceeding 0.5 m/s |
| Real-time factor | Simulated seconds per wall-clock second |

Front contact means the car drove into the pedestrian. Side contact means the
pedestrian walked into a car already passing; forward sensing cannot see that
coming. Both are failures.

## Limits

One seed per scenario and deterministic physics, so each pair is run once; the
scenario set was fixed before the revision was written, but the revision was
tuned on the dry-run mock of the same ten scenarios. Simulator ground-truth pose; PhysX ray queries, not LiDAR; a straight-walking
kinematic capsule, not an animated character; one route template that scales
with the layout rather than route planning; trees and lamps have no colliders.
The vehicle comes from NVIDIA's PhysX vehicle sample (BasicSetup).
