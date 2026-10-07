# Driving Scenario Benchmark

Project page with videos and results: https://www.alirezabidar.net/robotics.html#driving-benchmark

Does the procedural-streets driving controller hold up when the city and the
pedestrian change? This project generates **10 street layouts** from the Houdini
HDA and runs **10 pedestrian-crossing scenarios** on each, headless in Isaac Sim,
with every run logged and classified.

Status: **200 runs completed in Isaac Sim 6.0.1 on an RTX 3080** (6 October 2026):
10 layouts × 10 scenarios, each under two sensing conditions.

## Results

| | Original: 3 forward rays | Revised: full-width rays + side watch |
|---|---:|---:|
| Completed without contact | **65 / 100** (95% CI 55–74%) | **96 / 100** (95% CI 90–98%) |
| Car drove into pedestrian (front) | 2 | 0 |
| Pedestrian walked into side of car | 33 | 4 |
| Kerb strikes / off-route / timeouts | 0 / 0 / 0 | 0 / 0 / 0 |
| Max path error, all runs | 0.30 m | 0.30 m |
| Mean time to finish (runs both completed) | 32.8 s | 34.6 s |
| Median simulation speed | 1.6× real time | 1.7× real time |

Classifying each scenario by its setup (before looking at outcomes) explains the
original controller completely: it succeeded in **all 65** scenarios where the
pedestrian crossed clearly before or after the car, and failed in **all 35** where
they stepped out as the car arrived. Forward rays cannot see someone approaching
from the kerb.

The revision fixed 33 runs and broke 2 (L00_S06, L06_S01). In both, slowing for a
person on the kerb changed the timing so that they walked into the car's side. The
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

## Requirements

Isaac Sim 6.0.1 (standalone Python), Houdini 21.0.440 for the layout export, and the
district HDA from [procedural-streets-isaac-sim](https://github.com/alirezabidar/procedural-streets-isaac-sim)
checked out next to this repository. The analysis scripts need NumPy, Matplotlib and,
for the layout plots, `usd-core`.

## Pipeline

1. `sim/scenarios.py` writes `configs/benchmark_v1.json` from one master seed
   (2026): layout controls for the HDA and crossing parameters.
2. `houdini/export_layouts.py` (hython) opens the linked HDA scene read-only,
   sets each layout's controls, presses the HDA's own export callback, and
   checks each exported route against the sidewalks before simulation.
3. `sim/benchmark.py` (Isaac Sim Python) builds one stage per layout and runs
   each scenario as stop → reset → play. A kinematic capsule stands in for the
   pedestrian and starts walking when the car is a set distance away.
4. `analysis/summarize.py` turns `results/<sensor>/runs/*.json` into tables and
   figures.

## Run it

```powershell
# 1. layouts (Houdini 21.0.440)
& "C:\Program Files\Side Effects Software\Houdini 21.0.440\bin\hython.exe" houdini\export_layouts.py
# 2. one smoke test (add --sensor wide for the revised controller)
C:\isaacsim\python.bat sim\run.py sim\benchmark.py --layout L00 --scenarios S00
# 3. everything
powershell -ExecutionPolicy Bypass -File run_all.ps1 -Sensor narrow
```

Offline checks that need neither Houdini nor Isaac Sim:
`python -m pytest tests` and `python tests/dry_run.py` (a bicycle-model mock used
only to catch logic errors; its numbers are not results).

## What is measured

| Metric | Definition |
|---|---|
| Outcome | `completed`, `collision_front`, `collision_side`, `kerb_strike`, `off_route` (>2 m from route), `timeout` |
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
