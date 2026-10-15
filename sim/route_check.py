"""Car-only route check in Isaac Sim: no pedestrian, just the car and the curbs.

One process per layout. The car drives the exported route at each benchmark
speed (3.0, 3.5, 4.0 m/s) with no pedestrian, and every physics step checks the
2.05 x 4.90 m car body against the raised sidewalks (RoadValidator).

    C:\\isaacsim\\python.bat sim\\run.py sim\\route_check.py --layout L00
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--layout', required=True)
parser.add_argument('--speeds', nargs='*', type=float, default=[3.0, 3.5, 4.0])
args = parser.parse_args()
args.gui = args.record = False

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sim'))
CONFIG = ROOT / 'configs' / 'benchmark_v1.json'
OUT = ROOT / 'results' / 'route_check'
(OUT / 'runs').mkdir(parents=True, exist_ok=True)

from controller import RouteController  # noqa: E402
from road_validation import RoadValidator  # noqa: E402
from scenarios import RAY_OFFSETS  # noqa: E402
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({'headless': True, 'width': 1280, 'height': 720, 'sync_loads': True,
                     'extra_args': ['--portable', '--portable-root', str(ROOT / 'work' / 'kit')]})
try:
    import numpy as np
    import carb
    import omni.usd
    import omni.physx
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf
    from linked_map import load_map
    from isaacsim.core.utils.extensions import enable_extension

    config = json.loads(CONFIG.read_text())
    city_file = ROOT / 'layouts' / args.layout / 'city.usdc'
    controls, route = load_map(city_file)

    # ------------------------------------------------------------ scene
    enable_extension('omni.physx.vehicle')
    for _ in range(4):
        app.update()
    from omni.physxvehicle.scripts.samples import BasicSetup
    from omni.physx.scripts.physicsUtils import add_collision_to_collision_group, add_physics_material_to_prim

    stage = omni.usd.get_context().get_stage()
    world = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(world.GetPrim())
    paths = BasicSetup.create(stage, False)  # NVIDIA's PhysX vehicle sample
    stage.RemovePrim('/World/GroundPlane')
    district = stage.DefinePrim('/World/District', 'Xform')
    district.GetReferences().AddReference(str(city_file))
    for p in Usd.PrimRange(district):
        if p.IsA(UsdGeom.Mesh) and p.GetName() in ['Roads', 'Sidewalks', 'Buildings', 'Foundation', 'Plots']:
            UsdPhysics.CollisionAPI.Apply(p)
            add_collision_to_collision_group(stage, p.GetPath(), '/World/GroundSurfaceCollisionGroup')
            add_physics_material_to_prim(stage, p, Sdf.Path('/World/TarmacMaterial'))


    from isaacsim.core.api import SimulationContext
    sim = SimulationContext(physics_dt=1 / 120, rendering_dt=1 / 30, stage_units_in_meters=1,
                            physics_prim_path='/World/PhysicsScene', set_defaults=False)
    # The district is Y-up: restore Y-up and -Y gravity after the context is created.
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    scene = UsdPhysics.Scene(stage.GetPrimAtPath('/World/PhysicsScene'))
    scene.GetGravityDirectionAttr().Set(Gf.Vec3f(0, -1, 0))
    scene.GetGravityMagnitudeAttr().Set(9.81)
    sim.initialize_physics()

    car = stage.GetPrimAtPath(paths.vehiclePath)
    physx = omni.physx.get_physx_interface()
    FAST = not (args.gui or args.record)
    if FAST:
        # Read the car pose straight from PhysX instead of syncing every body to USD each step.
        settings = carb.settings.get_settings()
        settings.set_bool('/physics/updateToUsd', False)
        settings.set_bool('/physics/updateVelocitiesToUsd', False)

    def car_pose():
        if FAST:
            tr = physx.get_rigidbody_transformation(str(car.GetPath()))
            p, q = tr['position'], tr['rotation']  # q = (x, y, z, w)
            rot = Gf.Rotation(Gf.Quatd(float(q[3]), Gf.Vec3d(float(q[0]), float(q[1]), float(q[2]))))
            return Gf.Vec3d(*p), rot.TransformDir(Gf.Vec3d(0, 0, 1))
        mat = UsdGeom.Xformable(car).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return mat.ExtractTranslation(), mat.TransformDir(Gf.Vec3d(0, 0, 1))
    ctl = PhysxSchema.PhysxVehicleControllerAPI(car)
    query = omni.physx.get_physx_scene_query_interface()
    HALF_LEN, HALF_W = 2.45, 1.025
    PHYS_HZ = 120

    if args.record:
        import omni.replicator.core as rep
        from PIL import Image
        cam = UsdGeom.Camera.Define(stage, '/World/ChaseCam')
        cam.CreateFocalLengthAttr(24)

    def run(speed_target):
        controller = RouteController(speed_target, route)
        L = float(controller.distance[-1])
        sim.stop()
        start = controller.route[0]
        car.GetAttribute('xformOp:translate').Set(Gf.Vec3f(float(start[0]), 1.05, float(start[1])))
        car.GetAttribute('physics:velocity').Set(Gf.Vec3f(0, 0, 0))
        car.GetAttribute('physics:angularVelocity').Set(Gf.Vec3f(0, 0, 0))
        sim.play()
        validator = RoadValidator(controls)
        max_steps = int(PHYS_HZ * (2.5 * L / speed_target + 25))
        max_cte, cte_sq, n, held, prev_pos = 0.0, 0.0, 0, 0, None
        outcome = 'timeout'
        path = []
        wall0 = time.perf_counter()
        for step in range(max_steps):
            pos3, fwd3 = car_pose()
            pos = np.array([pos3[0], pos3[2]])
            fwd = np.array([fwd3[0], fwd3[2]])
            fwd = fwd / max(1e-6, np.linalg.norm(fwd))
            speed = 0.0 if step == 0 else float(np.linalg.norm(pos - prev_pos)) * PHYS_HZ
            prev_pos = pos
            origin = (pos3[0] + fwd3[0] * 2.65, 0.8, pos3[2] + fwd3[2] * 2.65)
            dist = 30.0
            for off in RAY_OFFSETS['narrow']:
                o = carb.Float3(origin[0] + fwd3[2] * off, origin[1], origin[2] - fwd3[0] * off)
                h = query.raycast_closest(o, carb.Float3(fwd3[0], 0, fwd3[2]), 30.0)
                if h.get('hit') and h['distance'] < dist:
                    dist = float(h['distance'])
            cmd = controller.update(pos, fwd, speed, dist)
            ctl.GetAcceleratorAttr().Set(cmd['throttle'])
            ctl.GetBrake0Attr().Set(cmd['brake'])
            ctl.GetSteerAttr().Set(cmd['steer'])
            validator.check(pos, fwd)
            max_cte = max(max_cte, cmd['cte']); cte_sq += cmd['cte'] ** 2; n += 1
            if step % 24 == 0:
                path.append([round(float(pos[0]), 3), round(float(pos[1]), 3)])
            sim.step(render=False)
            if cmd['cte'] > 2.0:
                outcome = 'off_route'
                break
            held = held + 1 if (cmd['remaining'] < 1.2 and speed < 0.15) else 0
            if held >= 2 * PHYS_HZ:
                outcome = 'completed'
                break
        road = validator.report()
        if outcome == 'completed' and road['sidewalk_overlap_samples'] > 0:
            outcome = 'curb_strike'
        result = dict(layout=args.layout, target_speed_mps=speed_target,
                      outcome=outcome, success=outcome == 'completed', route_length_m=L, route_points=len(route),
                      progress=cmd['progress'], sim_time_s=(step + 1) / PHYS_HZ, wall_time_s=time.perf_counter() - wall0,
                      rms_cte_m=math.sqrt(cte_sq / max(1, n)), max_cte_m=max_cte, road_check=road, path_xz_5hz=path)
        name = f"{args.layout}_v{speed_target:.1f}"
        (OUT / 'runs' / f'{name}.json').write_text(json.dumps(result, indent=2))
        print('RUN', name, outcome, 'overlaps', road['sidewalk_overlap_samples'], 'wrong_lane', road['wrong_lane_samples'],
              f"min_gap={road['minimum_sidewalk_separating_gap_m']:.3f}", f'max_cte={max_cte:.3f}', flush=True)

    for s in args.speeds:
        run(s)
    sim.stop()
    print('LAYOUT_DONE', args.layout, flush=True)
except Exception:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
