"""Run every crossing scenario on one exported layout, headless, and log metrics.

Launch through Isaac Sim's Python (see README):

    C:\\isaacsim\\python.bat sim\\run.py sim\\benchmark.py --layout L00

One process per layout: the stage is built once, then each scenario is a
stop -> reset -> play cycle. Results go to results/runs/<layout>_<scenario>.json.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--config', type=Path)
parser.add_argument('--layout', required=True, help='layout id, e.g. L00')
parser.add_argument('--scenarios', nargs='*', help='subset of scenario ids, e.g. S00 S03')
parser.add_argument('--gui', action='store_true', help='open a viewport instead of running headless')
parser.add_argument('--record', action='store_true', help='save chase-camera frames at 10 Hz (slow)')
parser.add_argument('--sensor', choices=['narrow', 'wide'], default='narrow',
                    help='narrow: the original 3 forward rays; wide: car-width forward rays plus a side watch that yields to people stepping out')
args = parser.parse_args()

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sim'))
CONFIG = args.config or ROOT / 'configs' / 'benchmark_v1.json'
OUT = ROOT / 'results' / args.sensor
(OUT / 'runs').mkdir(parents=True, exist_ok=True)
(OUT / 'traces').mkdir(parents=True, exist_ok=True)

from controller import RouteController  # noqa: E402
from road_validation import RoadValidator, vehicle_polygon, separation  # noqa: E402
from scenarios import RAY_OFFSETS, crossing_geometry, pedestrian_position  # noqa: E402
from perception import FAN_ANGLES_DEG, FAN_RANGE_M, SideWatch  # noqa: E402

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({'headless': not args.gui, 'width': 1280, 'height': 720, 'sync_loads': True,
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
    layout = next(l for l in config['layouts'] if l['id'] == args.layout)
    scenarios = [s for s in config['scenarios'] if not args.scenarios or s['id'] in args.scenarios]
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

    # Pedestrian stand-in: a kinematic capsule (0.5 m wide, 1.8 m tall). PhysX moves it
    # to wherever we place it each step; it is solid to the car and visible to ray queries.
    PED_R, PED_H = 0.25, 1.3
    ped = UsdGeom.Capsule.Define(stage, '/World/Pedestrian')
    ped.CreateRadiusAttr(PED_R)
    ped.CreateHeightAttr(PED_H)
    ped.CreateAxisAttr('Y')
    ped_op = ped.AddTranslateOp()
    ped_op.Set(Gf.Vec3d(0, -50, 0))
    UsdPhysics.CollisionAPI.Apply(ped.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(ped.GetPrim()).CreateKinematicEnabledAttr(True)
    add_collision_to_collision_group(stage, ped.GetPath(), '/World/GroundSurfaceCollisionGroup')
    PED_Y = 0.15 + PED_R + PED_H / 2  # standing on a raised kerb height

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
        cam_op = UsdGeom.Xformable(cam).MakeMatrixXform()
        product = rep.create.render_product(str(cam.GetPath()), (1280, 720))
        rgb = rep.AnnotatorRegistry.get_annotator('rgb', device='cpu')
        rgb.attach([product])

    def ped_square(p):
        return np.array([[p[0] - PED_R, p[1] - PED_R], [p[0] + PED_R, p[1] - PED_R],
                         [p[0] + PED_R, p[1] + PED_R], [p[0] - PED_R, p[1] + PED_R]])

    def run(scn):
        geom = crossing_geometry(route, scn, controls['road_width'], controls['sidewalk'])
        controller = RouteController(scn['target_speed_mps'], route)
        L = float(controller.distance[-1])
        sim.stop()
        start = controller.route[0]
        car.GetAttribute('xformOp:translate').Set(Gf.Vec3f(float(start[0]), 1.05, float(start[1])))
        car.GetAttribute('physics:velocity').Set(Gf.Vec3f(0, 0, 0))
        car.GetAttribute('physics:angularVelocity').Set(Gf.Vec3f(0, 0, 0))
        ped_op.Set(Gf.Vec3d(float(geom['start'][0]), PED_Y, float(geom['start'][1])))
        sim.play()
        validator = RoadValidator(controls)
        max_steps = int(PHYS_HZ * (2.5 * L / scn['target_speed_mps'] + 25))
        m = dict(collision=False, min_ped_gap_m=float('inf'), first_detect_m=None, first_detect_t=None,
                 lane_entry_t=None, lane_entry_gap_m=None, lane_entry_speed=None,
                 stopped_for_ped=False, min_gap_while_stopped_m=None, ped_clear_t=None, resume_t=None,
                 max_cte_m=0.0, cte_sq=0.0, n=0)
        walking_t0 = None
        prev_pos = None
        watch = SideWatch(1 / PHYS_HZ)
        states = {}
        held = 0
        trace = []
        outcome = 'timeout'
        wall0 = time.perf_counter()
        for step in range(max_steps):
            t = step / PHYS_HZ
            pos3, fwd3 = car_pose()
            pos = np.array([pos3[0], pos3[2]])
            fwd = np.array([fwd3[0], fwd3[2]])
            fwd = fwd / max(1e-6, np.linalg.norm(fwd))
            # Planar speed from the position change over one physics step.
            speed = 0.0 if step == 0 else float(np.linalg.norm(pos - prev_pos)) * PHYS_HZ
            prev_pos = pos

            # Forward rays at 0.8 m height. 'narrow' = the earlier study's three rays.
            origin = (pos3[0] + fwd3[0] * 2.65, 0.8, pos3[2] + fwd3[2] * 2.65)
            dist, hit_path = 30.0, ''
            for off in RAY_OFFSETS[args.sensor]:
                o = carb.Float3(origin[0] + fwd3[2] * off, origin[1], origin[2] - fwd3[0] * off)
                h = query.raycast_closest(o, carb.Float3(fwd3[0], 0, fwd3[2]), 30.0)
                if h.get('hit') and h['distance'] < dist:
                    dist, hit_path = float(h['distance']), str(h.get('collision', ''))

            if args.sensor == 'wide':
                side_hits = []
                right3 = (-fwd3[2], 0.0, fwd3[0])
                for a in np.radians(FAN_ANGLES_DEG):
                    d = (fwd3[0] * math.cos(a) + right3[0] * math.sin(a), 0.0, fwd3[2] * math.cos(a) + right3[2] * math.sin(a))
                    h = query.raycast_closest(carb.Float3(*origin), carb.Float3(*d), FAN_RANGE_M)
                    if h.get('hit'):
                        side_hits.append((h['distance'] * math.cos(a), h['distance'] * math.sin(a)))
                dist = min(dist, watch.update(side_hits, speed))
                states[watch.state] = states.get(watch.state, 0) + 1
            cmd = controller.update(pos, fwd, speed, dist)
            ctl.GetAcceleratorAttr().Set(cmd['throttle'])
            ctl.GetBrake0Attr().Set(cmd['brake'])
            ctl.GetSteerAttr().Set(cmd['steer'])

            # Pedestrian: starts walking when the car's front is trigger_distance away.
            s_front = cmd['progress'] * L + HALF_LEN
            gap_along = geom['s'] - s_front
            if walking_t0 is None and gap_along <= scn['trigger_distance_m']:
                walking_t0 = t
            pp = pedestrian_position(geom, 0.0 if walking_t0 is None else t - walking_t0, scn['walk_speed_mps'])
            ped_op.Set(Gf.Vec3d(float(pp[0]), PED_Y, float(pp[1])))

            # Metrics
            validator.check(pos, fwd)
            m['max_cte_m'] = max(m['max_cte_m'], cmd['cte'])
            m['cte_sq'] += cmd['cte'] ** 2
            m['n'] += 1
            gap = separation(vehicle_polygon(pos, fwd), ped_square(pp))
            m['min_ped_gap_m'] = min(m['min_ped_gap_m'], gap)
            lateral = abs(float(np.dot(pp - geom['point'], geom['right'])))
            in_path = lateral < HALF_W + PED_R
            if in_path and m['lane_entry_t'] is None:
                m['lane_entry_t'], m['lane_entry_gap_m'], m['lane_entry_speed'] = t, gap_along, speed
            if 'Pedestrian' in hit_path and m['first_detect_m'] is None:
                m['first_detect_m'], m['first_detect_t'] = dist, t
            if speed < 0.15 and walking_t0 is not None and m['ped_clear_t'] is None and gap < 10 and cmd['remaining'] > 3:
                m['stopped_for_ped'] = True
                m['min_gap_while_stopped_m'] = gap if m['min_gap_while_stopped_m'] is None else min(m['min_gap_while_stopped_m'], gap)
            if m['lane_entry_t'] is not None and m['ped_clear_t'] is None and not in_path:
                m['ped_clear_t'] = t
            if m['ped_clear_t'] is not None and m['resume_t'] is None and speed > 0.5 and m['stopped_for_ped']:
                m['resume_t'] = t

            if step % 12 == 0:
                trace.append(dict(t=round(t, 3), x=float(pos[0]), z=float(pos[1]), speed=speed, cte=cmd['cte'],
                                  ray=dist, hit=hit_path.rsplit('/', 1)[-1], ped_x=float(pp[0]), ped_z=float(pp[1]),
                                  ped_gap=gap, throttle=cmd['throttle'], brake=cmd['brake'], steer=cmd['steer']))
                if args.record:
                    eye = Gf.Vec3d(pos3[0] - fwd3[0] * 12, 9, pos3[2] - fwd3[2] * 12)
                    cam_op.Set(Gf.Matrix4d().SetLookAt(eye, Gf.Vec3d(pos3[0], 1, pos3[2]), Gf.Vec3d(0, 1, 0)).GetInverse())
                    rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
                    frames = OUT / 'frames' / f"{args.layout}_{scn['id']}"
                    frames.mkdir(parents=True, exist_ok=True)
                    Image.fromarray(np.asarray(rgb.get_data())[:, :, :3]).save(frames / f'{len(trace) - 1:05d}.png')

            sim.step(render=args.gui)
            if step % (PHYS_HZ * 10) == 0:
                print(f'  t={t:5.1f}s  progress={cmd["progress"]:.2f}  speed={speed:.2f}  '
                      f'wall={time.perf_counter() - wall0:.0f}s', flush=True)

            if gap < 0:
                # Front contact: the car drove into the pedestrian. Side contact: the
                # pedestrian walked into a car that was already passing (forward
                # sensing cannot see this coming). Both count as failures.
                lon = float(np.dot(pp - pos, fwd))
                m['collision'] = True
                m['contact_zone'] = 'front' if lon > HALF_LEN - 0.5 else 'side'
                outcome = 'collision_' + m['contact_zone']
                break
            if cmd['cte'] > 2.0:
                outcome = 'off_route'
                break
            held = held + 1 if (cmd['remaining'] < 1.2 and speed < 0.15) else 0
            if held >= 2 * PHYS_HZ:
                outcome = 'completed'
                break
        wall = time.perf_counter() - wall0
        sim_t = (step + 1) / PHYS_HZ
        road = validator.report()
        if outcome == 'completed' and road['sidewalk_overlap_samples'] > 0:
            outcome = 'kerb_strike'
        result = dict(
            layout=args.layout, scenario=scn['id'], scenario_params=scn, controls=controls,
            outcome=outcome, success=outcome == 'completed',
            route_length_m=L, progress=cmd['progress'], sim_time_s=sim_t, wall_time_s=wall,
            real_time_factor=sim_t / wall if wall > 0 else None,
            rms_cte_m=math.sqrt(m['cte_sq'] / max(1, m['n'])), max_cte_m=m['max_cte_m'],
            crossing_s_m=geom['s'], pedestrian_started=walking_t0 is not None,
            collision=m['collision'], contact_zone=m.get('contact_zone'), min_ped_gap_m=m['min_ped_gap_m'],
            lane_entry_t=m['lane_entry_t'], lane_entry_gap_m=m['lane_entry_gap_m'], lane_entry_speed=m['lane_entry_speed'],
            first_detect_t=m['first_detect_t'], first_detect_m=m['first_detect_m'],
            stopped_for_ped=m['stopped_for_ped'], min_gap_while_stopped_m=m['min_gap_while_stopped_m'],
            resume_delay_s=(m['resume_t'] - m['ped_clear_t']) if m['resume_t'] and m['ped_clear_t'] else None,
            road_check=road, side_watch_steps=states or None,
            method=dict(physics_hz=PHYS_HZ, pose='simulator ground truth', sensing=f'{len(RAY_OFFSETS[args.sensor])} PhysX ray queries at lateral offsets {RAY_OFFSETS[args.sensor]} m', sensor=args.sensor,
                        pedestrian='kinematic capsule r=0.25 m, straight-line walk', vehicle='NVIDIA BasicSetup PhysX vehicle'),
        )
        name = f"{args.layout}_{scn['id']}"
        (OUT / 'runs' / f'{name}.json').write_text(json.dumps(result, indent=2))
        (OUT / 'traces' / f'{name}.json').write_text(json.dumps(trace))
        print('RUN', name, outcome, f"gap={m['min_ped_gap_m']:.2f}", f'rtf={result["real_time_factor"]:.2f}', flush=True)
        return result

    for scn in scenarios:
        run(scn)
    sim.stop()
    print('LAYOUT_DONE', args.layout, flush=True)
except Exception:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
