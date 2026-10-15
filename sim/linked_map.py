"""Read the driving route and the layout values from the city USD exported by the HDA.

The HDA writes the route as /City/DrivingRoute (BasisCurves, purpose guide) and the
layout controls as constant primvars (blocks_x, blocks_z, pitch_x, pitch_z,
road_width, sidewalk).
"""
import numpy as np

KEYS = ('blocks_x', 'blocks_z', 'pitch_x', 'pitch_z', 'road_width', 'sidewalk')


def load_map(filename):
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(str(filename))
    if not stage or UsdGeom.GetStageUpAxis(stage) != 'Y' or UsdGeom.GetStageMetersPerUnit(stage) != 1:
        raise ValueError('The city must be a Y-up, metre-scale USD stage')
    curve = UsdGeom.BasisCurves(stage.GetPrimAtPath('/City/DrivingRoute'))
    if not curve:
        raise ValueError('No /City/DrivingRoute: export the city from the Street District HDA')
    primvars = UsdGeom.PrimvarsAPI(curve)
    controls = {}
    for key in KEYS:
        primvar = primvars.GetPrimvar(key)
        if not primvar or primvar.Get() is None:
            raise ValueError(f'Layout value {key} missing: export the city from the Street District HDA')
        controls[key] = int(primvar.Get()) if key.startswith('blocks') else round(float(primvar.Get()), 4)
    points = np.asarray(curve.GetPointsAttr().Get(), dtype=float)
    if len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('Invalid driving route')
    route = points[:, [0, 2]]
    if np.any(np.linalg.norm(np.diff(route, axis=0), axis=1) < 1e-6):
        raise ValueError('Duplicate route points')
    return controls, route
