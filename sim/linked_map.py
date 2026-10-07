"""Read the route and layout from the same USD layer as the city."""
import json
import numpy as np

def load_map(filename):
    from pxr import Usd, UsdGeom
    stage=Usd.Stage.Open(str(filename))
    if not stage or UsdGeom.GetStageUpAxis(stage)!='Y' or UsdGeom.GetStageMetersPerUnit(stage)!=1:
        raise ValueError('Linked district must be a Y-up, metre-scale USD stage')
    root=stage.GetDefaultPrim()
    if root.GetAttribute('driving:schemaVersion').Get()!=1:
        raise ValueError('Export the district using Export City + Driving Route first')
    controls=json.loads(root.GetAttribute('driving:layout').Get())
    curve=UsdGeom.BasisCurves(stage.GetPrimAtPath(str(root.GetPath())+'/DrivingRoute'))
    points=np.asarray(curve.GetPointsAttr().Get(),dtype=float)
    if len(points)<2 or not np.isfinite(points).all():raise ValueError('Invalid driving route')
    route=points[:,[0,2]]
    if np.any(np.linalg.norm(np.diff(route,axis=0),axis=1)<1e-6):raise ValueError('Duplicate route points')
    return controls,route
