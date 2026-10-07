"""Export every benchmark layout from the linked Street District HDA.

Run with Houdini's hython (the HIP file is opened read-only and never saved):

    hython houdini/export_layouts.py --config configs/benchmark_v1.json

For each layout this sets the HDA controls, presses the HDA's own
"Export City + Driving Route" callback, then checks the exported route
geometrically before any simulation runs.
"""
import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

os.environ['PATH'] = os.pathsep.join(p for p in os.environ['PATH'].split(os.pathsep) if 'WindowsApps' not in p)
import hou  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sim'))
from linked_map import load_map  # noqa: E402
from road_validation import RoadValidator  # noqa: E402
from scenarios import route_arclength  # noqa: E402

# The linked district scene comes from the procedural-streets-isaac-sim repository
# (houdini/link_routes.py). Pass --hip if it lives somewhere else.
DEFAULT_HIP = ROOT.parent / 'procedural-streets-isaac-sim' / 'houdini' / 'procedural_driving_linked.hip'
HDA_NODE = '/obj/procedural_street_district/street_district'

parser = argparse.ArgumentParser()
parser.add_argument('--config', type=Path, default=ROOT / 'configs' / 'benchmark_v1.json')
parser.add_argument('--hip', type=Path, default=DEFAULT_HIP)
parser.add_argument('--only', nargs='*', help='layout ids to export, e.g. L00 L03')
args = parser.parse_args()

config = json.loads(args.config.read_text())
hou.hipFile.load(str(args.hip), suppress_save_prompt=True, ignore_load_warnings=True)
hda = hou.node(HDA_NODE)
if hda is None:
    raise SystemExit(f'HDA node not found at {HDA_NODE}')

# Count what is actually inside the HDA, so the website number is checked, not remembered.
sops = [n for n in hda.allSubChildren(recurse_in_locked_nodes=True) if n.type().category().name() == 'Sop']
type_names = [n.type().name() for n in sops]
node_report = dict(
    hda_type=hda.type().name(),
    houdini=hou.applicationVersionString(),
    sop_nodes=len(sops),
    python_sops=sum(t == 'python' for t in type_names),
    wrangles=sum('wrangle' in t for t in type_names),
)
print('HDA', json.dumps(node_report), flush=True)

keys = ('blocks_x', 'blocks_z', 'pitch_x', 'pitch_z', 'road_width', 'sidewalk', 'seed')
defaults = {k: hda.evalParm(k) for k in keys}
rows = []
for layout in config['layouts']:
    if args.only and layout['id'] not in args.only:
        continue
    hda.setParms(defaults)
    hda.setParms(layout['controls'])
    dest = ROOT / 'layouts' / layout['id'] / 'city.usdc'
    dest.parent.mkdir(parents=True, exist_ok=True)
    hda.parm('usd_path').set(str(dest))
    hda.hdaModule().export_linked(hda)  # same callback as the HDA button

    controls, route = load_map(dest)
    validator = RoadValidator(controls)
    for i, p in enumerate(route):
        d = route[min(i + 1, len(route) - 1)] - route[max(0, i - 1)]
        validator.check(p, d / np.linalg.norm(d))
    check = validator.report()
    start_dir = route[5] - route[0]
    start_dir = start_dir / np.linalg.norm(start_dir)
    row = dict(
        id=layout['id'], controls=layout['controls'], file=str(dest.relative_to(ROOT)),
        sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),
        route_points=len(route), route_length_m=float(route_arclength(route)[-1]),
        start=route[0].tolist(), start_heading_xz=start_dir.tolist(),
        route_check=check,
        route_check_passed=check['sidewalk_overlap_samples'] == 0 and check['wrong_lane_samples'] == 0,
    )
    rows.append(row)
    print('LAYOUT', layout['id'], round(row['route_length_m'], 2), 'ok' if row['route_check_passed'] else 'FAILED', flush=True)

manifest = dict(config=config['name'], master_seed=config['master_seed'], hda=node_report, layouts=rows)
(ROOT / 'layouts' / 'manifest.json').write_text(json.dumps(manifest, indent=2))
print('EXPORTED', len(rows), 'layouts', flush=True)
