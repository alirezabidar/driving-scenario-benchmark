"""Top-down plan of every exported layout with its driving route.

    python analysis/plot_layouts.py   ->  results/layouts-grid.png
Needs usd-core (pip install usd-core) or Isaac Sim's Python.
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sim'))
from linked_map import load_map  # noqa: E402

STYLE = {'Roads': '#2a3238', 'Sidewalks': '#56626b', 'Buildings': '#b98d55', 'Plots': '#36423a', 'Trees': '#4f7d55'}


def faces(mesh):
    pts = np.asarray(mesh.GetPointsAttr().Get())[:, [0, 2]]
    counts = mesh.GetFaceVertexCountsAttr().Get()
    idx = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
    out, i = [], 0
    for c in counts:
        out.append(pts[idx[i:i + c]])
        i += c
    return out


def draw(ax, city):
    stage = Usd.Stage.Open(str(city))
    for name in ['Plots', 'Roads', 'Sidewalks', 'Trees', 'Buildings']:
        prim = stage.GetPrimAtPath(f'/City/{name}')
        if prim and prim.IsA(UsdGeom.Mesh):
            ax.add_collection(PolyCollection(faces(UsdGeom.Mesh(prim)), facecolor=STYLE[name], edgecolor='none'))
    controls, route = load_map(city)
    ax.plot(route[:, 0], route[:, 1], color='#79d1dc', lw=1.6)
    ax.plot(*route[0], 'o', color='#79d1dc', ms=4)
    ax.set_aspect('equal')
    ax.autoscale_view()
    ax.invert_yaxis()
    ax.axis('off')
    return controls


def main():
    manifest = json.loads((ROOT / 'layouts' / 'manifest.json').read_text())
    rows = manifest['layouts']
    fig, axes = plt.subplots(2, 5, figsize=(15, 7), facecolor='#11181e')
    for ax, row in zip(axes.flat, rows):
        c = draw(ax, ROOT / row['file'].replace('\\', '/'))
        ax.set_title(f"{row['id']}  ·  {c['blocks_x']}×{c['blocks_z']} blocks  ·  road {c['road_width']:g} m\n"
                     f"route {row['route_length_m']:.0f} m", color='#c3cbd0', fontsize=9)
    fig.tight_layout()
    out = ROOT / 'results' / 'layouts-grid.png'
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=110, facecolor=fig.get_facecolor())
    print('wrote', out)


if __name__ == '__main__':
    main()
