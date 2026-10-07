"""Run the experiment through Isaac Sim's configured Python environment."""
import os
import runpy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
os.environ["WARP_CACHE_PATH"] = str(ROOT / "cache/warp")
# USD scans PATH for DLL directories; Windows app aliases are unrelated to this task.
os.environ["PATH"] = os.pathsep.join(p for p in os.environ["PATH"].split(os.pathsep) if "WindowsApps" not in p)
script = sys.argv.pop(1)
runpy.run_path(script, run_name="__main__")
