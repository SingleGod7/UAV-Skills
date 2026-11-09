#!/usr/bin/env python3
"""Numerical planning costs or final routes, without solving assignments."""
import os
from pathlib import Path
import sys

runtime = Path(os.environ.get("MIGRATION_ROOT", Path(__file__).resolve().parents[5]))
sys.path.insert(0, str(runtime))
from uav_runtime.atomic.common import run
from uav_runtime.seven_geometry import compute

if __name__ == "__main__":
    raise SystemExit(run(compute, "planning_routes"))
