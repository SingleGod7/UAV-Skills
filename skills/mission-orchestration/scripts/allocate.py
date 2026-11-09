#!/usr/bin/env python3
"""Solve UAV ownership and visit order from supplied numerical costs."""
import json
import os
from pathlib import Path
import sys

runtime_root = os.environ.get("MIGRATION_ROOT")
if not runtime_root:
    print(json.dumps({"status": "ENVIRONMENT_ERROR", "message": "未配置 MIGRATION_ROOT"}, ensure_ascii=False))
    raise SystemExit(2)
sys.path.insert(0, str(Path(runtime_root).resolve()))

from uav_runtime.atomic.allocation import compute
from uav_runtime.atomic.common import run


if __name__ == "__main__":
    raise SystemExit(run(compute, "allocation"))
