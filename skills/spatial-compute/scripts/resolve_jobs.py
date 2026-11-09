#!/usr/bin/env python3
"""Resolve geographic jobs from supplied records without database reads."""
import os
from pathlib import Path
import sys

runtime = os.environ.get('MIGRATION_ROOT')
if not runtime:
    raise SystemExit('MIGRATION_ROOT is required')
sys.path.insert(0, str(Path(runtime).resolve()))
from uav_runtime.atomic.common import run
from uav_runtime.seven_inputs import resolve_jobs

if __name__ == '__main__':
    raise SystemExit(run(resolve_jobs, 'regions'))
