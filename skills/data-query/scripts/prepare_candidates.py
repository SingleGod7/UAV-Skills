#!/usr/bin/env python3
"""Read eligible fleet records and calculate available numerical budgets."""
import os
from pathlib import Path
import sys

runtime = os.environ.get('MIGRATION_ROOT')
if not runtime:
    raise SystemExit('MIGRATION_ROOT is required')
sys.path.insert(0, str(Path(runtime).resolve()))
from uav_runtime.atomic.common import run
from uav_runtime.seven_inputs import prepare_candidates

if __name__ == '__main__':
    raise SystemExit(run(prepare_candidates, 'candidates'))
