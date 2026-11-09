#!/usr/bin/env python3
"""Extract zone counts from solver output and apply them; writes geometry only, never flies."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT))
from uav_runtime.config import load_uavs
from uav_runtime.mission_models import MissionError
from uav_runtime.map_provider import create_map as SemanticMap
from uav_runtime.zone_allocation import apply_allocation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--world', type=Path)
    parser.add_argument('--config')
    parser.add_argument('--result', type=Path,
                        help='Actual solver output or successful central ALLOCATE queue receipt')
    args = parser.parse_args()
    try:
        source = json.loads(args.file.read_text())
        if args.result:
            result = json.loads(args.result.read_text())
            if 'state' in result:
                if result.get('state') != 'SUCCEEDED' or result.get('kind') != 'ALLOCATE':
                    raise ValueError('central allocation task must have succeeded')
                allocation = result['request']
                if allocation['uav_ids'] != source['uav_ids']:
                    raise ValueError('candidate UAV mapping differs from allocation request')
                if 'job_ids' in allocation:
                    tasks = sorted(source['tasks'], key=lambda t:t['region_id'])
                    if ([t['region_id'] for t in tasks] != list(range(1,len(tasks)+1))
                            or [t['task_id'] for t in tasks] != allocation['job_ids']):
                        raise ValueError('task mapping differs from allocation request')
                result = result['result']
            source['result'] = result
        answer = apply_allocation(source, SemanticMap(args.world), load_uavs(args.config))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(answer,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        print(json.dumps(dict(output=str(args.output),submitted=False,
                              zone_alloc_source=answer['zone_alloc_source'])))
    except (MissionError, ValueError, KeyError, TypeError, OSError) as exc:
        print(json.dumps(dict(status='INVALID_ZONE_ALLOCATION',message=str(exc))))
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
