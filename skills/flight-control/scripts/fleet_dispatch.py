#!/usr/bin/env python3
"""Validate and dispatch one UAV or a concurrent group of independent SITL tasks."""

import argparse
import fcntl
import json
import math
import subprocess
import sys
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import rospy
import rosnode
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT.parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from uav_runtime.config import load_uavs, runtime_root
FLEET_FILE = ROOT / "config/uavs.json"
SUBMIT_SCRIPT = Path(__file__).with_name("submit_queue_batch.py")
LOCK_FILE = runtime_root() / "fleet_dispatch.lock"
RESERVATIONS_FILE = runtime_root() / "fleet_dispatch.json"
TERMINAL = {"SUCCEEDED", "FAILED", "CANCELED"}
MIN_SEPARATION_M = 2.0


def wait_message(topic, message_type, timeout=2.0):
    try:
        return rospy.wait_for_message(topic, message_type, timeout=timeout)
    except rospy.ROSException:
        return None


def live_vehicle(vehicle, config, nodes):
    ns = "/" + config["namespace"]
    controller = ns + "/drone_control"
    if controller not in nodes or ns + "/mavros" not in nodes:
        return None
    if rospy.get_param(controller + "/runtime_profile", None) != "sim":
        return None
    state = wait_message(ns + "/mavros/state", State)
    if state is None or not state.connected:
        return None
    pose = wait_message(ns + "/mavros/local_position/pose", PoseStamped)
    status_message = wait_message(controller + "/exec_status", String, timeout=0.5)
    try:
        status = json.loads(status_message.data) if status_message else {}
    except (ValueError, TypeError):
        status = {}
    world_xy = ([config["spawn_xy"][0] + pose.pose.position.x,
                 config["spawn_xy"][1] + pose.pose.position.y] if pose else None)
    return {"uav_id": vehicle, "namespace": ns, "armed": bool(state.armed),
            "world_xy": world_xy,
            "mode": state.mode, "status": status}


def load_state(run_id):
    try:
        saved = json.loads(RESERVATIONS_FILE.read_text(encoding="utf-8"))
        if saved.get("run_id") == run_id:
            return saved
    except (OSError, ValueError):
        pass
    return {"run_id": run_id, "reservations": {}, "last_used": {}}


def release_completed(saved, live):
    for vehicle, reservation in list(saved["reservations"].items()):
        current = live.get(vehicle)
        if current is None or current["armed"]:
            continue
        status = current["status"]
        old_session = reservation.get("executor_session_id")
        new_session = status.get("executor_session_id")
        if old_session and new_session and old_session != new_session:
            del saved["reservations"][vehicle]
            continue
        if (status.get("task_id") == reservation["task_id"]
                and (status.get("task_state") in ("SUCCEEDED", "FAILED", "CANCELED", "PARTIAL")
                     or ("task_state" not in status and status.get("state") in ("FAILED", "CANCELED"))
                     or ("task_state" not in status and status.get("step_id") == reservation["last_step_id"]
                         and status.get("state") == "SUCCEEDED"))):
            del saved["reservations"][vehicle]


def planned_xy(validated_batch, config, current_xy=None):
    spawn_x, spawn_y = config["spawn_xy"]
    points = [list(current_xy) if current_xy is not None else [float(spawn_x), float(spawn_y)]]
    for action in validated_batch:
        for waypoint in action["params"].get("waypoints", []):
            xy = [waypoint[axis] for axis in ("x", "y")] if isinstance(waypoint, dict) else waypoint[:2]
            points.append([spawn_x + xy[0], spawn_y + xy[1]])
    return points


def remaining_path(reservation, current_xy):
    """Advance only along a segment supported by fresh measured position.

    Retain the old path if the vehicle has detoured far from it.  Search boxes
    remain reserved independently; their start point can follow live pose.
    """
    path = reservation.get("path_xy") or []
    if not path:
        return None
    if reservation.get("repeating"):
        return path
    if reservation.get("regions_xy") and len(path) == 1:
        return [list(current_xy)]
    if len(path) == 1:
        return [list(current_xy)]
    nearest = None
    for index, (start, end) in enumerate(zip(path, path[1:])):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_sq = dx * dx + dy * dy
        fraction = max(0.0, min(1.0, ((current_xy[0] - start[0]) * dx +
                                        (current_xy[1] - start[1]) * dy) / length_sq)) if length_sq else 0.0
        projection = [start[0] + fraction * dx, start[1] + fraction * dy]
        distance = math.hypot(current_xy[0] - projection[0], current_xy[1] - projection[1])
        if nearest is None or distance < nearest[0]:
            nearest = (distance, index)
    if nearest[0] > 1.0:
        return path
    return [list(current_xy)] + path[nearest[1] + 1:]


def planned_regions(validated_batch, config):
    """Reserve the whole exploration footprint, not only a nominal route."""
    spawn_x, spawn_y = config["spawn_xy"]
    return [[spawn_x + action["params"]["box_min"][0],
             spawn_y + action["params"]["box_min"][1],
             spawn_x + action["params"]["box_max"][0],
             spawn_y + action["params"]["box_max"][1]]
            for action in validated_batch if action["cmd"] == "fuel_explore"]


def region_conflicts_path(region, path):
    x0, y0, x1, y1 = region
    if any(x0 - MIN_SEPARATION_M < point[0] < x1 + MIN_SEPARATION_M and
           y0 - MIN_SEPARATION_M < point[1] < y1 + MIN_SEPARATION_M for point in path):
        return True
    outline = [[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]
    return paths_too_close(path, outline)


def regions_conflict(first, second):
    dx = max(first[0] - second[2], second[0] - first[2], 0.0)
    dy = max(first[1] - second[3], second[1] - first[3], 0.0)
    return math.hypot(dx, dy) < MIN_SEPARATION_M


def point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = max(0.0, min(1.0, ((point[0] - start[0]) * dx +
                                   (point[1] - start[1]) * dy) / length_sq))
    return math.hypot(point[0] - start[0] - fraction * dx,
                      point[1] - start[1] - fraction * dy)


def paths_too_close(first, second):
    # Consecutive point pairs are the conservative swept paths of both UAVs.
    first_segments = list(zip(first, first[1:])) or [(first[0], first[0])]
    second_segments = list(zip(second, second[1:])) or [(second[0], second[0])]
    for a, b in first_segments:
        for c, d in second_segments:
            cross = lambda u, v, w: ((v[0] - u[0]) * (w[1] - u[1]) -
                                     (v[1] - u[1]) * (w[0] - u[0]))
            boxes_overlap = (max(min(a[0], b[0]), min(c[0], d[0])) <=
                             min(max(a[0], b[0]), max(c[0], d[0])) and
                             max(min(a[1], b[1]), min(c[1], d[1])) <=
                             min(max(a[1], b[1]), max(c[1], d[1])))
            if (boxes_overlap and cross(a, b, c) * cross(a, b, d) <= 0
                    and cross(c, d, a) * cross(c, d, b) <= 0):
                return True
            distance = min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
                           point_segment_distance(c, a, b), point_segment_distance(d, a, b))
            if distance < MIN_SEPARATION_M:
                return True
    return False


def prepare_group(requests, fleet):
    """Validate every original WGS84 plan before any publisher is created."""
    from submit_queue_batch import validate_batch, conversion_summary, ValidationError
    if not isinstance(requests, dict) or len(requests) < 2:
        raise ValueError('--fleet-json requires at least two UAV ID -> action array entries')
    plans = {}
    for vehicle, batch in requests.items():
        if vehicle not in fleet:
            raise ValueError('Unknown UAV ID: ' + vehicle)
        config = fleet[vehicle]
        try:
            validated = validate_batch(batch, config['origin_lat'], config['origin_lon'])
        except ValidationError as exc:
            raise ValueError(vehicle + ': ' + str(exc)) from None
        plans[vehicle] = {'batch': validated, 'task_id': validated[0]['task_id'],
                          'last_step_id': validated[-1]['step_id'],
                          'conversion': conversion_summary(batch, config['origin_lat'], config['origin_lon'], 10000.)}
    return plans


def collect_live(fleet, nodes):
    # Each vehicle's telemetry is independent; do not accumulate per-UAV waits.
    with ThreadPoolExecutor(max_workers=min(16, len(fleet))) as pool:
        futures = {vehicle: pool.submit(live_vehicle, vehicle, config, nodes)
                   for vehicle, config in fleet.items()}
        return {vehicle: value for vehicle, future in futures.items()
                for value in [future.result()] if value is not None}


def group_reservations(plans, fleet, live, saved, enforce_separation=False):
    """Preflight the whole group; do not mutate persisted reservations on failure."""
    reservations = dict(saved['reservations'])
    additions = {}
    for vehicle, plan in plans.items():
        current = live.get(vehicle)
        if (current is None or current['armed'] or current['world_xy'] is None or vehicle in reservations
                or current['status'].get('task_state', current['status'].get('state')) in
                ('RUNNING','PENDING','PAUSED','QUEUED','ACCEPTED')):
            raise ValueError('UAV is not online, idle and unreserved: ' + vehicle)
        if not current['status'].get('executor_session_id'):
            raise ValueError('No fresh executor session status for ' + vehicle)
        if current['status'].get('task_id') == plan['task_id']:
            raise ValueError('task_id was already used by ' + vehicle + '; use a new task ID')
        path = planned_xy(plan['batch'], fleet[vehicle], current['world_xy'])
        regions = planned_regions(plan['batch'], fleet[vehicle])
        if enforce_separation:
            for other, reservation in reservations.items():
                other_path = reservation.get('path_xy')
                if other not in live or not other_path:
                    raise ValueError('Reserved UAV has no verified live path: ' + other)
                other_regions = reservation.get('regions_xy', [])
                if (paths_too_close(path, other_path)
                        or any(region_conflicts_path(r, other_path) for r in regions)
                        or any(region_conflicts_path(r, path) for r in other_regions)
                        or any(regions_conflict(a,b) for a in regions for b in other_regions)):
                    raise ValueError('Planned group conflicts with reserved path or region: ' + other)
            for other, other_current in live.items():
                if other == vehicle:
                    continue
                if (paths_too_close(path, [other_current['world_xy']])
                        or any(region_conflicts_path(r, [other_current['world_xy']]) for r in regions)):
                    raise ValueError('Planned group conflicts with current UAV position: ' + other)
                if other_current['armed'] and other not in reservations:
                    raise ValueError('Airborne UAV has no registered path: ' + other)
        reservation = {'task_id': plan['task_id'], 'last_step_id': plan['last_step_id'],
                       'executor_session_id': current['status'].get('executor_session_id'),
                       'path_xy': path, 'regions_xy': regions,
                       'repeating': any(a['cmd'] in ('follow_path','patrol_move') and
                                        (a['params'].get('loop_forever') or a['params'].get('repeat',1)>1)
                                        for a in plan['batch'])}
        reservations[vehicle] = reservation
        additions[vehicle] = reservation
    members = {uid: {k:item[k] for k in ('task_id','executor_session_id')} for uid,item in additions.items()}
    for item in additions.values():
        item['group_members'] = members
    return additions


def publish_group(plans, publishers, timeout=5.):
    """Wait for every transport, then release the per-UAV publishes together."""
    deadline = time.monotonic() + timeout
    while not all(pub.get_num_connections() > 0 for pub in publishers.values()):
        if rospy.is_shutdown() or time.monotonic() >= deadline:
            raise RuntimeError('Not all UAV queues connected; no group batch was published')
        time.sleep(.02)
    gate = threading.Barrier(len(plans))
    def send(vehicle):
        gate.wait(timeout=3.)
        started = time.time()
        try:
            publishers[vehicle].publish(String(data=json.dumps(plans[vehicle]['batch'],ensure_ascii=False)))
            return {'submitted': True, 'stage': 'PUBLISHED', 'published_at': started,
                    'command_accepted': None, 'execution_state': 'UNKNOWN'}
        except Exception as exc:
            return {'submitted': None, 'stage': 'PUBLISH_UNCERTAIN', 'published_at': started,
                    'error': str(exc)[:300], 'command_accepted': None, 'execution_state': 'UNKNOWN'}
    with ThreadPoolExecutor(max_workers=len(plans)) as pool:
        futures = {vehicle: pool.submit(send,vehicle) for vehicle in plans}
        return {vehicle: future.result() for vehicle,future in futures.items()}


def dispatch_group(requests, submit=False, enforce_separation=False):
    fleet = load_uavs()
    plans = prepare_group(requests, fleet)
    rospy.init_node('fleet_dispatch_group', anonymous=True, disable_signals=True)
    run_id = rospy.get_param('/run_id')
    LOCK_FILE.parent.mkdir(parents=True,exist_ok=True)
    publishers, subscribers = {}, []
    acknowledgments = {}
    ack_condition = threading.Condition()
    def callback(vehicle, message):
        try:
            value = json.loads(message.data)
        except (ValueError,TypeError):
            return
        if (value.get('task_id') != plans[vehicle]['task_id']
                or value.get('executor_session_id') != live[vehicle]['status'].get('executor_session_id')
                or value.get('state') not in ('ACCEPTED','REJECTED')):
            return
        with ack_condition:
            acknowledgments[vehicle] = value['state']
            ack_condition.notify_all()
    try:
        with LOCK_FILE.open('a+') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            saved = load_state(run_id)
            live = collect_live(fleet,set(rosnode.get_node_names()))
            release_completed(saved,live)
            if any(c['world_xy'] is None for c in live.values()):
                raise ValueError('Connected UAV has no live pose')
            for vehicle,reservation in saved['reservations'].items():
                if vehicle in live:
                    updated = remaining_path(reservation,live[vehicle]['world_xy'])
                    if updated is not None:reservation['path_xy'] = updated
            additions = group_reservations(plans,fleet,live,saved,enforce_separation)
            if not submit:
                return {'status':'OK','submitted':False,'stage':'VALIDATED','mode':'concurrent',
                        'vehicles':plans}
            # Verify the expected controller subscribers before creating any command traffic.
            code, message, system = rospy.get_master().getSystemState()
            if code != 1:raise RuntimeError('Cannot verify ROS subscribers')
            subscriptions = dict(system[1])
            for vehicle in plans:
                ns = '/' + fleet[vehicle]['namespace']
                topic = ns+'/drone_control/queue_batch'
                if ns+'/drone_control' not in subscriptions.get(topic,[]):
                    raise RuntimeError('Queue has no matching controller: ' + vehicle)
                subscribers.append(rospy.Subscriber(ns+'/drone_control/command_ack',String,
                                   lambda msg,v=vehicle:callback(v,msg),queue_size=10))
                publishers[vehicle] = rospy.Publisher(topic,String,queue_size=1,latch=False)
            deadline = time.monotonic()+5.
            while not all(p.get_num_connections() for p in publishers.values()):
                if rospy.is_shutdown() or time.monotonic()>=deadline:
                    raise RuntimeError('Not all UAV queues connected; no batch published')
                time.sleep(.02)
            # Recheck after connection setup so stale preflight state cannot start a busy UAV.
            refreshed = collect_live({v:fleet[v] for v in plans},set(rosnode.get_node_names()))
            for vehicle in plans:
                if vehicle not in refreshed:
                    raise ValueError('UAV disconnected before group release: ' + vehicle)
                live[vehicle] = refreshed[vehicle]
            additions = group_reservations(plans,fleet,live,saved,enforce_separation)
            # Reserve all members once; independent dispatchers cannot claim a group member.
            saved['reservations'].update(additions)
            saved['last_used'].update({v:time.time() for v in plans})
            RESERVATIONS_FILE.write_text(json.dumps(saved,ensure_ascii=False,indent=2),encoding='utf-8')
            results = publish_group(plans,publishers)
        # A bounded receipt check only: never wait for takeoff or mission completion.
        deadline=time.monotonic()+5.
        with ack_condition:
            while len(acknowledgments)<len(plans) and time.monotonic()<deadline and not rospy.is_shutdown():
                ack_condition.wait(max(0.,deadline-time.monotonic()))
        for vehicle,result in results.items():
            result['command_ack']=acknowledgments.get(vehicle,'UNKNOWN')
            result['command_accepted']={'ACCEPTED':True,'REJECTED':False}.get(acknowledgments.get(vehicle))
            result['task_id']=plans[vehicle]['task_id']
        stamps=[r['published_at'] for r in results.values()]
        ok=all(r['submitted'] is True and r['command_accepted'] is True for r in results.values())
        return {'status':'OK' if ok else 'PARTIAL_OR_UNCONFIRMED','submitted':all(r['submitted'] is True for r in results.values()),
                'stage':'PUBLISHED','mode':'concurrent','publish_skew_sec':max(stamps)-min(stamps),
                'vehicles':results,'execution_state':'UNKNOWN',
                'message':'Published together; receipt is not mission completion. Do not automatically resubmit uncertain members.'}
    finally:
        for sub in subscribers:sub.unregister()
        for pub in publishers.values():pub.unregister()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group()
    inputs.add_argument("--file", type=Path)
    inputs.add_argument("--json")
    inputs.add_argument("--fleet-json", help="UAV ID -> original WGS84 batch mapping; validate all, publish concurrently")
    parser.add_argument("--uav-id", help="explicit vehicle; omitted selects an available vehicle")
    execution = parser.add_mutually_exclusive_group()
    execution.add_argument("--submit", action="store_true", help="publish after validation")
    execution.add_argument("--dry-run", action="store_true", help="explicit preview; also the default")
    parser.add_argument("--list", action="store_true", help="show live vehicles and reservations only")
    parser.add_argument("--enforce-separation", action="store_true",
                        help="opt in to the 2 m horizontal route separation check")
    args = parser.parse_args()

    if args.list and (args.file or args.json or args.fleet_json or args.submit or args.dry_run or args.uav_id or args.enforce_separation):
        parser.error("--list cannot be combined with a task or vehicle")
    if args.fleet_json:
        if args.uav_id:
            parser.error('--fleet-json already binds every UAV; do not use --uav-id')
        try:
            result = dispatch_group(json.loads(args.fleet_json),args.submit,args.enforce_separation)
        except (ValueError,RuntimeError,KeyError) as exc:
            print(json.dumps({'status':'ERROR','message':str(exc)},ensure_ascii=False))
            raise SystemExit(2)
        print(json.dumps(result,ensure_ascii=False,indent=2))
        if result['status'] != 'OK':raise SystemExit(1)
        return
    if not args.list and not (args.file or args.json):
        parser.error("a batch is required unless --list is used")
    if not args.list:
        batch = json.loads(args.file.read_text(encoding="utf-8") if args.file else args.json)
        if not isinstance(batch, list) or not batch or not all(isinstance(item, dict) for item in batch):
            parser.error("batch must be a non-empty JSON array of actions")
        task_ids = {item.get("task_id") for item in batch}
        if len(task_ids) != 1 or not next(iter(task_ids)):
            parser.error("all actions must share one non-empty task_id")
        task_id = next(iter(task_ids))
        if not args.uav_id and any(item.get("cmd") in ("follow_path", "patrol_move", "fuel_explore")
                                   for item in batch):
            parser.error("coordinate tasks require --uav-id after planning that UAV's complete route")
        last_step_id = batch[-1].get("step_id")
        if not isinstance(last_step_id, int) or last_step_id < 1:
            parser.error("last action needs a positive step_id")
    fleet = load_uavs()
    if args.uav_id and args.uav_id not in fleet:
        parser.error("unknown UAV ID")
    rospy.init_node("fleet_dispatch", anonymous=True, disable_signals=True)
    run_id = rospy.get_param("/run_id")

    LOCK_FILE.parent.mkdir(parents=True, exist_ok=True)
    with LOCK_FILE.open("a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        saved = load_state(run_id)
        nodes = set(rosnode.get_node_names())
        live = {}
        for vehicle, config in fleet.items():
            current = live_vehicle(vehicle, config, nodes)
            if current is not None:
                live[vehicle] = current
        release_completed(saved, live)
        if any(current["world_xy"] is None for current in live.values()):
            raise SystemExit("A connected UAV has no live pose; reservation check cannot be trusted")
        for vehicle, reservation in saved["reservations"].items():
            if vehicle in live:
                updated = remaining_path(reservation, live[vehicle]["world_xy"])
                if updated is not None:
                    reservation["path_xy"] = updated
        RESERVATIONS_FILE.write_text(json.dumps(saved, ensure_ascii=False, indent=2), encoding="utf-8")
        candidates = [vehicle for vehicle in fleet if vehicle in live
                      and vehicle not in saved["reservations"] and not live[vehicle]["armed"]
                      and live[vehicle]["status"].get("task_state", live[vehicle]["status"].get("state")) not in ("RUNNING", "PENDING", "PAUSED", "QUEUED", "ACCEPTED")]
        if args.list:
            print(json.dumps({"vehicles": live, "available": candidates,
                              "reservations": saved["reservations"]}, ensure_ascii=False, indent=2))
            return
        if args.uav_id:
            candidates = [vehicle for vehicle in candidates if vehicle == args.uav_id]
        if not candidates:
            raise SystemExit("No live, idle, unreserved SITL vehicle is available")
        vehicle = min(candidates, key=lambda item: (saved["last_used"].get(item, 0), item))
        config = fleet[vehicle]
        if not all(math.isfinite(float(config[key])) for key in ("origin_lat", "origin_lon")):
            raise SystemExit("Vehicle has no valid WGS84 origin")
        command = [sys.executable, str(SUBMIT_SCRIPT), "--json", json.dumps(batch, ensure_ascii=False),
                   "--uav-id", vehicle, "--origin-lat", str(config["origin_lat"]),
                   "--origin-lon", str(config["origin_lon"])]
        preview = subprocess.run(command + ["--dry-run"], text=True, capture_output=True)
        if preview.returncode != 0:
            raise SystemExit(preview.stdout.strip() or preview.stderr.strip())
        validated = json.loads(preview.stdout)
        path_xy = planned_xy(validated["batch"], config, live[vehicle]["world_xy"])
        regions_xy = planned_regions(validated["batch"], config)
        if args.enforce_separation:
            for other, reservation in saved["reservations"].items():
                if other not in live:
                    raise SystemExit("Reserved UAV has no live telemetry: " + other)
                other_path = reservation.get("path_xy")
                if not other_path:
                    raise SystemExit("Reserved path is unknown for " + other)
                if paths_too_close(path_xy, other_path):
                    raise SystemExit("Planned path is within 2 m of reserved path for " + other)
                other_regions = reservation.get("regions_xy", [])
                if (any(region_conflicts_path(region, other_path) for region in regions_xy)
                        or any(region_conflicts_path(region, path_xy) for region in other_regions)
                        or any(regions_conflict(first, second) for first in regions_xy for second in other_regions)):
                    raise SystemExit("Exploration region is within 2 m of reserved task for " + other)
            for other, current in live.items():
                if other != vehicle and paths_too_close(path_xy, [current["world_xy"]]):
                    raise SystemExit("Planned path is within 2 m of current position of " + other)
                if other != vehicle and any(region_conflicts_path(region, [current["world_xy"]]) for region in regions_xy):
                    raise SystemExit("Exploration region is within 2 m of current position of " + other)
                if other != vehicle and current["armed"] and other not in saved["reservations"]:
                    raise SystemExit("Another airborne UAV has no registered path: " + other)
        result = subprocess.run(command + ["--require-sim"], text=True, capture_output=True) if args.submit else preview
        if result.returncode != 0:
            raise SystemExit(result.stdout.strip() or result.stderr.strip())
        if args.submit:
            repeating = any(action["cmd"] in ("follow_path", "patrol_move") and
                            (action["params"].get("loop_forever") or
                             action["params"].get("repeat", 1) > 1)
                            for action in validated["batch"])
            saved["reservations"][vehicle] = {"task_id": task_id, "last_step_id": last_step_id,
                                               "executor_session_id": live[vehicle]["status"].get("executor_session_id"),
                                               "path_xy": path_xy, "regions_xy": regions_xy,
                                               "repeating": repeating}
            saved["last_used"][vehicle] = time.time()
            RESERVATIONS_FILE.write_text(json.dumps(saved, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"selected_uav": vehicle, "live_vehicles": sorted(live),
                          "submission": json.loads(result.stdout)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
