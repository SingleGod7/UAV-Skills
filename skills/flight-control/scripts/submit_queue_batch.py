#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Validate a WGS84 drone_control queue_batch, convert its coordinates to
MAVROS local ENU, and optionally publish the converted batch.

Coordinate-bearing actions require --origin-lat and --origin-lon. The origin
is assumed to be MAVROS local (0, 0, 0). Waypoint and FUEL altitudes are
relative to the takeoff point, so they become local z without subtraction.

Input:
  --file batch.json
  --json '[{...}]'
  or stdin JSON

By default publishes the converted batch to:
  /drone_control/queue_batch
  std_msgs/String

Use --dry-run to validate and convert without publishing.
"""

import argparse
import json
import math
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

SUPPORTED_CMDS = {
    "arm",
    "takeoff",
    "follow_path",
    "patrol_move",
    "fuel_explore",
    "hover",
    "land",
}

TOP_LEVEL_KEYS = {"cmd", "task_id", "step_id", "params"}
COORDINATE_CMDS = {"follow_path", "patrol_move", "fuel_explore"}
EARTH_RADIUS_M = 6378137.0
DEFAULT_MAX_DISTANCE_M = 10000.0
OUTPUT_DECIMALS = 6


class ValidationError(Exception):
    pass


def finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValidationError(f"{name} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValidationError(f"{name} must be finite")
    return value


def positive(value: Any, name: str) -> float:
    v = finite_number(value, name)
    if v <= 0:
        raise ValidationError(f"{name} must be > 0")
    return v


def nonnegative(value: Any, name: str) -> float:
    v = finite_number(value, name)
    if v < 0:
        raise ValidationError(f"{name} must be >= 0")
    return v


def validate_longitude(value: Any, name: str) -> float:
    lon = finite_number(value, name)
    if not -180 <= lon <= 180:
        raise ValidationError(f"{name} out of range [-180,180]")
    return lon


def validate_latitude(value: Any, name: str) -> float:
    lat = finite_number(value, name)
    if not -90 <= lat <= 90:
        raise ValidationError(f"{name} out of range [-90,90]")
    return lat


def validate_origin(
    origin_lat: Optional[float],
    origin_lon: Optional[float],
) -> Optional[List[float]]:
    if origin_lat is None and origin_lon is None:
        return None
    if origin_lat is None or origin_lon is None:
        raise ValidationError("--origin-lat and --origin-lon must be provided together")

    lat = validate_latitude(origin_lat, "--origin-lat")
    lon = validate_longitude(origin_lon, "--origin-lon")
    if abs(lat) == 90:
        raise ValidationError("--origin-lat cannot be a pole for ENU approximation")
    return [lat, lon]


def wgs84_to_local(
    lon: Any,
    lat: Any,
    relative_alt: Any,
    origin: List[float],
    max_distance_m: float,
    name: str,
) -> List[float]:
    target_lon = validate_longitude(lon, f"{name}.longitude")
    target_lat = validate_latitude(lat, f"{name}.latitude")
    z = finite_number(relative_alt, f"{name}.altitude")
    origin_lat, origin_lon = origin

    dlat = math.radians(target_lat - origin_lat)
    dlon = math.radians(target_lon - origin_lon)
    x = EARTH_RADIUS_M * dlon * math.cos(math.radians(origin_lat))
    y = EARTH_RADIUS_M * dlat
    distance = math.hypot(x, y)
    if distance > max_distance_m:
        raise ValidationError(
            f"{name} is {distance:.3f} m from origin, exceeds "
            f"--max-distance-m {max_distance_m:.3f}"
        )

    return [
        round(x, OUTPUT_DECIMALS),
        round(y, OUTPUT_DECIMALS),
        round(z, OUTPUT_DECIMALS),
    ]


def convert_wgs84_waypoints(
    value: Any,
    name: str,
    origin: List[float],
    max_distance_m: float,
) -> List[Any]:
    if not isinstance(value, list) or not value:
        raise ValidationError(f"{name} must be a non-empty array")

    converted = []
    required = {"longitude", "latitude", "altitude"}
    for i, waypoint in enumerate(value):
        item_name = f"{name}[{i}]"
        if not isinstance(waypoint, dict):
            raise ValidationError(
                f"{item_name} must be a WGS84 object with "
                "longitude, latitude, altitude"
            )

        missing = required - set(waypoint)
        unknown = set(waypoint) - required - {"pass_through"}
        if missing:
            raise ValidationError(f"{item_name} missing fields: {sorted(missing)}")
        if unknown:
            raise ValidationError(f"{item_name} unknown fields: {sorted(unknown)}")

        transit = waypoint.get("pass_through", False)
        if type(transit) is not bool:
            raise ValidationError(f"{item_name}.pass_through must be boolean")
        if transit and i == len(value)-1:
            raise ValidationError(f"{item_name}: final waypoint must require arrival")
        xyz = wgs84_to_local(
                waypoint["longitude"],
                waypoint["latitude"],
                waypoint["altitude"],
                origin,
                max_distance_m,
                item_name,
            )
        converted.append(dict(x=xyz[0], y=xyz[1], z=xyz[2], pass_through=True,
                              reach_xy=.35, reach_z=.25, hold_sec=0.) if transit else xyz)
    return converted


def validate_wgs84_box(value: Any, name: str) -> List[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValidationError(f"{name} must be [longitude,latitude,altitude]")
    return [
        validate_longitude(value[0], f"{name}[0]"),
        validate_latitude(value[1], f"{name}[1]"),
        finite_number(value[2], f"{name}[2]"),
    ]


def convert_wgs84_box(
    box_min_value: Any,
    box_max_value: Any,
    origin: List[float],
    max_distance_m: float,
) -> List[List[float]]:
    box_min = validate_wgs84_box(
        box_min_value,
        "fuel_explore.params.box_min",
    )
    box_max = validate_wgs84_box(
        box_max_value,
        "fuel_explore.params.box_max",
    )
    for i, axis in enumerate(("longitude", "latitude", "altitude")):
        if box_min[i] >= box_max[i]:
            raise ValidationError(
                f"fuel_explore {axis}: box_min must be < box_max"
            )

    local_min = wgs84_to_local(
        box_min[0],
        box_min[1],
        box_min[2],
        origin,
        max_distance_m,
        "fuel_explore.params.box_min",
    )
    local_max = wgs84_to_local(
        box_max[0],
        box_max[1],
        box_max[2],
        origin,
        max_distance_m,
        "fuel_explore.params.box_max",
    )
    for i, axis in enumerate(("x", "y", "z")):
        if local_min[i] >= local_max[i]:
            raise ValidationError(
                f"fuel_explore converted {axis}: box_min must be < box_max"
            )
    return [local_min, local_max]


def apply_skill_defaults(cmd: str, params: Dict[str, Any]) -> Dict[str, Any]:
    """Materialize legacy skill defaults at the ROS boundary, not model input."""
    result = dict(params)
    if cmd in {"takeoff", "follow_path", "patrol_move"}:
        if cmd == "takeoff" and "attitude" not in result:
            result.setdefault("altitude", 1.2)
        result.setdefault("speed", 1.2)
        # Explicit values retain precedence and are validated below.
        speed = positive(result["speed"], f"{cmd}.params.speed")
        result.setdefault("max_acc", max(speed * 2.0, 1.5))
        result.setdefault("segment_timeout_sec", 60.0)
        result.setdefault("skip_unreachable", True)
        if cmd in {"follow_path", "patrol_move"}:
            result.setdefault("repeat", 1)
            if "loop" not in result:
                result.setdefault("loop_forever", False)
    elif cmd == "fuel_explore":
        result.setdefault("max_vel", result.get("speed", 0.6))
        result.setdefault("max_acc", 0.5)
        result.setdefault("timeout_sec", 1200.0)
    return result


def convert_params(
    cmd: str,
    params: Any,
    origin: Optional[List[float]],
    max_distance_m: float,
) -> Dict[str, Any]:
    if not isinstance(params, dict):
        raise ValidationError(f"{cmd}.params must be an object")

    converted = apply_skill_defaults(cmd, params)
    converted.pop("_coordinates_already_mavros", None)
    if cmd not in COORDINATE_CMDS:
        return converted
    if origin is None:
        raise ValidationError(
            f"{cmd} requires --origin-lat and --origin-lon for WGS84 conversion"
        )

    if cmd in {"follow_path", "patrol_move"}:
        if "waypoints" not in params:
            raise ValidationError(f"{cmd}.params.waypoints is required")
        converted["waypoints"] = convert_wgs84_waypoints(
            params["waypoints"],
            f"{cmd}.params.waypoints",
            origin,
            max_distance_m,
        )
        return converted

    for required in ("box_min", "box_max"):
        if required not in params:
            raise ValidationError(f"fuel_explore.params.{required} is required")
    local_min, local_max = convert_wgs84_box(
        params["box_min"],
        params["box_max"],
        origin,
        max_distance_m,
    )
    converted["box_min"] = local_min
    converted["box_max"] = local_max
    return converted


def validate_local_waypoints(value: Any, name: str) -> None:
    if not isinstance(value, list) or not value:
        raise ValidationError(f"{name} must be a non-empty array")
    for i, waypoint in enumerate(value):
        if isinstance(waypoint, dict):
            expected = {"x", "y", "z", "pass_through", "reach_xy", "reach_z", "hold_sec"}
            if set(waypoint) != expected or waypoint.get("pass_through") is not True or i == len(value)-1:
                raise ValidationError(f"{name}[{i}] must be a generated intermediate waypoint")
            if waypoint["reach_xy"] != .35 or waypoint["reach_z"] != .25 or waypoint["hold_sec"] != 0.:
                raise ValidationError(f"{name}[{i}] invalid intermediate arrival limits")
            for axis in ("x", "y", "z"):
                finite_number(waypoint[axis], f"{name}[{i}].{axis}")
            continue
        if not isinstance(waypoint, list) or len(waypoint) != 3:
            raise ValidationError(f"{name}[{i}] must be local [x,y,z]")
        for j, item in enumerate(waypoint):
            finite_number(item, f"{name}[{i}][{j}]")


def validate_local_box(value: Any, name: str) -> List[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValidationError(f"{name} must be local [x,y,z]")
    return [finite_number(v, f"{name}[{i}]") for i, v in enumerate(value)]


def validate_params(cmd: str, params: Dict[str, Any]) -> None:
    if not isinstance(params, dict):
        raise ValidationError(f"{cmd}.params must be an object")
    if cmd in {"arm", "hover", "land"}:
        if params:
            raise ValidationError(f"{cmd}.params must be empty")
        return

    if cmd == "takeoff":
        if "altitude" in params:
            positive(params["altitude"], "takeoff.params.altitude")
        elif "attitude" in params:
            positive(params["attitude"], "takeoff.params.attitude")
        else:
            raise ValidationError("takeoff.params.altitude is required")
        for key in (
            "handover_altitude", "speed", "max_acc", "max_jerk",
            "segment_timeout_sec"
        ):
            if key in params:
                positive(params[key], f"takeoff.params.{key}")
        if "arrival_hold_sec" in params:
            nonnegative(params["arrival_hold_sec"], "takeoff.params.arrival_hold_sec")
        for key in ("wait_reach", "skip_unreachable"):
            if key in params and not isinstance(params[key], bool):
                raise ValidationError(f"takeoff.params.{key} must be boolean")
        return

    if cmd in {"follow_path", "patrol_move"}:
        if "waypoints" not in params:
            raise ValidationError(f"{cmd}.params.waypoints is required")
        validate_local_waypoints(params["waypoints"], f"{cmd}.params.waypoints")

        if "repeat" in params:
            repeat = params["repeat"]
            if isinstance(repeat, bool) or not isinstance(repeat, int):
                raise ValidationError(f"{cmd}.params.repeat must be an integer")

        for key in ("loop_forever", "loop", "wait_reach", "skip_unreachable"):
            if key in params and not isinstance(params[key], bool):
                raise ValidationError(f"{cmd}.params.{key} must be boolean")

        for key in (
            "speed", "max_acc", "max_jerk",
            "segment_republish_sec", "segment_timeout_sec"
        ):
            if key in params:
                positive(params[key], f"{cmd}.params.{key}")

        if "arrival_hold_sec" in params:
            nonnegative(params["arrival_hold_sec"], f"{cmd}.params.arrival_hold_sec")
        return

    if cmd == "fuel_explore":
        for required in ("box_min", "box_max"):
            if required not in params:
                raise ValidationError(f"fuel_explore.params.{required} is required")

        box_min = validate_local_box(
            params["box_min"],
            "fuel_explore.params.box_min",
        )
        box_max = validate_local_box(
            params["box_max"],
            "fuel_explore.params.box_max",
        )
        for i, axis in enumerate(("x", "y", "z")):
            if box_min[i] >= box_max[i]:
                raise ValidationError(
                    f"fuel_explore {axis}: box_min must be < box_max"
                )

        for key in (
            "max_vel", "max_acc", "entry_timeout_sec",
            "startup_timeout_sec", "first_command_timeout_sec", "timeout_sec"
        ):
            if key in params:
                positive(params[key], f"fuel_explore.params.{key}")

        for key in ("entry_hold_sec", "start_retry_delay_sec"):
            if key in params:
                nonnegative(params[key], f"fuel_explore.params.{key}")

        if "start_retry_count" in params:
            value = params["start_retry_count"]
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValidationError(
                    "fuel_explore.params.start_retry_count must be integer >= 0"
                )
        return

    raise ValidationError(f"validator missing command: {cmd}")


def validate_batch(
    batch: Any,
    origin_lat: Optional[float] = None,
    origin_lon: Optional[float] = None,
    max_distance_m: float = DEFAULT_MAX_DISTANCE_M,
) -> List[Dict[str, Any]]:
    if not isinstance(batch, list) or not batch:
        raise ValidationError("batch must be a non-empty JSON array")

    max_distance = positive(max_distance_m, "--max-distance-m")
    origin = validate_origin(origin_lat, origin_lon)
    validated = []
    previous_step = None

    # IMPORTANT: validation and conversion finish entirely before publish.
    for index, action in enumerate(batch):
        prefix = f"batch[{index}]"

        if not isinstance(action, dict):
            raise ValidationError(f"{prefix} must be an object")

        unknown_top = set(action) - TOP_LEVEL_KEYS
        missing_top = TOP_LEVEL_KEYS - set(action)
        if unknown_top:
            raise ValidationError(f"{prefix} unknown fields: {sorted(unknown_top)}")
        if missing_top:
            raise ValidationError(f"{prefix} missing fields: {sorted(missing_top)}")

        cmd = action["cmd"]
        if not isinstance(cmd, str) or cmd not in SUPPORTED_CMDS:
            raise ValidationError(f"{prefix}.cmd unsupported: {cmd!r}")

        task_id = action["task_id"]
        if not isinstance(task_id, str) or not task_id.strip():
            raise ValidationError(f"{prefix}.task_id must be a non-empty string")

        step_id = action["step_id"]
        if isinstance(step_id, bool) or not isinstance(step_id, int) or step_id < 1:
            raise ValidationError(f"{prefix}.step_id must be integer >= 1")

        if validated and task_id != validated[0]["task_id"]:
            raise ValidationError("all actions in a batch must use the same task_id")

        if previous_step is not None and step_id <= previous_step:
            raise ValidationError(
                f"{prefix}.step_id must be strictly increasing in batch order"
            )
        previous_step = step_id

        try:
            converted_params = convert_params(
                cmd,
                action["params"],
                origin,
                max_distance,
            )
            validate_params(cmd, converted_params)
        except ValidationError as exc:
            raise ValidationError(f"{prefix}: {exc}") from exc

        if cmd in COORDINATE_CMDS:
            converted_params["_coordinates_already_mavros"] = True

        converted_action = dict(action)
        converted_action["params"] = converted_params
        validated.append(converted_action)

    return validated


def load_input(args: argparse.Namespace) -> Any:
    sources = sum(bool(x) for x in (args.file, args.json))
    if sources > 1:
        raise ValidationError("use only one of --file or --json")

    if args.file:
        raw = Path(args.file).read_text(encoding="utf-8")
    elif args.json:
        raw = args.json
    else:
        raw = sys.stdin.read()

    if not raw.strip():
        raise ValidationError("empty input")

    try:
        return json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValidationError(f"invalid JSON: {exc}") from exc


def publish(batch: List[Dict[str, Any]], topic: str) -> None:
    rostopic = shutil.which("rostopic")
    if not rostopic:
        raise RuntimeError("rostopic not found in PATH")

    payload = json.dumps(batch, ensure_ascii=False, separators=(",", ":"))
    ros_message = json.dumps({"data": payload}, ensure_ascii=False)

    proc = subprocess.run(
        [rostopic, "pub", "-1", topic, "std_msgs/String", ros_message],
        text=True,
        capture_output=True,
        timeout=15,
    )

    if proc.returncode != 0:
        detail = (proc.stderr or proc.stdout or "").strip()
        raise RuntimeError(f"rostopic publish failed: {detail}")


def control_paths(uav_id: Optional[str]) -> Dict[str, str]:
    if not uav_id:
        namespace = ""
    else:
        sys.path.insert(0, str(Path(__file__).resolve().parents[5]))
        from uav_runtime.config import get_uav
        try:
            namespace = "/" + get_uav(uav_id)["namespace"]
        except (ValueError, KeyError, OSError) as exc:
            raise ValidationError(str(exc))
    return {
        "queue": namespace + "/drone_control/queue_batch",
        "profile": namespace + "/drone_control/runtime_profile",
        "node": namespace + "/drone_control",
        "state": namespace + "/mavros/state",
    }


def require_sim_runtime(topic: str, paths: Dict[str, str]) -> None:
    if topic != paths["queue"]:
        raise RuntimeError(f"simulation submission requires {paths['queue']}")
    rosparam = shutil.which("rosparam")
    rostopic = shutil.which("rostopic")
    if not rosparam or not rostopic:
        raise RuntimeError("ROS command line tools not found; source ROS and catkin setup")
    profile = subprocess.run(
        [rosparam, "get", paths["profile"]],
        text=True, capture_output=True, timeout=5,
    )
    if profile.returncode != 0 or profile.stdout.strip().strip("\"'") != "sim":
        raise RuntimeError(f"refusing submission: {paths['profile']} is not sim")
    info = subprocess.run(
        [rostopic, "info", topic], text=True, capture_output=True, timeout=5,
    )
    subscribers = info.stdout.partition("Subscribers:")[2]
    if info.returncode != 0 or not any(
        line.strip().split()[:2] == ["*", paths["node"]]
        for line in subscribers.splitlines()
    ):
        raise RuntimeError(f"refusing submission: queue has no {paths['node']} subscriber")
    state = subprocess.run(
        [rostopic, "echo", "-n", "1", paths["state"]],
        text=True, capture_output=True, timeout=5,
    )
    if state.returncode != 0 or not re.search(r"^connected:\s*True\s*$", state.stdout, re.M):
        raise RuntimeError(f"refusing submission: {paths['state']} is not connected")


def conversion_summary(
    batch: Any,
    origin_lat: Optional[float],
    origin_lon: Optional[float],
    max_distance_m: float,
) -> Dict[str, Any]:
    performed = isinstance(batch, list) and any(
        isinstance(action, dict) and action.get("cmd") in COORDINATE_CMDS
        for action in batch
    )
    summary: Dict[str, Any] = {
        "performed": performed,
        "source_coordinate_system": "wgs84_lon_lat_relative_altitude",
        "target_coordinate_system": "mavros_local_enu",
        "origin_local": [0.0, 0.0, 0.0],
        "altitude_mode": "relative_to_takeoff",
        "max_distance_m": max_distance_m,
    }
    if origin_lat is not None and origin_lon is not None:
        summary["origin_gps"] = {
            "latitude": origin_lat,
            "longitude": origin_lon,
        }
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Validate WGS84 queue_batch and convert it to MAVROS local ENU"
    )
    ap.add_argument("--file", help="WGS84 JSON batch file")
    ap.add_argument("--json", help="WGS84 JSON batch string")
    ap.add_argument("--uav-id", help="target vehicle, for example UAV-001")
    ap.add_argument("--topic", help="queue topic; defaults to the selected vehicle")
    ap.add_argument("--origin-lat", type=float, help="takeoff-origin latitude")
    ap.add_argument("--origin-lon", type=float, help="takeoff-origin longitude")
    ap.add_argument(
        "--max-distance-m",
        type=float,
        default=DEFAULT_MAX_DISTANCE_M,
        help="maximum horizontal distance supported by the ENU approximation",
    )
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--require-sim", action="store_true",
                    help="required for publishing; check the live simulation profile and subscriber")
    args = ap.parse_args()

    try:
        paths = control_paths(args.uav_id)
        args.topic = args.topic or paths["queue"]
        batch = load_input(args)
        validated = validate_batch(
            batch,
            origin_lat=args.origin_lat,
            origin_lon=args.origin_lon,
            max_distance_m=args.max_distance_m,
        )
        conversion = conversion_summary(
            batch,
            args.origin_lat,
            args.origin_lon,
            args.max_distance_m,
        )

        if args.dry_run:
            print(json.dumps({
                "status": "OK",
                "validated": True,
                "submitted": False,
                "stage": "VALIDATED",
                "execution_state": "NOT_STARTED",
                "topic": args.topic,
                "action_count": len(validated),
                "conversion": conversion,
                "batch": validated,
            }, ensure_ascii=False, indent=2))
            return 0

        if not args.require_sim:
            raise ValidationError("publishing requires --require-sim; use --dry-run to validate only")
        require_sim_runtime(args.topic, paths)
        publish(validated, args.topic)

        print(json.dumps({
            "status": "OK",
            "validated": True,
            "submitted": True,
            "stage": "PUBLISHED",
            "command_accepted": None,
            "execution_state": "UNKNOWN",
            "message": "Transport published only; verify command_ack and task_state before reporting execution result",
            "topic": args.topic,
            "action_count": len(validated),
            "conversion": conversion,
        }, ensure_ascii=False, indent=2))
        return 0

    except ValidationError as exc:
        print(json.dumps({
            "status": "INVALID_ARGUMENT",
            "validated": False,
            "submitted": False,
            "message": str(exc),
        }, ensure_ascii=False, indent=2))
        return 2

    except Exception as exc:
        print(json.dumps({
            "status": "ERROR",
            "validated": False,
            "submitted": False,
            "message": str(exc),
        }, ensure_ascii=False, indent=2))
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
