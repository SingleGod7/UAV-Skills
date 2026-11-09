#!/usr/bin/env python3
"""Launch per-UAV perception and target behavior; retain legacy single-UAV support."""

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path


PLAN_AGENT_ROOT = Path(__file__).resolve().parents[3]
MIGRATION_ROOT = PLAN_AGENT_ROOT.parents[1]
sys.path.insert(0, str(MIGRATION_ROOT))
from uav_runtime.config import get_uav
from uav_runtime.search_targets import validate_targets


def _resolve_python_command(value):
    """Resolve either an explicit interpreter path or a PATH command."""
    raw = str(value or "").strip()
    if raw and os.path.sep not in raw:
        located = shutil.which(raw)
        if located:
            return Path(located).expanduser().resolve()
    return Path(raw or "python3").expanduser().resolve()


def _can_run_yolo_runtime(python_path, runtime_env=None):
    try:
        probe = subprocess.run(
            [str(python_path), "-c", "import torch, cv2, rospy, cv_bridge, ultralytics"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
            timeout=12,
            env=runtime_env,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return probe.returncode == 0


def _default_yolo_python(runtime_env):
    """Prefer a verified dedicated environment even if Nanobot lacks env vars."""
    candidates = [
        str(Path.home() / ".venvs" / "yolo_ros" / "bin" / "python"),
        str(Path.home() / "anaconda3" / "envs" / "yolo_ros" / "bin" / "python"),
        str(Path.home() / "anaconda3" / "envs" / "yolo" / "bin" / "python"),
        str(Path.home() / "miniforge3" / "envs" / "yolo_ros" / "bin" / "python"),
        str(Path.home() / "miniconda3" / "envs" / "yolo_ros" / "bin" / "python"),
        "python3",
    ]
    for candidate in candidates:
        path = _resolve_python_command(candidate)
        if path.is_file() and os.access(str(path), os.X_OK) and _can_run_yolo_runtime(path, runtime_env):
            return str(path)
    return os.environ.get("YOLO_PYTHON", "python3")


DEFAULT_PYTHON = os.environ.get("YOLO_PYTHON")
DEFAULT_WORKDIR = os.environ.get("YOLO_WORKDIR", str(PLAN_AGENT_ROOT.parent / "yolo"))
DEFAULT_ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/noetic/setup.bash")
DEFAULT_PID_FILE = "/tmp/plan_agent_target_detection.pid"
DEFAULT_LOG_FILE = os.environ.get(
    "YOLO_LOG_FILE", str(PLAN_AGENT_ROOT / "logs" / "target_detection.log")
)
DEFAULT_READY_TIMEOUT_SEC = 45.0
DEFAULT_READY_POLL_SEC = 0.25

MODE_CONFIG = {
    "detect-and-continue": {
        "backend": "xuanting_target_follow_yaw.py",
        "allowed": {
            "target_class",
            "classes",
            "target_description",
            "targets",
            "search_id",
            "completion_policy",
        },
        "injected": {
            "enable_follow_control": False,
            "show": False,
            "output_image_topic": "/yolo/tracking_image",
            "output_json_topic": "/yolo/tracking_detections",
        },
        "continues_current_task": True,
        "handover_source": None,
        "image_topic": "/yolo/tracking_image",
        "detections_topic": "/yolo/tracking_detections",
    },
    "track-target": {
        "backend": "xuanting_target_follow_yaw.py",
        "allowed": {
            "target_class",
            "classes",
            "home_position",
            "land_on_return",
            "direct_handover",
            "tracking_timeout_sec",
            "desired_distance",
            "target_description",
            "start_condition",
            "after_track",
            "face_target_yaw",
            "tracking_max_vel",
            "tracking_lost_timeout_sec",
            "post_track_hover_sec",
        },
        "injected": {
            "enable_follow_control": True,
            "require_active_exploration": True,
            "show": False,
            "output_image_topic": "/yolo/tracking_image",
            "output_json_topic": "/yolo/tracking_detections",
            "home_position": [0.0, 0.0, 1.2],
            "return_speed": 1.2,
            "land_on_return": True,
        },
        "continues_current_task": False,
        "handover_source": "fuel_explore",
        "image_topic": "/yolo/tracking_image",
        "detections_topic": "/yolo/tracking_detections",
    },
    "detect-and-return": {
        "backend": "xuanting.py",
        "allowed": {
            "target_class",
            "classes",
            "home_position",
            "land_on_return",
            "target_hover_sec",
        },
        "injected": {
            "require_active_exploration": True,
            "show": False,
        },
        "continues_current_task": False,
        "handover_source": "fuel_explore",
        "image_topic": "/yolo/ranging_image",
        "detections_topic": "/yolo/ranging_detections",
    },
}

BOOLEAN_PARAMS = {
    "land_on_return",
    "direct_handover",
    "face_target_yaw",
}
POSITIVE_PARAMS = {"tracking_timeout_sec", "desired_distance", "tracking_max_vel", "tracking_lost_timeout_sec"}
NONNEGATIVE_PARAMS = {"post_track_hover_sec", "target_hover_sec"}


def fail(message):
    raise SystemExit(message)


def finite_number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def validate_params(mode, params):
    config = MODE_CONFIG[mode]
    unknown = sorted(set(params) - config["allowed"])
    if unknown:
        fail(f"unsupported parameter(s) for {mode}: {', '.join(unknown)}")

    target_class = params.get("target_class")
    if 'targets' in params:
        try:validate_targets(params['targets'])
        except ValueError as exc:fail(str(exc))
        if 'target_class' in params or 'target_description' in params or 'classes' in params:
            fail('targets replace target_class, target_description and classes')
        if not isinstance(params.get('search_id'),str) or not params['search_id']:
            fail('targets require a shared search_id')
    elif not isinstance(target_class, str) or not target_class.strip():
        fail("target_class must be a non-empty model label")
    if "target_description" in params and (not isinstance(params["target_description"], str)
                                            or len(params["target_description"]) > 2000):
        fail("target_description must be a string of at most 2000 characters")

    if "classes" in params:
        classes = params["classes"]
        if (
            not isinstance(classes, list)
            or not classes
            or any(
                not isinstance(value, int)
                or isinstance(value, bool)
                or value < 0
                for value in classes
            )
        ):
            fail(
                "classes must be a non-empty JSON array "
                "of non-negative integers"
            )

    for key in BOOLEAN_PARAMS & params.keys():
        if not isinstance(params[key], bool):
            fail(f"{key} must be a JSON boolean")

    for key in POSITIVE_PARAMS & params.keys():
        if not finite_number(params[key]) or params[key] <= 0:
            fail(f"{key} must be a finite number greater than 0")

    for key in NONNEGATIVE_PARAMS & params.keys():
        if not finite_number(params[key]) or params[key] < 0:
            fail(
                f"{key} must be a finite number "
                "greater than or equal to 0"
            )

    if params.get('start_condition','fuel-explore') not in ('fuel-explore','mission','hover'):
        fail('start_condition must be fuel-explore, mission or hover')
    if params.get('after_track','resume') not in ('resume','return','hover'):
        fail('after_track must be resume, return or hover')
    if params.get('tracking_max_vel',.25) > .4:
        fail('tracking_max_vel exceeds controller cap 0.4 m/s')
    if params.get('after_track')=='return' and 'home_position' not in params:
        fail('after_track=return requires an explicit local home_position')
    if "home_position" in params:
        home = params["home_position"]
        if (
            not isinstance(home, list)
            or len(home) != 3
            or any(not finite_number(value) for value in home)
        ):
            fail(
                "home_position must be a JSON array of three finite "
                "numbers in the controller frame"
            )


def ros_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, dict)):
        return json.dumps(
            value,
            ensure_ascii=False,
            separators=(",", ":"),
        )
    return str(value)


def resolve_runtime(python, workdir, backend, runtime_env):
    python_path = _resolve_python_command(python)
    workdir_path = Path(workdir).expanduser().resolve()
    backend_path = workdir_path / backend

    if not python_path.is_file() or not os.access(str(python_path), os.X_OK):
        fail(f"python interpreter is not executable: {python_path}")
    if not _can_run_yolo_runtime(python_path, runtime_env):
        fail(
            "YOLO Python environment is incomplete: %s; expected torch, cv2, rospy, "
            "cv_bridge and ultralytics. Omit --python to use the verified dedicated "
            "YOLO environment, and do not use /usr/bin/python3." % python_path
        )
    if not workdir_path.is_dir():
        fail(f"YOLO workdir does not exist: {workdir_path}")
    if not backend_path.is_file():
        fail(f"YOLO backend does not exist: {backend_path}")
    return python_path, workdir_path, backend_path


def load_ros_environment(ros_setup):
    """Load ROS into an explicit child environment without relying on a login shell."""
    setup_path = Path(ros_setup).expanduser().resolve()
    if not setup_path.is_file():
        fail(f"ROS setup script does not exist: {setup_path}")

    result = subprocess.run(
        [
            "/bin/bash",
            "-c",
            'source "$1" >/dev/null && env -0',
            "bash",
            str(setup_path),
        ],
        env=os.environ.copy(),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        error = result.stderr.decode("utf-8", errors="replace").strip()
        fail(f"failed to load ROS environment from {setup_path}: {error}")

    runtime_env = {}
    for entry in result.stdout.split(b"\0"):
        if not entry or b"=" not in entry:
            continue
        key, value = entry.split(b"=", 1)
        runtime_env[
            os.fsdecode(key)
        ] = os.fsdecode(value)
    return setup_path, runtime_env


def find_running_backend(namespace=None):
    backend_names = {config["backend"] for config in MODE_CONFIG.values()} | {"perception_node.py"}
    proc_root = Path("/proc")
    for proc_dir in proc_root.iterdir():
        if not proc_dir.name.isdigit() or int(proc_dir.name) == os.getpid():
            continue
        try:
            tokens = (proc_dir / "cmdline").read_bytes().split(b"\0")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if namespace and ("__ns:=/" + namespace).encode() not in tokens:
            continue
        for token in tokens:
            if token and Path(os.fsdecode(token)).name in backend_names:
                return int(proc_dir.name), Path(os.fsdecode(token)).name
    return None


def read_log_tail(log_path, max_bytes=4000):
    try:
        with Path(log_path).open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            size = stream.tell()
            stream.seek(max(0, size - max_bytes), os.SEEK_SET)
            return stream.read().decode("utf-8", errors="replace").strip()
    except OSError:
        return ""


def terminate_started_backend(proc, timeout_sec=5.0):
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=timeout_sec)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait()


def topic_has_publisher(topic, runtime_env):
    try:
        result = subprocess.run(
            ["rostopic", "info", topic],
            env=runtime_env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    output = result.stdout
    ready = (
        result.returncode == 0
        and "Publishers:" in output
        and "Publishers: None" not in output
    )
    error = result.stderr.strip() or output.strip()
    return ready, error


def wait_for_backend_ready(
    proc,
    topics,
    message_topic,
    runtime_env,
    timeout_sec,
    poll_sec,
):
    """Wait for publishers and one processed detection message."""
    deadline = time.monotonic() + timeout_sec
    last_error = ""
    while time.monotonic() < deadline:
        return_code = proc.poll()
        if return_code is not None:
            raise RuntimeError(
                f"backend exited before ROS readiness: returncode={return_code}"
            )

        all_ready = True
        for topic in topics:
            ready, error = topic_has_publisher(topic, runtime_env)
            if not ready:
                all_ready = False
                last_error = error
                break
        if all_ready:
            break
        time.sleep(poll_sec)
    else:
        detail = f"; last ROS error: {last_error}" if last_error else ""
        raise RuntimeError(
            f"timed out waiting for YOLO publishers: {', '.join(topics)}{detail}"
        )

    remaining = max(0.1, deadline - time.monotonic())
    try:
        echo_proc = subprocess.Popen(
            ["rostopic", "echo", "-n", "1", message_topic],
            env=runtime_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
    except OSError as exc:
        raise RuntimeError(f"failed to start ROS message probe: {exc}") from exc

    try:
        while time.monotonic() < deadline:
            return_code = proc.poll()
            if return_code is not None:
                raise RuntimeError(
                    f"backend exited before first detection message: returncode={return_code}"
                )
            probe_return_code = echo_proc.poll()
            if probe_return_code is not None:
                error = echo_proc.stderr.read().strip() if echo_proc.stderr else ""
                if probe_return_code == 0:
                    return {
                        "publishers_ready": True,
                        "first_message_received": True,
                        "message_topic": message_topic,
                    }
                raise RuntimeError(
                    "ROS detection-message probe failed"
                    + (f": {error}" if error else "")
                )
            time.sleep(min(poll_sec, remaining))
        raise RuntimeError(
            f"timed out waiting for first YOLO message on {message_topic}"
        )
    finally:
        if echo_proc.poll() is None:
            echo_proc.terminate()
            try:
                echo_proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                echo_proc.kill()
                echo_proc.wait()


def start_detached(
    cmd,
    workdir,
    pid_file,
    log_file,
    startup_wait_sec,
    runtime_env,
    topics,
    message_topic,
    ready_timeout_sec,
    ready_poll_sec,
    namespace=None,
):
    running = find_running_backend(namespace)
    if running is not None:
        pid, backend = running
        fail(f"target detection backend already running: pid={pid}, backend={backend}")

    pid_path = Path(pid_file).expanduser()
    log_path = Path(log_file).expanduser()
    pid_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    # One log represents one detached run.  Appending made old interpreter
    # failures look like failures of a later healthy launch to both the web UI
    # and the agent reading the log.
    with log_path.open("w", encoding="utf-8") as log_stream:
        log_stream.write("=== YOLO run started ===\ncommand: %s\n\n" % " ".join(cmd))
        log_stream.flush()
        proc = subprocess.Popen(
            cmd,
            cwd=str(workdir),
            env=runtime_env,
            text=True,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            close_fds=True,
        )

    time.sleep(startup_wait_sec)
    return_code = proc.poll()
    if return_code is not None:
        fail(
            "target detection backend exited during startup: "
            f"returncode={return_code}, log={log_path}"
        )

    try:
        readiness = wait_for_backend_ready(
            proc,
            topics,
            message_topic,
            runtime_env,
            ready_timeout_sec,
            ready_poll_sec,
        )
    except RuntimeError as exc:
        terminate_started_backend(proc)
        log_tail = read_log_tail(log_path)
        detail = f"\nYOLO log tail:\n{log_tail}" if log_tail else ""
        fail(f"target detection backend failed readiness: {exc}; log={log_path}{detail}")

    pid_path.write_text(f"{proc.pid}\n", encoding="utf-8")
    return proc.pid, pid_path, log_path, readiness


def stop_detached(pid_file, stop_timeout_sec, namespace=None):
    pid_path = Path(pid_file).expanduser()
    if not pid_path.is_file():
        return {"status": "NOT_RUNNING", "pid_file": str(pid_path)}
    try:
        pid = int(pid_path.read_text(encoding="utf-8").strip())
    except (OSError, ValueError) as exc:
        fail(f"invalid target detection pid file {pid_path}: {exc}")

    proc_cmdline = Path(f"/proc/{pid}/cmdline")
    if not proc_cmdline.exists():
        pid_path.unlink(missing_ok=True)
        return {"status": "NOT_RUNNING", "pid": pid, "pid_file": str(pid_path)}

    try:
        tokens = proc_cmdline.read_bytes().split(b"\0")
    except (PermissionError, ProcessLookupError) as exc:
        fail(f"cannot verify managed target detection pid {pid}: {exc}")
    backend_names = {config["backend"] for config in MODE_CONFIG.values()}
    if not any(
        token and Path(os.fsdecode(token)).name in backend_names
        for token in tokens
    ):
        fail(f"refusing to stop pid {pid}: process is not a known YOLO backend")
    if namespace and ("__ns:=/" + namespace).encode() not in tokens:
        fail(f"refusing to stop pid {pid}: backend does not belong to {namespace}")

    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + stop_timeout_sec
    while time.monotonic() < deadline:
        if not proc_cmdline.exists():
            pid_path.unlink(missing_ok=True)
            return {"status": "STOPPED", "pid": pid, "pid_file": str(pid_path)}
        time.sleep(0.1)
    fail(f"target detection pid {pid} did not stop within {stop_timeout_sec}s")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=sorted(MODE_CONFIG))
    parser.add_argument("--uav-id",
                        help="isolate all ROS inputs, outputs and runtime files to one simulated UAV")
    parser.add_argument("--backend", help="replaceable backend for configured UAV perception")
    parser.add_argument(
        "--params-json",
        help=(
            "Published task parameters as a JSON object, "
            "without leading underscores."
        ),
    )
    parser.add_argument("--python", default=None)
    parser.add_argument("--workdir", default=DEFAULT_WORKDIR)
    parser.add_argument(
        "--ros-setup",
        default=DEFAULT_ROS_SETUP,
        help="ROS setup.bash loaded into the YOLO child process environment",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--detach",
        action="store_true",
        help="start the YOLO backend in a new session and return immediately",
    )
    parser.add_argument("--pid-file", default=DEFAULT_PID_FILE)
    parser.add_argument("--log-file", default=DEFAULT_LOG_FILE)
    parser.add_argument("--startup-wait-sec", type=float, default=1.0)
    parser.add_argument(
        "--ready-timeout-sec",
        type=float,
        default=DEFAULT_READY_TIMEOUT_SEC,
        help="maximum time to wait for YOLO publishers and one processed message",
    )
    parser.add_argument(
        "--ready-poll-sec",
        type=float,
        default=DEFAULT_READY_POLL_SEC,
        help="poll interval while waiting for YOLO ROS readiness",
    )
    parser.add_argument(
        "--stop",
        action="store_true",
        help="stop the detached YOLO backend recorded by --pid-file",
    )
    parser.add_argument("--stop-timeout-sec", type=float, default=5.0)
    args = parser.parse_args()
    try:
        uav_config = get_uav(args.uav_id) if args.uav_id else None
    except (ValueError, KeyError, OSError) as exc:
        fail(str(exc))
    namespace = uav_config["namespace"] if uav_config else None
    if namespace:
        if args.pid_file == DEFAULT_PID_FILE:
            args.pid_file = "/tmp/plan_agent_target_detection_%s.pid" % namespace
        if args.log_file == DEFAULT_LOG_FILE:
            args.log_file = str(PLAN_AGENT_ROOT / "logs" / ("target_detection_%s.log" % namespace))

    if args.stop:
        if args.mode is not None or args.params_json is not None:
            fail("--stop cannot be combined with --mode or --params-json")
        if args.dry_run or args.detach:
            fail("--stop cannot be combined with --dry-run or --detach")
        if not math.isfinite(args.stop_timeout_sec) or args.stop_timeout_sec <= 0:
            fail("--stop-timeout-sec must be a finite number > 0")
        legacy = stop_detached(args.pid_file, args.stop_timeout_sec, namespace)
        if namespace and (Path(os.environ.get("UAV_RUNTIME_ROOT", "/tmp/uav-runtime")) / "current.json").is_file():
            _, runtime_env = load_ros_environment(args.ros_setup)
            raise SystemExit(subprocess.run([sys.executable, str(MIGRATION_ROOT / "scripts/uav_perception.py"),
                                            "stop", "--uav-id", args.uav_id], env=runtime_env).returncode)
        print(json.dumps(legacy))
        return

    if args.mode is None or args.params_json is None:
        fail("--mode and --params-json are required unless --stop is used")
    if args.dry_run and args.detach:
        fail("--dry-run and --detach cannot be used together")
    if not math.isfinite(args.startup_wait_sec) or args.startup_wait_sec < 0:
        fail("--startup-wait-sec must be a finite number >= 0")
    if not math.isfinite(args.ready_timeout_sec) or args.ready_timeout_sec <= 0:
        fail("--ready-timeout-sec must be a finite number > 0")
    if not math.isfinite(args.ready_poll_sec) or args.ready_poll_sec <= 0:
        fail("--ready-poll-sec must be a finite number > 0")

    try:
        params = json.loads(args.params_json)
    except json.JSONDecodeError as exc:
        fail(f"invalid --params-json: {exc}")
    if not isinstance(params, dict):
        fail("--params-json must be a JSON object")

    validate_params(args.mode, params)
    if args.backend and not namespace:
        fail("--backend requires a configured --uav-id")
    if 'targets' in params and not namespace:
        fail('multi-target searches require a configured --uav-id')
    if namespace:
        query = ({'targets':params['targets'],'search_id':params['search_id'],
                  'completion_policy':params.get('completion_policy','find-once')}
                 if 'targets' in params else {"categories": [params["target_class"]]})
        family = args.backend or uav_config.get("perception", {}).get("backend", "yolo")
        if "targets" in params and family != "yolo-ds-flash":
            fail("targets require yolo-ds-flash")
        if "target_description" in params:
            if family != "yolo-ds-flash":
                fail("target_description requires yolo-ds-flash")
            query["target_description"] = params["target_description"]
        if params.get("classes"):
            if family == "yolo-ds-flash":
                fail("central semantic searches use target_class text; omit classes IDs")
            if family in ("yoloe", "yolo-world"):
                fail("open vocabulary uses target_class text; omit COCO classes IDs")
            query["class_ids"] = params["classes"]
        command = [sys.executable, str(MIGRATION_ROOT / "scripts/uav_perception.py"), "start",
                   "--uav-id", args.uav_id, "--query-json", json.dumps(query),
                   "--mode", args.mode, "--params-json", json.dumps(params),
                   "--ready-timeout-sec", str(args.ready_timeout_sec)]
        if args.backend:
            command.extend(["--backend", args.backend])
        if args.python:
            command.extend(["--model-python", args.python])
        if args.dry_run:
            command.append("--dry-run")
            runtime_env = os.environ.copy()
        else:
            _, runtime_env = load_ros_environment(args.ros_setup)
        raise SystemExit(subprocess.run(command, env=runtime_env).returncode)
    if args.mode == 'track-target':
        fail('tracking with task restoration requires a configured --uav-id')
    config = MODE_CONFIG[args.mode]
    ros_setup_path, runtime_env = load_ros_environment(args.ros_setup)
    python_path, workdir_path, backend_path = resolve_runtime(
        args.python or DEFAULT_PYTHON or _default_yolo_python(runtime_env),
        args.workdir,
        config["backend"],
        runtime_env,
    )
    forwarded = {**config["injected"], **params}
    image_topic = config["image_topic"]
    detections_topic = config["detections_topic"]
    if namespace:
        prefix = "/" + namespace
        image_topic = prefix + image_topic
        detections_topic = prefix + detections_topic
        forwarded.update({
            "rgb_topic": prefix + "/camera/rgb/image_raw",
            "depth_topic": prefix + "/camera/depth/image_raw",
            "camera_info_topic": prefix + "/camera/depth/camera_info",
            "odom_topic": prefix + "/mavros/local_position/odom",
            "drone_model_name": uav_config["model_name"],
            "control_action_topic": prefix + "/drone_control/action",
            "control_queue_batch_topic": prefix + "/drone_control/queue_batch",
            "control_status_topic": prefix + "/drone_control/exec_status",
            "controller_tracking_enabled_param": prefix + "/drone_control/target_tracking_enabled",
            "tracked_target_topic": prefix + "/yolo/tracked_target",
            "output_image_topic": image_topic,
            "output_json_topic": detections_topic,
        })
    cmd = [str(python_path), str(backend_path)]
    if namespace:
        cmd.append("__ns:=/" + namespace)
    for key, value in forwarded.items():
        cmd.append(f"_{key}:={ros_value(value)}")

    if args.dry_run:
        print(
            json.dumps(
                {
                    "status": "OK",
                    "mode": args.mode,
                    "backend": config["backend"],
                    "command": cmd,
                    "continues_current_task": config[
                        "continues_current_task"
                    ],
                    "handover_source": config["handover_source"],
                    "workdir": str(workdir_path),
                    "ros_setup": str(ros_setup_path),
                    "image_topic": image_topic,
                    "detections_topic": detections_topic,
                    "ready_timeout_sec": args.ready_timeout_sec,
                },
                ensure_ascii=False,
            )
        )
        return

    profile = subprocess.run(
        ["rosparam", "get", ("/" + namespace if namespace else "") + "/drone_control/runtime_profile"], env=runtime_env,
        text=True, capture_output=True, timeout=5,
    )
    if profile.returncode != 0 or profile.stdout.strip().strip("\"'") != "sim":
        fail("refusing YOLO launch: selected drone_control/runtime_profile is not sim")
    if not args.detach:
        running = find_running_backend(namespace)
        if running is not None:
            fail("target detection backend already running: pid=%s" % running[0])

    if args.detach:
        pid, pid_path, log_path, readiness = start_detached(
            cmd,
            workdir_path,
            args.pid_file,
            args.log_file,
            args.startup_wait_sec,
            runtime_env,
            [image_topic, detections_topic],
            detections_topic,
            args.ready_timeout_sec,
            args.ready_poll_sec,
            namespace,
        )
        print(
            json.dumps(
                {
                    "status": "STARTED",
                    "mode": args.mode,
                    "backend": str(backend_path),
                    "pid": pid,
                    "pid_file": str(pid_path),
                    "log_file": str(log_path),
                    "image_topic": image_topic,
                    "detections_topic": detections_topic,
                    "ready": True,
                    "readiness": readiness,
                },
                ensure_ascii=False,
            )
        )
        return

    proc = subprocess.run(
        cmd,
        cwd=str(workdir_path),
        env=runtime_env,
    )
    raise SystemExit(proc.returncode)


if __name__ == "__main__":
    main()
