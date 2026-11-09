#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""将任务区域转换为带绝对 WGS84 位置的 FUEL 地理搜索框。"""

import argparse
import json
import math
from typing import Tuple

from calc_offset import vincenty_direct

MIN_Z = 0.3
# FUEL start_box_margin is 0.2 m; leave 0.5 m above the mission altitude.
MAX_Z_MARGIN = 0.5


def emit(status: str, message: str = "", code: int = 0, **extra) -> None:
    result = {"status": status}
    if message:
        result["message"] = message
    result.update(extra)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(code)


def offset(
    lon: float, lat: float, bearing: float, distance: float
) -> Tuple[float, float]:
    """沿 WGS84 大地线偏移；零距离原样返回。"""
    if distance == 0:
        return round(lon, 6), round(lat, 6)
    return vincenty_direct(lon, lat, bearing, distance)


def main() -> None:
    ap = argparse.ArgumentParser(description="构造 FUEL WGS84 地理搜索框")
    ap.add_argument("--center-lon", type=float)
    ap.add_argument("--center-lat", type=float)
    ap.add_argument("--shape", choices=["circle", "rectangle"])
    ap.add_argument("--radius", type=float)
    ap.add_argument("--length", type=float)
    ap.add_argument("--width", type=float)
    ap.add_argument("--flight-alt", type=float)
    args = ap.parse_args()

    missing = []
    for name in ("center_lon", "center_lat", "shape", "flight_alt"):
        if getattr(args, name) is None:
            missing.append(name)
    if args.shape == "circle" and args.radius is None:
        missing.append("radius")
    if args.shape == "rectangle":
        if args.length is None:
            missing.append("length")
        if args.width is None:
            missing.append("width")
    if missing:
        emit("MISSING_PARAMETER", f"缺少参数: {', '.join(missing)}", 2, missing=missing)

    numeric_values = [args.center_lon, args.center_lat, args.flight_alt]
    numeric_values.extend(
        value for value in (args.radius, args.length, args.width) if value is not None
    )
    if not all(math.isfinite(value) for value in numeric_values):
        emit("INVALID_ARGUMENT", "所有数值参数必须为有限数", 2)
    if not -180 <= args.center_lon <= 180:
        emit("INVALID_ARGUMENT", "center_lon 必须在 [-180, 180] 范围内", 2)
    if not -90 <= args.center_lat <= 90:
        emit("INVALID_ARGUMENT", "center_lat 必须在 [-90, 90] 范围内", 2)
    if args.flight_alt <= MIN_Z:
        emit("INVALID_ARGUMENT", f"flight_alt 必须 > {MIN_Z}", 2)

    if args.shape == "circle":
        if args.radius <= 0:
            emit("INVALID_ARGUMENT", "radius 必须 > 0", 2)
        half_east_west = args.radius
        half_north_south = args.radius
        dimensions = {"radius_m": args.radius}
    else:
        if args.length <= 0:
            emit("INVALID_ARGUMENT", "length 必须 > 0", 2)
        if args.width <= 0:
            emit("INVALID_ARGUMENT", "width 必须 > 0", 2)
        half_east_west = args.length / 2.0
        half_north_south = args.width / 2.0
        dimensions = {"length_m": args.length, "width_m": args.width}

    west_lon, _ = offset(args.center_lon, args.center_lat, 270.0, half_east_west)
    east_lon, _ = offset(args.center_lon, args.center_lat, 90.0, half_east_west)
    _, south_lat = offset(args.center_lon, args.center_lat, 180.0, half_north_south)
    _, north_lat = offset(args.center_lon, args.center_lat, 0.0, half_north_south)

    emit(
        "OK",
        coordinate_system="wgs84",
        shape=args.shape,
        center=[round(args.center_lon, 6), round(args.center_lat, 6)],
        box_min=[west_lon, south_lat, MIN_Z],
        box_max=[east_lon, north_lat, round(args.flight_alt + MAX_Z_MARGIN, 3)],
        flight_alt_m=round(args.flight_alt, 3),
        **dimensions,
    )


if __name__ == "__main__":
    main()
