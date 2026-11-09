#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
相对位置解算：
基准坐标 + 方位角 + 距离 -> 目标中心坐标

使用 WGS84 Vincenty Direct。
脚本不解析地名或自然语言方向。
"""

import argparse
import json
import math
import sys
from math import atan2, cos, degrees, pi, radians, sin, sqrt, tan
from typing import Tuple

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_B = WGS84_A * (1 - WGS84_F)


def emit(status: str, message: str = "", code: int = 0, **extra) -> None:
    result = {"status": status}
    if message:
        result["message"] = message
    result.update(extra)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(code)


def vincenty_direct(
    lon: float,
    lat: float,
    bearing_deg: float,
    distance_m: float,
) -> Tuple[float, float]:
    """WGS84 大地线正解：给定起点、方位角、距离求终点经纬度。"""

    phi1 = radians(lat)
    lambda1 = radians(lon)
    alpha1 = radians(bearing_deg)

    tan_u1 = (1 - WGS84_F) * tan(phi1)
    cos_u1 = 1 / sqrt(1 + tan_u1 ** 2)
    sin_u1 = tan_u1 * cos_u1

    sigma1 = atan2(tan_u1, cos(alpha1))
    sin_alpha = cos_u1 * sin(alpha1)
    cos_sq_alpha = 1 - sin_alpha ** 2
    u_sq = cos_sq_alpha * (WGS84_A ** 2 - WGS84_B ** 2) / WGS84_B ** 2

    a = 1 + u_sq / 16384 * (
        4096 + u_sq * (-768 + u_sq * (320 - 175 * u_sq))
    )
    b = u_sq / 1024 * (
        256 + u_sq * (-128 + u_sq * (74 - 47 * u_sq))
    )

    sigma = distance_m / (WGS84_B * a)

    converged = False
    for _ in range(200):
        cos_2sm = cos(2 * sigma1 + sigma)
        sin_s, cos_s = sin(sigma), cos(sigma)
        delta = (
            b
            * sin_s
            * (
                cos_2sm
                + b / 4
                * (
                    cos_s * (-1 + 2 * cos_2sm ** 2)
                    - b / 6
                    * cos_2sm
                    * (-3 + 4 * sin_s ** 2)
                    * (-3 + 4 * cos_2sm ** 2)
                )
            )
        )
        sigma_new = distance_m / (WGS84_B * a) + delta
        if abs(sigma_new - sigma) < 1e-12:
            sigma = sigma_new
            converged = True
            break
        sigma = sigma_new

    if not converged:
        raise RuntimeError("Vincenty 迭代未收敛")

    sin_s, cos_s = sin(sigma), cos(sigma)
    cos_2sm = cos(2 * sigma1 + sigma)
    tmp = sin_u1 * sin_s - cos_u1 * cos_s * cos(alpha1)

    phi2 = atan2(
        sin_u1 * cos_s + cos_u1 * sin_s * cos(alpha1),
        (1 - WGS84_F) * sqrt(sin_alpha ** 2 + tmp ** 2),
    )

    lambda_delta = atan2(
        sin_s * sin(alpha1),
        cos_u1 * cos_s - sin_u1 * sin_s * cos(alpha1),
    )

    c = WGS84_F / 16 * cos_sq_alpha * (
        4 + WGS84_F * (4 - 3 * cos_sq_alpha)
    )

    L = lambda_delta - (1 - c) * WGS84_F * sin_alpha * (
        sigma + c * sin_s * (
            cos_2sm + c * cos_s * (-1 + 2 * cos_2sm ** 2)
        )
    )

    lambda2 = (lambda1 + L + 3 * pi) % (2 * pi) - pi

    return round(degrees(lambda2), 6), round(degrees(phi2), 6)


def main() -> None:
    ap = argparse.ArgumentParser(description="相对位置解算（WGS84 大地线正解）")
    ap.add_argument("--lon", type=float)
    ap.add_argument("--lat", type=float)
    ap.add_argument("--alt", type=float, default=0.0)
    ap.add_argument("--bearing", type=float)
    ap.add_argument("--distance", type=float)
    args = ap.parse_args()

    missing = []
    for name in ("lon", "lat", "bearing", "distance"):
        if getattr(args, name) is None:
            missing.append(name)

    if missing:
        emit(
            "MISSING_PARAMETER",
            f"缺少参数: {', '.join(missing)}",
            2,
            missing=missing,
        )

    values = [args.lon, args.lat, args.alt, args.bearing, args.distance]
    if not all(math.isfinite(v) for v in values):
        emit("INVALID_ARGUMENT", "所有数值参数必须为有限数", 2)

    if not -180 <= args.lon <= 180:
        emit("INVALID_ARGUMENT", "lon 必须在 [-180, 180] 范围内", 2)

    if not -90 <= args.lat <= 90:
        emit("INVALID_ARGUMENT", "lat 必须在 [-90, 90] 范围内", 2)

    if args.distance <= 0:
        emit("INVALID_ARGUMENT", "distance 必须 > 0", 2)

    bearing = args.bearing % 360.0

    try:
        target_lon, target_lat = vincenty_direct(
            args.lon,
            args.lat,
            bearing,
            args.distance,
        )
    except Exception as exc:
        emit("ERROR", str(exc), 3)

    emit(
        "OK",
        code=0,
        base_lon=args.lon,
        base_lat=args.lat,
        base_alt=args.alt,
        bearing_deg=bearing,
        distance_m=args.distance,
        center_lon=target_lon,
        center_lat=target_lat,
        center_alt=args.alt,
    )


if __name__ == "__main__":
    main()
