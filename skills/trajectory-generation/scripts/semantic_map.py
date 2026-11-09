#!/usr/bin/env python3
"""Obstacle-aware trajectory queries backed by the active Gazebo SDF world."""

import argparse
import json
import math
import sys
from pathlib import Path
from xml.etree.ElementTree import ParseError

sys.path.insert(0, str(Path(__file__).resolve().parents[5]))
from uav_runtime.map_provider import create_map as SemanticMap, map_context


def fail(code, message):
    print(json.dumps({"status": code, "message": str(message)}, ensure_ascii=False))
    raise SystemExit(2 if code == "INVALID_ARGUMENT" else 1)


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        fail("INVALID_ARGUMENT", name + " must be a finite number")


def gps(map_data, lon, lat, altitude):
    finite(lon, "longitude"); finite(lat, "latitude")
    if not -180 <= lon <= 180 or not -90 <= lat <= 90:
        fail("INVALID_ARGUMENT", "longitude/latitude outside WGS84 range")
    return (*map_data.to_xy(lon, lat), altitude)


def waypoint(map_data, point):
    lon, lat = map_data.to_gps(point[0], point[1])
    return {"longitude": round(lon, 8), "latitude": round(lat, 8),
            "altitude": round(point[2], 3)}


def distance(points):
    return round(sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:])), 3)


def route(map_data, start, end, altitude, threshold):
    finite(threshold, "threshold")
    if threshold <= 0:
        fail("INVALID_ARGUMENT", "threshold must be > 0")
    points = map_data.route(start[:2], end[:2], altitude)
    # Preserve the old density control without replacing safe turning points.
    dense = [points[0]]
    for a, b in zip(points, points[1:]):
        segments = max(1, math.ceil(math.dist(a[:2], b[:2]) / threshold))
        dense.extend((a[0] + (b[0]-a[0])*i/segments,
                      a[1] + (b[1]-a[1])*i/segments, altitude)
                     for i in range(1, segments+1))
    go = [waypoint(map_data, p) for p in dense]
    # Only straight interpolation points allow a moving handover.  Geometric
    # corners require the normal arrival/low-speed check in both directions.
    for i in range(1, len(dense)-1):
        a, b, c = dense[i-1:i+2]
        ux, uy = b[0]-a[0], b[1]-a[1]
        vx, vy = c[0]-b[0], c[1]-b[1]
        scale = math.hypot(ux, uy)*math.hypot(vx, vy)
        if scale > 0 and abs(ux*vy-uy*vx) <= 1e-9*scale and ux*vx+uy*vy > 0:
            go[i]['pass_through'] = True
    return {"status": "OK", "go_waypoints": go,
            "back_waypoints": list(reversed(go)),
            "one_way_distance": distance(dense),
            "total_distance": round(distance(dense)*2, 3)}


def query_path(map_data, args):
    finite(args.flight_alt, "flight_alt")
    finite(args.end_alt, "end_alt")
    if not 0 < args.flight_alt <= map_data.max_altitude:
        fail("INVALID_ARGUMENT", f"flight_alt must be within (0, {map_data.max_altitude}]")
    if args.starts_json is None:
        if any(v is None for v in (args.start_lon, args.start_lat, args.start_alt)):
            fail("INVALID_ARGUMENT", "single query-path requires start-lon, start-lat and start-alt")
        starts = [(None, args.start_lon, args.start_lat, args.start_alt)]
    else:
        if any(v is not None for v in (args.start_lon, args.start_lat, args.start_alt)):
            fail("INVALID_ARGUMENT", "use either starts-json or single start arguments")
        try:
            raw = json.loads(args.starts_json)
        except json.JSONDecodeError as exc:
            fail("INVALID_ARGUMENT", "starts_json is not valid JSON: " + str(exc))
        if not isinstance(raw, list) or not raw:
            fail("INVALID_ARGUMENT", "starts_json must be a non-empty array")
        starts = []
        seen = set()
        for index, item in enumerate(raw):
            if not isinstance(item, dict) or set(item) != {"uav_id", "longitude", "latitude", "altitude"}:
                fail("INVALID_ARGUMENT", f"starts_json[{index}] requires uav_id, longitude, latitude, altitude only")
            uid = item["uav_id"]
            if not isinstance(uid, str) or not uid.strip() or uid in seen:
                fail("INVALID_ARGUMENT", f"starts_json[{index}].uav_id must be unique and non-empty")
            seen.add(uid)
            starts.append((uid, item["longitude"], item["latitude"], item["altitude"]))
    goal = gps(map_data, args.end_lon, args.end_lat, args.end_alt)
    results = []
    for uid, lon, lat, alt in starts:
        finite(alt, "start_alt")
        start = gps(map_data, lon, lat, alt)
        answer = route(map_data, start, goal, args.flight_alt, args.threshold)
        results.append({"uav_id": uid, **answer} if uid is not None else answer)
    return {"status": "OK", "results": results} if args.starts_json is not None else results[0]


def query_area(map_data, args):
    finite(args.flight_alt, "flight_alt"); finite(args.lane, "lane")
    finite(args.center_alt, "center_alt")
    if args.lane <= 0:
        fail("INVALID_ARGUMENT", "lane must be > 0")
    center = gps(map_data, args.center_lon, args.center_lat, args.center_alt)
    if not map_data.within_map(*center[:2]):
        raise ValueError("SEMANTIC_MAP_OUTSIDE_CAMPUS")
    if not 0 < args.flight_alt <= map_data.max_altitude:
        fail("INVALID_ARGUMENT", f"flight_alt must be within (0, {map_data.max_altitude}]")
    if args.shape == "rectangle":
        if args.length is None or args.width is None:
            fail("INVALID_ARGUMENT", "rectangle requires length and width")
        finite(args.length, "length"); finite(args.width, "width")
        if args.length <= 0 or args.width <= 0:
            fail("INVALID_ARGUMENT", "length and width must be > 0")
        x0, y0, x1, y1 = (center[0]-args.length/2, center[1]-args.width/2,
                          center[0]+args.length/2, center[1]+args.width/2)
        in_shape = lambda x,y: True
        on_boundary = lambda x,y: min(x-x0,x1-x,y-y0,y1-y) < 2.6
    else:
        if args.radius is None:
            fail("INVALID_ARGUMENT", "circle requires radius")
        finite(args.radius, "radius")
        if args.radius <= 0:
            fail("INVALID_ARGUMENT", "radius must be > 0")
        x0, y0, x1, y1 = (center[0]-args.radius, center[1]-args.radius,
                          center[0]+args.radius, center[1]+args.radius)
        in_shape = lambda x,y: math.hypot(x-center[0],y-center[1]) <= args.radius
        on_boundary = lambda x,y: math.hypot(x-center[0],y-center[1]) >= args.radius-2.6
    # The caller has resolved user overrides/defaults already. Building lookup
    # labels the estimate; it must not silently replace or clip that footprint.
    region = next((p for p in map_data.places if p["category"] == "building"
                   and p["bounds_xy"][0] <= center[0] <= p["bounds_xy"][2]
                   and p["bounds_xy"][1] <= center[1] <= p["bounds_xy"][3]), None)
    if not all(map_data.within_map(x,y) for x,y in ((x0,y0),(x0,y1),(x1,y0),(x1,y1))):
        raise ValueError("SEMANTIC_MAP_AREA_OUTSIDE_MAP")
    start_lon, start_lat = getattr(args,'start_lon',None), getattr(args,'start_lat',None)
    if (start_lon is None) != (start_lat is None):
        fail("INVALID_ARGUMENT", "start-lon and start-lat must be provided together")
    entry = gps(map_data,start_lon,start_lat,args.flight_alt) if start_lon is not None else (center[0],center[1],args.flight_alt)
    if not (x0 <= entry[0] <= x1 and y0 <= entry[1] <= y1 and in_shape(*entry[:2])):
        raise ValueError("SEMANTIC_MAP_AREA_START_OUTSIDE_REGION")
    if x0 >= x1 or y0 >= y1:
        raise ValueError("SEMANTIC_MAP_AREA_EMPTY")
    if args.mode == "boundary":
        # A lap follows the complete perimeter in order. Never silently remove
        # occupied/unreachable portions or replace them with an interior tour.
        if args.shape == "rectangle":
            corners = [(x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)]
        else:
            count = max(16, math.ceil(2*math.pi*args.radius/min(args.lane, 2.0)))
            if count > 500:
                fail("INVALID_ARGUMENT", "perimeter too large")
            corners = [(center[0]+args.radius*math.cos(2*math.pi*i/count),
                        center[1]+args.radius*math.sin(2*math.pi*i/count)) for i in range(count+1)]
            corners[-1] = corners[0]
        perimeter = [(*corners[0],args.flight_alt)]
        for a,b in zip(corners,corners[1:]):
            if not map_data.planner.segment_clear(a,b,args.flight_alt,args.flight_alt):
                raise ValueError("SEMANTIC_MAP_BOUNDARY_BLOCKED: complete perimeter is not traversable")
            count = max(1, math.ceil(math.dist(a,b)/min(args.lane,2.0)))
            if len(perimeter)+count > 1000:
                fail("INVALID_ARGUMENT", "perimeter too large")
            perimeter.extend((a[0]+(b[0]-a[0])*i/count,
                              a[1]+(b[1]-a[1])*i/count,args.flight_alt) for i in range(1,count+1))
        perimeter[-1] = perimeter[0]
        approach = map_data.route(entry[:2],perimeter[0][:2],args.flight_alt)
        # Transit is separate: repeating waypoints repeats only the closed lap.
        return {"status":"OK", "waypoints":[waypoint(map_data,p) for p in perimeter],
                "approach_waypoints":[waypoint(map_data,p) for p in approach],
                "approach_length":distance(approach), "path_length":distance(perimeter),
                "closed":True, "boundary_complete":True, "coverage_certified":False,
                "shape":args.shape, "bounds_xy":[x0,y0,x1,y1],
                "region":region["name"] if region else "公共区域",
                "area_m2":args.length*args.width if args.shape=='rectangle' else math.pi*args.radius**2}
    # Bound the work and keep enough samples to reflect a room-sized maze.
    step = min(5.0, max(2.0, args.lane))
    nx, ny = max(1, math.ceil((x1-x0)/step)), max(1, math.ceil((y1-y0)/step))
    if nx*ny > 500:
        fail("INVALID_ARGUMENT", "area too large for semantic coverage query")
    candidates = []
    for j in range(ny+1):
        y = y0 + (y1-y0)*j/ny
        xs = range(nx+1) if j%2 == 0 else range(nx,-1,-1)
        for i in xs:
            x = x0 + (x1-x0)*i/nx
            if not in_shape(x,y) or (args.mode == "boundary" and not on_boundary(x,y)):
                continue
            if map_data.within_map(x,y) and not map_data.planner.blocked((x,y),args.flight_alt,args.flight_alt):
                candidates.append((x,y,args.flight_alt))
    if map_data.planner.blocked(entry[:2],args.flight_alt,args.flight_alt):
        raise ValueError("SEMANTIC_MAP_AREA_START_OCCUPIED" if start_lon is not None else "SEMANTIC_MAP_AREA_CENTER_OCCUPIED")
    if not candidates:
        raise ValueError("SEMANTIC_MAP_AREA_NO_FREE_CELLS")
    points = [entry]
    reached = 0
    for target in candidates:
        if math.dist(points[-1][:2],target[:2]) < .1:
            continue
        try:
            points.extend(map_data.planner.plan(points[-1], target))
            reached += 1
        except ValueError as exc:
            if str(exc) != "GLOBAL_ROUTE_UNREACHABLE":
                raise
    if len(points) < 2:
        raise ValueError("SEMANTIC_MAP_AREA_NO_REACHABLE_CELLS")
    return {"status": "OK", "waypoints": [waypoint(map_data,p) for p in points],
            "path_length": distance(points), "region": region["name"] if region else "校园公共区域",
            "reachable_samples": reached, "candidate_samples": len(candidates),
            "shape": args.shape, "bounds_xy": [x0,y0,x1,y1],
            "area_m2": args.length*args.width if args.shape == 'rectangle' else math.pi*args.radius**2}


def main():
    parser = argparse.ArgumentParser(description="Gazebo semantic map and safe static routing")
    parser.add_argument("--world", type=Path, help="Gazebo SDF world; defaults to active UAV scene")
    sub = parser.add_subparsers(dest="operation", required=True)
    sub.add_parser("query-map")
    p = sub.add_parser("query-path")
    p.add_argument("--start-lon", type=float); p.add_argument("--start-lat", type=float)
    p.add_argument("--start-alt", type=float); p.add_argument("--starts-json")
    p.add_argument("--end-lon", type=float, required=True); p.add_argument("--end-lat", type=float, required=True)
    p.add_argument("--end-alt", type=float, required=True); p.add_argument("--flight-alt", type=float, required=True)
    p.add_argument("--threshold", type=float, default=5000.0)
    a = sub.add_parser("query-area")
    a.add_argument("--center-lon", type=float, required=True); a.add_argument("--center-lat", type=float, required=True)
    a.add_argument("--center-alt", type=float, required=True); a.add_argument("--flight-alt", type=float, required=True)
    a.add_argument("--shape", choices=("rectangle","circle"), required=True)
    a.add_argument("--mode", choices=("boundary","coverage"), required=True)
    a.add_argument("--length", type=float); a.add_argument("--width", type=float)
    a.add_argument("--radius", type=float); a.add_argument("--lane", type=float, default=50.0)
    a.add_argument("--start-lon",type=float); a.add_argument("--start-lat",type=float)
    args = parser.parse_args()
    try:
        map_data = SemanticMap(args.world)
        result = (map_data.describe() if args.operation == "query-map" else
                  query_path(map_data,args) if args.operation == "query-path" else query_area(map_data,args))
        result["map_context"] = map_context(map_data)
        print(json.dumps(result, ensure_ascii=False))
    except SystemExit:
        raise
    except (OSError, ValueError, ParseError) as exc:
        fail("ERROR", exc)

if __name__ == "__main__":
    main()
