#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import math
import os
import sys
import time

import yaml


RUNNER_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKSPACE_ROOT = RUNNER_ROOT


def runner_path(path):
    if not path:
        return path
    path = os.path.expanduser(os.path.expandvars(path))
    if os.path.isabs(path):
        return path
    return os.path.join(RUNNER_ROOT, path)


def load_yaml(path, default=None):
    if not os.path.exists(path):
        return default
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return default if data is None else data


def deep_merge_dict(base, override):
    out = dict(base or {})
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge_dict(out[key], value)
        else:
            out[key] = value
    return out


def load_runtime_config(config_path=None):
    cfg = load_yaml(config_path or runner_path("config/runtime.yaml"), default={})
    local_cfg = load_yaml(runner_path("config/local_override.yaml"), default={})
    cfg = deep_merge_dict(cfg, local_cfg)
    paths = cfg.get("paths", {})
    for key, value in list(paths.items()):
        if not isinstance(value, str):
            continue
        value = os.path.expanduser(os.path.expandvars(value))
        if not os.path.isabs(value):
            value = os.path.join(RUNNER_ROOT, value)
        paths[key] = os.path.abspath(value)
    return cfg


def add_grpc_paths(grpc_src):
    grpc_src = os.path.abspath(grpc_src)
    paths = [
        grpc_src,
        os.path.join(grpc_src, "api"),
        os.path.join(grpc_src, "proto"),
    ]
    for path in paths:
        if path not in sys.path:
            sys.path.insert(0, path)


def load_current_route():
    path = runner_path("runtime/current_route.json")
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_saved_ego_positions():
    return load_yaml(runner_path("config/saved_ego_positions.yaml"), default={})


def find_saved_pose(saved_positions, pose_name):
    for pose in saved_positions.get("poses", []) or []:
        if str(pose.get("name", "")) == str(pose_name):
            return pose
    return None


def pose_from_saved_pose(saved_pose):
    return (
        float(saved_pose["x"]),
        float(saved_pose["y"]),
        float(saved_pose["z"]),
        float(saved_pose.get("yaw_deg", 0.0)),
    )


def link_from_saved_pose(saved_pose):
    for key in ("current_link", "candidate_link", "nearest_link"):
        link_id = saved_pose.get(key)
        if link_id:
            return str(link_id)
    return ""


def parse_route_links(raw, current_route):
    if raw:
        links = []
        for chunk in raw:
            links.extend(part.strip() for part in chunk.split(",") if part.strip())
        return links
    return list(current_route.get("route_links") or [])


def route_default_pose(current_route):
    points = list(current_route.get("route_points") or [])
    if len(points) >= 2:
        idx = min(5, len(points) - 2)
        p0 = points[idx]
        p1 = points[idx + 1]
        yaw = math.degrees(math.atan2(float(p1[1]) - float(p0[1]), float(p1[0]) - float(p0[0])))
        return float(p0[0]), float(p0[1]), float(p0[2]), yaw

    start_xy = current_route.get("start_xy")
    if start_xy and len(start_xy) >= 3:
        return float(start_xy[0]), float(start_xy[1]), float(start_xy[2]), 0.0

    return None


def yaw_deg_from_state(state):
    return float(getattr(state.transform.rotation, "z", 0.0))


def pose_from_state(state):
    loc = state.transform.location
    return float(loc.x), float(loc.y), float(loc.z), yaw_deg_from_state(state)


def pose_ahead_of(pose, ahead_m):
    x, y, z, yaw_deg = pose
    yaw_rad = math.radians(yaw_deg)
    return (
        x + math.cos(yaw_rad) * float(ahead_m),
        y + math.sin(yaw_rad) * float(ahead_m),
        z,
        yaw_deg,
    )


def current_link_from_state(state):
    try:
        link_id = state.vehicle_state.current_link_info.id.value
    except Exception:
        link_id = ""
    return str(link_id) if link_id else ""


def speed_kmh_from_state(state):
    gv = getattr(state, "global_velocity", None)
    lv = getattr(state, "velocity", None)
    vec = gv if gv is not None and any(abs(getattr(gv, axis, 0.0)) > 1e-6 for axis in ("x", "y", "z")) else lv
    if vec is None:
        return 0.0
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z) * 3.6


def actor_position(state):
    loc = state.transform.location
    return loc.x, loc.y, loc.z


def result_ok(result, success_code):
    return result is not None and getattr(result, "status", None) == success_code


def result_summary(result):
    if result is None:
        return "None"
    return (
        f"status={getattr(result, 'status', None)} "
        f"description={getattr(result, 'description', '')!r} "
        f"custom_message={getattr(result, 'custom_message', '')!r}"
    )


def print_network_check():
    print("[CHECK] Now run: rostopic echo -n 1 /Ego_topic")
    print("[CHECK] Check MORAI F4 Ego Network Vehicle List")


def log_stage_boundary(stage_name, phase):
    print(f"[STEP] {stage_name} ({phase})")
    print_network_check()


def log_stop_after(option_name):
    print(f"[STOP] {option_name} requested; exiting before next step.")
    print_network_check()


def log_create_vehicle_spawn_point_response(result, request_object_info):
    print(f"[create-response-repr] {result!r}")
    print(f"[create-response-text] {result}")
    print(f"[create-response-status] {getattr(result, 'status', None)}")
    print(f"[create-response-description] {getattr(result, 'description', '')!r}")
    print(f"[create-response-custom-message] {getattr(result, 'custom_message', '')!r}")
    print(
        "[create-request-object] "
        f"id={request_object_info.id.value!r} "
        f"type={request_object_info.object_type} "
        f"client_key={request_object_info.client_key!r}"
    )


def spawnpoint_info_from_create_result(result, request_object_info):
    info = make_object_info(
        request_object_info.id.value,
        request_object_info.object_type,
        request_object_info.client_key,
    )
    custom_message = getattr(result, "custom_message", "") if result is not None else ""
    if custom_message:
        info.id.value = str(custom_message)
    return info


def get_vehicle_states(adapter, client_key):
    from proto.morai.actor.actor_get_pb2 import GetAllActorsFilter

    states = {}
    for key in (client_key, ""):
        param = GetAllActorsFilter()
        param.client_key = key
        param.vehicle = True
        param.pedestrian = False
        param.obstacle = False
        response = adapter.get_all_actors_state(param)
        if response is None:
            continue
        for state in getattr(response, "states", []) or []:
            actor_id = getattr(getattr(state.actor_info, "id", None), "value", "")
            if actor_id:
                states[actor_id] = state
    return states


def get_vehicle_state_by_id(adapter, client_key, actor_id):
    entry = get_vehicle_state_entry_by_id(adapter, client_key, actor_id)
    return entry[1] if entry is not None else None


def get_vehicle_state_entry_by_id(adapter, client_key, actor_id):
    try:
        from proto.morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE

        object_info = make_object_info(actor_id, OBJECT_TYPE_VEHICLE, client_key)
        state = adapter.get_actor_state(object_info)
        if state is not None and hasattr(state, "transform"):
            returned_id = getattr(getattr(state.actor_info, "id", None), "value", "") or str(actor_id)
            return returned_id, state
    except Exception as exc:
        print(f"[ego-state] direct GetActorState failed actor_id={actor_id!r}: {exc}")

    states = get_vehicle_states(adapter, client_key)
    if actor_id in states:
        return actor_id, states[actor_id]
    lowered = str(actor_id).lower()
    for candidate_id, state in states.items():
        if str(candidate_id).lower() == lowered:
            return candidate_id, state
    return None


def route_suffix_from_link(route_links, link_id):
    if link_id and link_id in route_links:
        return list(route_links[route_links.index(link_id) :])
    return []


def resolve_pose_and_route(args, adapter, client_key, current_route):
    manual_route_links = parse_route_links(args.route_links, {}) if args.route_links else []
    current_route_links = list(current_route.get("route_links") or [])
    ego_state = None
    ego_actor_id = ""
    ego_link = ""

    if args.use_current_ego_pose:
        ego_entry = get_vehicle_state_entry_by_id(adapter, client_key, args.ego_id)
        if ego_entry is None:
            raise SystemExit(f"No Ego actor state found for --ego-id {args.ego_id!r}.")
        ego_actor_id, ego_state = ego_entry
        ego_pose = pose_from_state(ego_state)
        pose = pose_ahead_of(ego_pose, args.spawn_ahead_m)
        pose_source = "current_ego"
        ego_link = current_link_from_state(ego_state)
    elif args.saved_pose_name:
        saved_positions = load_saved_ego_positions()
        saved_pose = find_saved_pose(saved_positions, args.saved_pose_name)
        if saved_pose is None:
            raise SystemExit(f"No saved pose named {args.saved_pose_name!r} in config/saved_ego_positions.yaml.")
        pose = pose_from_saved_pose(saved_pose)
        pose_source = "saved_ego_positions.yaml"
        saved_link = link_from_saved_pose(saved_pose)
    elif args.x is not None and args.y is not None:
        pose = (args.x, args.y, args.z if args.z is not None else 0.0, 0.0)
        pose_source = "manual"
    else:
        pose = route_default_pose(current_route)
        pose_source = "current_route"

    if pose is None:
        raise SystemExit(
            "No pose available. Pass --x --y [--z], use --use-current-ego-pose, "
            "or create runtime/current_route.json first."
        )

    if manual_route_links:
        route_links = manual_route_links
        route_source = "manual"
    elif args.saved_pose_name:
        route_links = [saved_link] if saved_link else []
        route_source = "saved_pose_link"
    elif args.use_current_ego_pose:
        route_links = [ego_link] if ego_link else []
        route_source = "current_ego_link"
    else:
        route_links = current_route_links
        route_source = "current_route"

    return pose, pose_source, route_links, route_source, ego_link, ego_actor_id, ego_state


def nearest_state(states, x, y, candidate_ids=None):
    best = None
    best_dist = float("inf")
    ids = candidate_ids if candidate_ids is not None else states.keys()
    for actor_id in ids:
        state = states.get(actor_id)
        if state is None:
            continue
        px, py, _ = actor_position(state)
        dist = math.hypot(px - x, py - y)
        if dist < best_dist:
            best = (actor_id, state, dist)
            best_dist = dist
    return best


def wait_for_spawned_actor(adapter, client_key, before_ids, x, y, timeout_sec, poll_sec):
    deadline = time.time() + timeout_sec
    before_ids = set(before_ids)
    while time.time() < deadline:
        states = get_vehicle_states(adapter, client_key)
        new_ids = sorted(set(states) - before_ids)
        best = nearest_state(states, x, y, candidate_ids=new_ids)
        if best is not None:
            return best[0], best[1]
        time.sleep(poll_sec)
    return None, None


def select_model(adapter, requested_model):
    if requested_model:
        return requested_model

    try:
        from proto.morai.simulator.category_obstacles_pb2 import CategoryObstacles

        param = CategoryObstacles()
        param.vehicle = True
        objects = adapter._simulator_stub.GetAvailableObject(param)
        models = list(getattr(objects, "surround_vehicle", []) or [])
        if models:
            print(f"[model] auto-selected surround vehicle model: {models[0]}")
            return models[0]
    except Exception as exc:
        print(f"[model] auto-select failed: {exc}")

    return "2018_Hyundai_Sonata"


def build_spawn_point_param(args, model_name, field_speed, route_links, pose, client_key):
    from proto.morai.common.enum_pb2 import OBJECT_TYPE_SPAWN_POINT
    from proto.morai.scenario.scenario_enum_pb2 import (
        LAT_BIAS_FIX,
        LENGTH_TYPE_MIDDLE_SIZE,
        VEHICLE_DRIVING_PARAM_TYPE_CONSTANT,
        VELOCITY_TYPE_CUSTOMVELOCITY,
    )
    from proto.morai.scenario.spawn_point_pb2 import CreateVehicleSpawnPointParam

    x, y, z, _ = pose
    param = CreateVehicleSpawnPointParam()
    info = param.spawn_point_info
    info.object_info.id.value = args.spawnpoint_id
    info.object_info.object_type = OBJECT_TYPE_SPAWN_POINT
    info.object_info.client_key = client_key
    info.location.x = x
    info.location.y = y
    info.location.z = z
    info.model_name = model_name
    info.pause = False

    cfg = param.config
    cfg.list_vehicle_length_type.append(LENGTH_TYPE_MIDDLE_SIZE)
    cfg.is_close_loop = False
    cfg.is_lane_change = False
    cfg.parameter_type = VEHICLE_DRIVING_PARAM_TYPE_CONSTANT
    cfg.maximum_spawn_vehicle = 1
    cfg.min_spawn_period = float(args.spawn_period_sec)
    cfg.max_spawn_period = float(args.spawn_period_sec)
    cfg.spawn_velocity_type = VELOCITY_TYPE_CUSTOMVELOCITY
    cfg.min_spawn_velocity_custom = float(args.initial_speed_kmh if args.field_unit == "kmh" else args.initial_speed_kmh / 3.6)
    cfg.max_spawn_velocity_custom = float(args.initial_speed_kmh if args.field_unit == "kmh" else args.initial_speed_kmh / 3.6)
    cfg.desired_velocity_type = VELOCITY_TYPE_CUSTOMVELOCITY
    cfg.min_desired_velocity_custom = float(field_speed)
    cfg.max_desired_velocity_custom = float(field_speed)
    cfg.lateral_bias_mode = LAT_BIAS_FIX
    cfg.min_lateral_bias = 0.0
    cfg.max_lateral_bias = 0.0

    if route_links:
        cfg.start_link_info.id.value = str(route_links[0])
        cfg.start_link_info.waypoint_idx = 0
        cfg.end_link_info.id.value = str(route_links[-1])
        cfg.end_link_info.waypoint_idx = 0

    return param


def enable_spawn_point(adapter, spawn_object_info, active, success_code):
    from proto.morai.scenario.spawn_point_pb2 import EnableSpawnPointParam

    print(
        "[spawnpoint-enable-target] "
        f"id={spawn_object_info.id.value!r} "
        f"type={spawn_object_info.object_type} "
        f"active={bool(active)}"
    )
    if not is_confirmed_vehicle_spawn_point(adapter, spawn_object_info):
        print("[WARN] skip EnableSpawnPoint: target is not confirmed as VehicleSpawnPoint")
        return False

    param = EnableSpawnPointParam()
    param.object_info.CopyFrom(spawn_object_info)
    param.is_active = bool(active)
    result = adapter.enable_spawn_point(param)
    print(f"[spawnpoint] enable={active}: {result_summary(result)}")
    return result_ok(result, success_code)


def set_route(adapter, actor_info, route_links, decision_range, success_code):
    from proto.morai.actor.actor_set_pb2 import VehicleRoute

    if not route_links:
        print("[route] skipped: no route links supplied")
        return False

    param = VehicleRoute()
    param.actor_info.CopyFrom(actor_info)
    param.decision_range = float(decision_range)
    for link_id in route_links:
        link = param.links.add()
        link.id.value = str(link_id)
        link.waypoint_idx = 0

    result = adapter.set_vehicle_route(param)
    print(f"[route] SetVehicleRoute: {result_summary(result)} links={route_links}")
    return result_ok(result, success_code)


def describe_object(adapter, object_info):
    object_id = getattr(getattr(object_info, "id", None), "value", "")
    object_type = getattr(object_info, "object_type", "")
    option_name = ""
    try:
        result = adapter.get_option_name(object_info)
        option_name = getattr(result, "value", "") if result is not None else ""
    except Exception as exc:
        option_name = f"unavailable:{exc}"
    return f"id={object_id} type={object_type} option_name={option_name!r}"


def object_option_name(adapter, object_info):
    try:
        result = adapter.get_option_name(object_info)
        return getattr(result, "value", "") if result is not None else ""
    except Exception:
        return ""


def is_confirmed_vehicle_spawn_point(adapter, object_info):
    from proto.morai.common.enum_pb2 import OBJECT_TYPE_SPAWN_POINT

    object_id = getattr(getattr(object_info, "id", None), "value", "")
    object_type = getattr(object_info, "object_type", None)
    option_name = object_option_name(adapter, object_info)
    print(
        "[spawnpoint-target-check] "
        f"id={object_id!r} type={object_type} option_name={option_name!r}"
    )
    if object_type != OBJECT_TYPE_SPAWN_POINT:
        return False
    if option_name == "DestinationPoint":
        return False
    return "VehicleSpawnPoint" in option_name or option_name == "SpawnPoint"


def actor_id_matches_ego(actor_id, ego_actor_id):
    return bool(actor_id) and bool(ego_actor_id) and str(actor_id) == str(ego_actor_id)


def destroy_object(adapter, object_info, label, success_code, ego_actor_id=""):
    object_id = getattr(getattr(object_info, "id", None), "value", "")
    if actor_id_matches_ego(object_id, ego_actor_id):
        print(
            "[FATAL] refusing to destroy actor_id that matches Ego "
            f"actor_id={object_id} label={label}"
        )
        return False
    print(f"[cleanup] destroy target {label}: {describe_object(adapter, object_info)}")
    result = adapter.destroy_actor(object_info)
    print(f"[cleanup] destroy {label}: {result_summary(result)}")
    return result_ok(result, success_code)


def make_object_info(object_id, object_type, client_key):
    from proto.morai.common.object_info_pb2 import ObjectInfo

    info = ObjectInfo()
    info.id.value = str(object_id)
    info.object_type = object_type
    info.client_key = client_key
    return info


def cleanup_test_objects(adapter, spawnpoint_id, actor_id, client_key, success_code, ego_actor_id=""):
    from proto.morai.common.enum_pb2 import OBJECT_TYPE_SPAWN_POINT, OBJECT_TYPE_VEHICLE

    spawn_info = make_object_info(spawnpoint_id, OBJECT_TYPE_SPAWN_POINT, client_key)
    if is_confirmed_vehicle_spawn_point(adapter, spawn_info):
        try:
            enable_spawn_point(adapter, spawn_info, False, success_code)
        except Exception as exc:
            print(f"[cleanup] disable spawn point failed: {exc}")
        destroy_object(adapter, spawn_info, f"spawn_point:{spawnpoint_id}", success_code, ego_actor_id=ego_actor_id)
    else:
        print("[WARN] cleanup skipped spawn point destroy: target is not confirmed as VehicleSpawnPoint")

    if actor_id:
        if actor_id_matches_ego(actor_id, ego_actor_id):
            print(
                "[FATAL] cleanup actor_id matches ego actor_id; "
                "aborting to prevent destroying Ego."
            )
            return
        actor_info = make_object_info(actor_id, OBJECT_TYPE_VEHICLE, client_key)
        destroy_object(adapter, actor_info, f"actor:{actor_id}", success_code, ego_actor_id=ego_actor_id)
    else:
        print("[cleanup] no --cleanup-actor-id supplied; only spawn point cleanup was attempted")


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Create one MORAI vehicle spawn point with custom desired velocity and log spawned NPC speed. "
            "Recommended first pass: --use-current-ego-pose --dry-run-pose to inspect the computed spawn pose."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--config", default=runner_path("config/runtime.yaml"))
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--client-key")
    parser.add_argument("--grpc-src")
    parser.add_argument("--model", default="")
    parser.add_argument("--spawnpoint-id", default="AIM_TEST_DESIRED_SPEED_SP")
    parser.add_argument("--ego-id", default="Ego", help="MORAI Ego actor id used by --use-current-ego-pose.")
    parser.add_argument("--saved-pose-name", default="")
    parser.add_argument(
        "--use-current-ego-pose",
        action="store_true",
        help="Read the live MORAI Ego pose via gRPC and use it as the spawn reference.",
    )
    parser.add_argument(
        "--spawn-ahead-m",
        type=float,
        default=10.0,
        help="Longitudinal offset from Ego yaw. Negative values spawn behind Ego.",
    )
    parser.add_argument(
        "--dry-run-pose",
        action="store_true",
        help="Only print Ego pose, spawn pose, and route links; do not create a MORAI spawn point.",
    )
    parser.add_argument("--x", type=float)
    parser.add_argument("--y", type=float)
    parser.add_argument("--z", type=float)
    parser.add_argument("--desired-speed-kmh", type=float, default=12.0)
    parser.add_argument("--initial-speed-kmh", type=float, default=0.0)
    parser.add_argument("--field-unit", choices=("kmh", "mps"), default="kmh")
    parser.add_argument(
        "--route-links",
        action="append",
        default=[],
        help="Comma-separated links. Overrides saved pose/current Ego route links when provided.",
    )
    parser.add_argument("--decision-range", type=float, default=30.0)
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--log-interval-sec", type=float, default=1.0)
    parser.add_argument("--spawn-timeout-sec", type=float, default=10.0)
    parser.add_argument("--spawn-period-sec", type=float, default=0.1)
    parser.add_argument("--keep", action="store_true", help="Leave the spawned actor and spawn point for UI inspection.")
    parser.add_argument(
        "--stop-after-create",
        action="store_true",
        help="Exit immediately after CreateVehicleSpawnPoint. Cleanup is skipped for diagnosis.",
    )
    parser.add_argument(
        "--stop-after-enable",
        action="store_true",
        help="Exit immediately after EnableSpawnPoint. Cleanup is skipped for diagnosis.",
    )
    parser.add_argument(
        "--stop-after-route",
        action="store_true",
        help="Exit immediately after SetVehicleRoute. Cleanup is skipped for diagnosis.",
    )
    parser.add_argument(
        "--no-cleanup",
        action="store_true",
        help="Never disable or destroy test objects, including automatic cleanup.",
    )
    parser.add_argument("--cleanup-test-objects", action="store_true")
    parser.add_argument("--cleanup-actor-id", default="")
    args = parser.parse_args()

    cfg = load_runtime_config(args.config)
    grpc_cfg = cfg.get("grpc", {})
    path_cfg = cfg.get("paths", {})
    host = args.host or grpc_cfg.get("host", "127.0.0.1")
    port = int(args.port or grpc_cfg.get("port", 7789))
    client_key = args.client_key or grpc_cfg.get("client_key", "aim_scenario_runner")
    grpc_src = args.grpc_src or path_cfg.get("grpc_src") or os.path.join(WORKSPACE_ROOT, "grpc_inha_univ", "src")
    add_grpc_paths(grpc_src)

    from proto.morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE, STATUS_CODE_SUCCESS
    from proto.sim_adapter import SimAdapter

    adapter = SimAdapter()
    adapter.connect(host, port)
    spawned_actor_info = None
    spawned_actor_id = ""
    ego_actor_id = ""
    spawn_param = None
    spawnpoint_object_info = None
    skip_cleanup_reason = ""

    try:
        ego_entry = get_vehicle_state_entry_by_id(adapter, client_key, args.ego_id)
        if ego_entry is not None:
            ego_actor_id = str(ego_entry[0])
            print(f"[ego-id] actor_id={ego_actor_id}")
        else:
            print(f"[ego-id] actor_id=unknown requested_ego_id={args.ego_id!r}")

        if args.cleanup_test_objects:
            if args.no_cleanup:
                return
            log_stage_boundary("cleanup 호출 전", "BEFORE")
            cleanup_test_objects(
                adapter,
                args.spawnpoint_id,
                args.cleanup_actor_id,
                client_key,
                STATUS_CODE_SUCCESS,
                ego_actor_id=ego_actor_id,
            )
            log_stage_boundary("cleanup 호출 후", "AFTER")
            return

        current_route = {} if args.use_current_ego_pose else load_current_route()
        pose, pose_source, route_links, route_source, ego_link, resolved_ego_actor_id, ego_state = resolve_pose_and_route(
            args,
            adapter,
            client_key,
            current_route,
        )
        if resolved_ego_actor_id:
            ego_actor_id = str(resolved_ego_actor_id)
            print(f"[ego-id] actor_id={ego_actor_id} source=use-current-ego-pose")
        field_speed = args.desired_speed_kmh if args.field_unit == "kmh" else args.desired_speed_kmh / 3.6
        expected_kmh = args.desired_speed_kmh
        print(f"[config] host={host}:{port} client_key={client_key} grpc_src={grpc_src}")
        print(
            f"[pose-source] {pose_source}"
            + (f" ego_id={args.ego_id} spawn_ahead_m={args.spawn_ahead_m:.1f}" if pose_source == "current_ego" else "")
        )
        if pose_source == "current_ego" and ego_state is not None:
            ego_pose = pose_from_state(ego_state)
            print(f"[ego-pose] x={ego_pose[0]:.3f} y={ego_pose[1]:.3f} z={ego_pose[2]:.3f} yaw_deg={ego_pose[3]:.3f}")
            print(f"[spawn-offset] ahead_m={args.spawn_ahead_m:.3f}")
        if args.saved_pose_name:
            print(f"[saved-pose-name] {args.saved_pose_name}")
        print(f"[spawn-pose] x={pose[0]:.3f} y={pose[1]:.3f} z={pose[2]:.3f} yaw_deg={pose[3]:.3f}")
        if ego_link:
            print(f"[ego-link] {ego_link}")
        print(f"[route-source] {route_source}")
        print(f"[route-links] {route_links}")
        if route_source == "saved_pose_link" and len(route_links) <= 1:
            print("[WARN] route_links has only one link; SetVehicleRoute or spawn activation may fail.")
        if route_source == "current_ego_link" and len(route_links) <= 1:
            print("[WARN] route_links has only one link; SetVehicleRoute or spawn activation may fail.")
            if args.dry_run_pose:
                print("[dry-run] pose check complete; no CreateVehicleSpawnPoint call was made.")
                return
            raise SystemExit("Aborting: pass --route-links A,B,C to run with a multi-link route.")
        if args.dry_run_pose:
            print("[dry-run] pose check complete; no CreateVehicleSpawnPoint call was made.")
            return
        print(
            f"[config] desired_velocity_type=VELOCITY_TYPE_CUSTOMVELOCITY(2) "
            f"desired_field={field_speed:.6f} field_unit={args.field_unit} expected={expected_kmh:.2f}km/h"
        )

        before_states = get_vehicle_states(adapter, client_key)
        model_name = select_model(adapter, args.model)
        spawn_param = build_spawn_point_param(args, model_name, field_speed, route_links, pose, client_key)

        log_stage_boundary("CreateVehicleSpawnPoint 호출 전", "BEFORE")
        result = adapter.create_vehicle_spawn_point(spawn_param)
        log_stage_boundary("CreateVehicleSpawnPoint 호출 후", "AFTER")
        print(f"[spawnpoint] CreateVehicleSpawnPoint model={model_name}: {result_summary(result)}")
        log_create_vehicle_spawn_point_response(result, spawn_param.spawn_point_info.object_info)
        if result_ok(result, STATUS_CODE_SUCCESS):
            spawnpoint_object_info = spawnpoint_info_from_create_result(
                result,
                spawn_param.spawn_point_info.object_info,
            )
        if args.stop_after_create:
            skip_cleanup_reason = "--stop-after-create"
            log_stop_after("--stop-after-create")
            return
        if not result_ok(result, STATUS_CODE_SUCCESS):
            raise SystemExit("CreateVehicleSpawnPoint failed.")

        log_stage_boundary("Enable spawn point 호출 전", "BEFORE")
        enable_spawn_point(adapter, spawnpoint_object_info, True, STATUS_CODE_SUCCESS)
        log_stage_boundary("Enable spawn point 호출 후", "AFTER")
        if args.stop_after_enable:
            skip_cleanup_reason = "--stop-after-enable"
            log_stop_after("--stop-after-enable")
            return

        log_stage_boundary("spawned actor 탐지 전", "BEFORE")
        actor_id, state = wait_for_spawned_actor(
            adapter,
            client_key,
            before_states.keys(),
            pose[0],
            pose[1],
            args.spawn_timeout_sec,
            0.25,
        )
        log_stage_boundary("spawned actor 탐지 후", "AFTER")
        if not actor_id or state is None:
            raise SystemExit("No spawned vehicle actor detected before timeout.")

        spawned_actor_id = actor_id
        if actor_id_matches_ego(spawned_actor_id, ego_actor_id):
            print(
                "[FATAL] spawned actor_id matches ego actor_id — "
                "aborting to prevent destroying Ego."
            )
            spawned_actor_info = None
            raise SystemExit(2)

        spawned_actor_info = state.actor_info
        if not getattr(spawned_actor_info.id, "value", ""):
            spawned_actor_info.id.value = actor_id
            spawned_actor_info.object_type = OBJECT_TYPE_VEHICLE
            spawned_actor_info.client_key = client_key

        print(f"[actor] spawned actor_id={actor_id} actor_info={spawned_actor_info}")
        log_stage_boundary("SetVehicleRoute 호출 전", "BEFORE")
        route_ok = set_route(adapter, spawned_actor_info, route_links, args.decision_range, STATUS_CODE_SUCCESS)
        log_stage_boundary("SetVehicleRoute 호출 후", "AFTER")
        print(f"[question] CreateVehicleSpawnPoint actor accepts SetVehicleRoute: {route_ok}")
        if args.stop_after_route:
            skip_cleanup_reason = "--stop-after-route"
            log_stop_after("--stop-after-route")
            return

        samples = []
        start = time.time()
        next_log = start
        while time.time() - start <= args.duration_sec:
            now = time.time()
            if now < next_log:
                time.sleep(min(0.05, next_log - now))
                continue
            states = get_vehicle_states(adapter, client_key)
            state = states.get(actor_id)
            if state is None:
                print(f"[sample] t={now - start:5.1f}s actor={actor_id} missing")
            else:
                speed = speed_kmh_from_state(state)
                samples.append(speed)
                px, py, pz = actor_position(state)
                link_id = getattr(state.vehicle_state.current_link_info.id, "value", "")
                remaining = getattr(state.vehicle_state, "remaining_distance", 0.0)
                print(
                    f"[sample] t={now - start:5.1f}s actor={actor_id} "
                    f"speed={speed:6.2f}km/h pos=({px:.2f},{py:.2f},{pz:.2f}) "
                    f"link={link_id} remaining={remaining:.1f}m"
                )
            next_log += args.log_interval_sec

        if samples:
            avg = sum(samples) / len(samples)
            max_speed = max(samples)
            min_speed = min(samples)
            print(
                f"[summary] samples={len(samples)} "
                f"avg={avg:.2f}km/h max={max_speed:.2f}km/h min={min_speed:.2f}km/h "
                f"target={expected_kmh:.2f}km/h"
            )
        else:
            print("[summary] no speed samples")

    finally:
        if args.no_cleanup:
            print("[cleanup] skipped because --no-cleanup was set")
            if spawnpoint_object_info is not None:
                print(f"[keep] spawn_point_name={spawnpoint_object_info.id.value or args.spawnpoint_id}")
            if spawned_actor_id:
                print(f"[keep] actor_id={spawned_actor_id}")
            print_network_check()
        elif skip_cleanup_reason:
            print(f"[cleanup] skipped because {skip_cleanup_reason} was set")
            if spawnpoint_object_info is not None:
                print(f"[keep] spawn_point_name={spawnpoint_object_info.id.value or args.spawnpoint_id}")
            if spawned_actor_id:
                print(f"[keep] actor_id={spawned_actor_id}")
            print_network_check()
        elif spawn_param is not None and not args.keep:
            log_stage_boundary("cleanup 호출 전", "BEFORE")
            if spawnpoint_object_info is not None:
                try:
                    enable_spawn_point(adapter, spawnpoint_object_info, False, STATUS_CODE_SUCCESS)
                except Exception as exc:
                    print(f"[cleanup] disable spawn point failed: {exc}")
            if spawned_actor_info is not None:
                try:
                    spawned_id = getattr(getattr(spawned_actor_info, "id", None), "value", "")
                    if actor_id_matches_ego(spawned_id, ego_actor_id):
                        print(
                            "[FATAL] spawned actor_id matches ego actor_id — "
                            "skipping destroy to prevent destroying Ego."
                        )
                    else:
                        print(
                            f"[safety] destroy target actor_id={spawned_id} "
                            f"is not ego actor_id={ego_actor_id or 'unknown'}"
                        )
                        destroy_object(
                            adapter,
                            spawned_actor_info,
                            "spawned_actor",
                            STATUS_CODE_SUCCESS,
                            ego_actor_id=ego_actor_id,
                        )
                except Exception as exc:
                    print(f"[cleanup] destroy spawned actor failed: {exc}")
            try:
                if spawnpoint_object_info is not None and is_confirmed_vehicle_spawn_point(adapter, spawnpoint_object_info):
                    destroy_object(
                        adapter,
                        spawnpoint_object_info,
                        "spawn_point",
                        STATUS_CODE_SUCCESS,
                        ego_actor_id=ego_actor_id,
                    )
                else:
                    print("[WARN] cleanup skipped spawn point destroy: target is not confirmed as VehicleSpawnPoint")
            except Exception as exc:
                print(f"[cleanup] destroy spawn point failed: {exc}")
            log_stage_boundary("cleanup 호출 후", "AFTER")
        elif args.keep:
            print("[cleanup] skipped because --keep was set")
            keep_spawnpoint_id = (
                spawnpoint_object_info.id.value
                if spawnpoint_object_info is not None and spawnpoint_object_info.id.value
                else args.spawnpoint_id
            )
            print(f"[keep] spawn_point_name={keep_spawnpoint_id}")
            print(f"[keep] actor_id={spawned_actor_id or 'unknown'}")
        adapter.disconnect()


if __name__ == "__main__":
    main()
