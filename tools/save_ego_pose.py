#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import datetime as dt
import math
import os
import sys

import yaml

RUNNER_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKSPACE_ROOT = RUNNER_ROOT

if RUNNER_ROOT not in sys.path:
    sys.path.insert(0, RUNNER_ROOT)

from utils.mgeo_map_loader import MGeoMapLoader
from utils.route_link_groups import (
    add_link_to_group,
    ensure_candidate_link_groups,
    flatten_candidate_links,
)


def runner_path(path):
    if not path:
        return path
    path = os.path.expanduser(os.path.expandvars(path))
    if os.path.isabs(path):
        return path
    return os.path.join(RUNNER_ROOT, path)


def workspace_path(path):
    if not path:
        return path
    path = os.path.expanduser(os.path.expandvars(path))
    if os.path.isabs(path):
        return path
    return os.path.join(WORKSPACE_ROOT, path)


def load_yaml(path, default=None):
    if not os.path.exists(path):
        return default
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        return default
    return data


def write_yaml(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def load_runtime_config():
    cfg = load_yaml(runner_path("config/runtime.yaml"), default={})
    local_cfg_path = runner_path("config/local_override.yaml")
    local_cfg = load_yaml(local_cfg_path, default={})

    for section, values in local_cfg.items():
        if isinstance(values, dict) and isinstance(cfg.get(section), dict):
            cfg[section].update(values)
        else:
            cfg[section] = values

    paths = cfg.get("paths", {})
    for key, value in list(paths.items()):
        if isinstance(value, str) and not os.path.isabs(value):
            paths[key] = workspace_path(value)

    return cfg


def add_grpc_paths(grpc_src):
    for path in [
        grpc_src,
        os.path.join(grpc_src, "api"),
        os.path.join(grpc_src, "proto"),
    ]:
        if path not in sys.path:
            sys.path.append(path)


def extract_ego_state(actor_state):
    vehicle_state = actor_state.vehicle_state
    try:
        current_link = vehicle_state.current_link_info.id.value
    except Exception:
        current_link = ""

    vx = float(actor_state.velocity.x)
    vy = float(actor_state.velocity.y)
    vz = float(actor_state.velocity.z)

    return {
        "x": float(actor_state.transform.location.x),
        "y": float(actor_state.transform.location.y),
        "z": float(actor_state.transform.location.z),
        "yaw_deg": float(actor_state.transform.rotation.z),
        "speed": math.sqrt(vx * vx + vy * vy + vz * vz),
        "current_link": current_link,
        "front_wheel_angle": float(vehicle_state.front_wheel_angle),
    }


def read_ego_state_direct(global_cfg, ego_id, client_key_override=None):
    grpc_cfg = global_cfg["grpc"]
    path_cfg = global_cfg["paths"]

    host = grpc_cfg.get("host", "127.0.0.1")
    port = int(grpc_cfg.get("port", 7789))
    client_key = client_key_override or grpc_cfg.get("client_key", "aim_scenario_runner")

    add_grpc_paths(path_cfg["grpc_src"])

    from api.morai_sim_client import MoraiSimClient
    from proto.morai.common.enum_pb2 import OBJECT_TYPE_UNSPECIFIED, OBJECT_TYPE_VEHICLE
    from proto.morai.common.object_info_pb2 import ObjectInfo

    client = MoraiSimClient(client_key)
    client.connect(host, port)
    if not client.is_connected():
        raise RuntimeError(f"Failed to connect MORAI gRPC server: {host}:{port}")

    attempts = [
        (client_key, OBJECT_TYPE_VEHICLE),
        ("", OBJECT_TYPE_VEHICLE),
        (client_key, OBJECT_TYPE_UNSPECIFIED),
    ]

    try:
        for key, object_type in attempts:
            object_info = ObjectInfo()
            object_info.id.value = ego_id
            object_info.object_type = object_type
            object_info.client_key = key

            actor_state = client._sim_adapter.get_actor_state(object_info)
            if actor_state is not None:
                return extract_ego_state(actor_state)
    finally:
        client.disconnect()

    raise RuntimeError(f"Failed to read Ego actor state: ego_id={ego_id}")


def point_segment_distance_sq(px, py, p0, p1):
    x0, y0 = float(p0[0]), float(p0[1])
    x1, y1 = float(p1[0]), float(p1[1])
    dx = x1 - x0
    dy = y1 - y0
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return (px - x0) ** 2 + (py - y0) ** 2
    t = ((px - x0) * dx + (py - y0) * dy) / denom
    t = max(0.0, min(1.0, t))
    cx = x0 + t * dx
    cy = y0 + t * dy
    return (px - cx) ** 2 + (py - cy) ** 2


def nearest_link(map_loader, x, y):
    best_link = None
    best_dist_sq = float("inf")

    for link_id, link in map_loader.link_set.items():
        points = link.get("points", [])
        if len(points) < 2:
            continue
        for p0, p1 in zip(points[:-1], points[1:]):
            dist_sq = point_segment_distance_sq(x, y, p0, p1)
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_link = link_id

    if best_link is None:
        return None, float("inf")
    return best_link, math.sqrt(best_dist_sq)


def unique_pose_name(poses, requested_name):
    base = requested_name or dt.datetime.now().strftime("ego_pose_%Y%m%d_%H%M%S")
    used = {pose.get("name") for pose in poses}
    if base not in used:
        return base

    index = 2
    while f"{base}_{index:02d}" in used:
        index += 1
    return f"{base}_{index:02d}"


def make_pose_record(name, state, candidate_link, nearest_link_id, nearest_dist_m, preview_png):
    return {
        "name": name,
        "saved_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "x": float(state["x"]),
        "y": float(state["y"]),
        "z": float(state["z"]),
        "yaw_deg": float(state["yaw_deg"]),
        "speed": float(state["speed"]),
        "current_link": state.get("current_link") or "",
        "nearest_link": nearest_link_id or "",
        "nearest_link_distance_m": float(nearest_dist_m),
        "candidate_link": candidate_link or "",
        "confirmed": False,
        "candidate_added": False,
        "preview_png": preview_png,
    }


def plot_pose_check(map_loader, zone_data, poses, current_record, output_path):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError(f"matplotlib is required for pose preview: {exc}") from exc

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    candidate_links = set(flatten_candidate_links(zone_data))
    current_link = current_record.get("candidate_link") or current_record.get("nearest_link")

    fig, ax = plt.subplots(figsize=(12, 12))

    for link_id, link in map_loader.link_set.items():
        points = link.get("points", [])
        if len(points) < 2:
            continue
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        if link_id in candidate_links:
            ax.plot(xs, ys, color="#5d8f64", linewidth=0.9, alpha=0.85, zorder=1)
        else:
            ax.plot(xs, ys, color="#cfcfcf", linewidth=0.35, alpha=0.65, zorder=0)

    if current_link in map_loader.link_set:
        points = map_loader.get_link_points(current_link)
        ax.plot(
            [p[0] for p in points],
            [p[1] for p in points],
            color="#006dff",
            linewidth=2.2,
            alpha=0.95,
            zorder=3,
            label="candidate link",
        )

    polygon = zone_data.get("polygon") or []
    if len(polygon) >= 3:
        xs = [p[0] for p in polygon] + [polygon[0][0]]
        ys = [p[1] for p in polygon] + [polygon[0][1]]
        ax.plot(xs, ys, color="#555555", linestyle="--", linewidth=1.0, zorder=2)

    other_poses = [pose for pose in poses if pose.get("name") != current_record["name"]]
    if other_poses:
        ax.scatter(
            [pose["x"] for pose in other_poses],
            [pose["y"] for pose in other_poses],
            color="#6a8caf",
            s=22,
            alpha=0.65,
            label="saved poses",
            zorder=4,
        )

    x = current_record["x"]
    y = current_record["y"]
    yaw = math.radians(current_record["yaw_deg"])
    ax.scatter([x], [y], color="#ff8c00", marker="^", s=140, label="current ego", zorder=6)
    ax.scatter(
        [x],
        [y],
        facecolors="none",
        edgecolors="#006dff",
        linewidths=2.4,
        s=220,
        label="saved pose",
        zorder=7,
    )
    ax.arrow(
        x,
        y,
        math.cos(yaw) * 6.0,
        math.sin(yaw) * 6.0,
        width=0.35,
        color="#ff8c00",
        length_includes_head=True,
        zorder=8,
    )
    ax.text(
        x,
        y,
        f"  {current_record['name']}\n  {current_record.get('candidate_link', '')}",
        fontsize=8,
        color="#111111",
        zorder=9,
    )

    ax.set_aspect("equal", adjustable="box")
    ax.set_title(
        f"Ego pose check: {current_record['name']} | "
        f"link={current_record.get('candidate_link', '')}"
    )
    ax.grid(True, linewidth=0.25, alpha=0.45)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def display_preview(image_path, window_title, no_display):
    if no_display:
        return False

    try:
        import cv2

        image = cv2.imread(image_path)
        if image is None:
            return False
        cv2.imshow(window_title, image)
        cv2.waitKey(300)
        return True
    except Exception as exc:
        print(f"[PoseSave] preview window skipped: {exc}")
        return False


def close_preview(window_title):
    try:
        import cv2

        cv2.destroyWindow(window_title)
        cv2.waitKey(1)
    except Exception:
        pass


def prompt_group(groups):
    names = list(groups.keys())
    print("\n후보 링크를 넣을 차선 그룹을 고르세요.")
    for index, name in enumerate(names, start=1):
        print(f"  {index}. {name}")

    while True:
        raw = input("그룹 번호/이름 입력, Enter=후보 YAML 추가 안 함: ").strip()
        if not raw:
            return None
        if raw.isdigit():
            index = int(raw)
            if 1 <= index <= len(names):
                return names[index - 1]
        if raw in groups:
            return raw
        print("알 수 없는 그룹입니다. 다시 입력하세요.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Save current MORAI Ego pose and optionally add its link to urban candidates."
    )
    parser.add_argument("--name", default=None, help="Saved pose name.")
    parser.add_argument("--zone", default="urban", help="Zone key in route link YAML.")
    parser.add_argument("--ego-id", default="Ego", help="MORAI Ego actor id.")
    parser.add_argument("--client-key", default=None, help="Override gRPC client_key.")
    parser.add_argument(
        "--poses-yaml",
        default="config/saved_ego_positions.yaml",
        help="Pose archive YAML path relative to aim_scenario_runner.",
    )
    parser.add_argument(
        "--route-links-yaml",
        default="config/urban_route_links.yaml",
        help="Candidate link YAML path relative to aim_scenario_runner.",
    )
    parser.add_argument(
        "--preview-dir",
        default="ego_pose_checks",
        help="Preview image directory relative to aim_scenario_runner.",
    )
    parser.add_argument("--group", default=None, help="Candidate link group name.")
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Add to candidate YAML without interactive confirmation. Requires --group.",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Only save preview PNG; do not open an OpenCV window.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.yes and not args.group:
        raise SystemExit("--yes requires --group")

    global_cfg = load_runtime_config()
    map_loader = MGeoMapLoader(global_cfg["paths"]["mgeo_root"])

    route_links_path = runner_path(args.route_links_yaml)
    route_data = load_yaml(route_links_path, default={})
    zone_data = route_data.setdefault(args.zone, {})
    groups = ensure_candidate_link_groups(zone_data)

    state = read_ego_state_direct(global_cfg, args.ego_id, args.client_key)

    nearest_link_id, nearest_dist_m = nearest_link(map_loader, state["x"], state["y"])
    current_link = state.get("current_link") or ""
    if current_link in map_loader.link_set:
        candidate_link = current_link
    else:
        candidate_link = nearest_link_id

    poses_path = runner_path(args.poses_yaml)
    poses_data = load_yaml(poses_path, default={"poses": []})
    poses = poses_data.setdefault("poses", [])

    name = unique_pose_name(poses, args.name)
    preview_dir = runner_path(args.preview_dir)
    preview_png = os.path.join(
        preview_dir,
        f"{name}_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.png",
    )
    preview_rel = os.path.relpath(preview_png, RUNNER_ROOT)

    record = make_pose_record(
        name=name,
        state=state,
        candidate_link=candidate_link,
        nearest_link_id=nearest_link_id,
        nearest_dist_m=nearest_dist_m,
        preview_png=preview_rel,
    )
    poses.append(record)
    write_yaml(poses_path, poses_data)

    plot_pose_check(map_loader, zone_data, poses, record, preview_png)
    displayed = display_preview(preview_png, "AIM Ego Pose Check", args.no_display)

    print(f"\n[PoseSave] saved pose: {poses_path}")
    print(f"[PoseSave] preview png: {preview_png}")
    print(
        "[PoseSave] ego="
        f"({state['x']:.3f}, {state['y']:.3f}, {state['z']:.3f}), "
        f"yaw={state['yaw_deg']:.2f}, speed={state['speed']:.2f}"
    )
    print(
        f"[PoseSave] current_link={current_link or '(empty)'}, "
        f"nearest_link={nearest_link_id} ({nearest_dist_m:.2f}m), "
        f"candidate_link={candidate_link}"
    )

    group_name = args.group
    if group_name and group_name not in groups:
        raise SystemExit(f"Unknown candidate link group: {group_name}")

    if not group_name and sys.stdin.isatty():
        group_name = prompt_group(groups)

    if group_name:
        if args.yes:
            answer = "y"
        else:
            answer = input(
                f"{candidate_link} 링크를 '{group_name}' 그룹에 추가하려면 Y 입력: "
            ).strip().lower()

        if answer == "y":
            added, target_group = add_link_to_group(zone_data, group_name, candidate_link)
            if added:
                write_yaml(route_links_path, route_data)
                print(f"[PoseSave] candidate link added: {target_group} <- {candidate_link}")
            else:
                print(f"[PoseSave] candidate link already exists in group: {target_group}")

            record["confirmed"] = True
            record["candidate_added"] = True
            record["candidate_group"] = target_group
            write_yaml(poses_path, poses_data)
        else:
            print("[PoseSave] candidate YAML was not changed.")
    else:
        print("[PoseSave] candidate YAML was not changed.")

    if displayed:
        close_preview("AIM Ego Pose Check")

    return 0


if __name__ == "__main__":
    sys.exit(main())
