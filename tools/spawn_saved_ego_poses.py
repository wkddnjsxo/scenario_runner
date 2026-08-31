#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import time

import yaml

RUNNER_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKSPACE_ROOT = RUNNER_ROOT

if RUNNER_ROOT not in sys.path:
    sys.path.insert(0, RUNNER_ROOT)

from utils.morai_sim_bridge import MoraiSimBridge


def runner_path(path):
    if not path or os.path.isabs(path):
        return path
    return os.path.join(RUNNER_ROOT, path)


def workspace_path(path):
    if not path or os.path.isabs(path):
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


def load_runtime_config():
    cfg = load_yaml(runner_path("config/runtime.yaml"), default={})
    local_cfg = load_yaml(runner_path("config/local_override.yaml"), default={})

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


def load_saved_poses(path, include_unconfirmed=False):
    data = load_yaml(path, default={"poses": []})
    poses = []
    for pose in data.get("poses", []) or []:
        link_id = pose.get("candidate_link") or pose.get("current_link")
        if not link_id:
            continue
        if not include_unconfirmed and not pose.get("confirmed", False):
            continue
        poses.append(pose)
    return poses


def pose_transform(sim_bridge, pose):
    return sim_bridge.make_transform(
        float(pose["x"]),
        float(pose["y"]),
        float(pose["z"]),
        float(pose["yaw_deg"]),
    )


def print_pose_order(poses):
    print(f"[SpawnCheck] poses={len(poses)}")
    for index, pose in enumerate(poses):
        link_id = pose.get("candidate_link") or pose.get("current_link")
        print(
            f"  {index:02d}: {link_id} "
            f"name={pose.get('name', 'unnamed')} "
            f"group={pose.get('candidate_group', '-')} "
            f"xy=({float(pose['x']):.3f}, {float(pose['y']):.3f}) "
            f"yaw={float(pose['yaw_deg']):.2f}"
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Spawn Ego at saved candidate poses in YAML order."
    )
    parser.add_argument(
        "--poses-yaml",
        default="config/saved_ego_positions.yaml",
        help="Saved pose YAML path relative to aim_scenario_runner.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
        help="Seconds to wait between spawns.",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=1,
        help="Number of full passes. Use 0 for infinite loop.",
    )
    parser.add_argument(
        "--settle-sec",
        type=float,
        default=0.1,
        help="Seconds to settle after each teleport.",
    )
    parser.add_argument(
        "--include-unconfirmed",
        action="store_true",
        help="Also spawn poses that were not confirmed with Y.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print pose order without connecting to MORAI.",
    )
    parser.add_argument(
        "--finalize",
        action="store_true",
        help="Finalize MORAI simulation at the end. Default leaves simulator at last pose.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    poses_path = runner_path(args.poses_yaml)
    poses = load_saved_poses(poses_path, include_unconfirmed=args.include_unconfirmed)
    if not poses:
        raise RuntimeError(f"No saved Ego poses found: {poses_path}")

    print_pose_order(poses)
    if args.dry_run:
        return 0

    global_cfg = load_runtime_config()
    sim_bridge = MoraiSimBridge(global_cfg)
    sim_bridge.connect()

    cycle = 0
    started = False
    try:
        while args.cycles == 0 or cycle < args.cycles:
            for index, pose in enumerate(poses):
                link_id = pose.get("candidate_link") or pose.get("current_link")
                transform = pose_transform(sim_bridge, pose)

                if not started:
                    sim_bridge.start_world(transform)
                    sim_bridge.place_ego_stopped(transform, settle_sec=args.settle_sec)
                    started = True
                else:
                    sim_bridge.place_ego_stopped(transform, settle_sec=args.settle_sec)

                print(
                    f"[SpawnCheck] spawned cycle={cycle + 1} "
                    f"index={index:02d}/{len(poses) - 1:02d} "
                    f"link={link_id} name={pose.get('name', 'unnamed')}"
                )

                if args.cycles == 0 or cycle < args.cycles - 1 or index < len(poses) - 1:
                    time.sleep(max(0.0, args.interval))
            cycle += 1
    except KeyboardInterrupt:
        print("\n[SpawnCheck] interrupted by user")
        return 130
    finally:
        if args.finalize:
            sim_bridge.stop()
        elif sim_bridge.client is not None:
            sim_bridge.client.disconnect()
            print("[MORAI] disconnected; simulator left at last spawned pose")

    return 0


if __name__ == "__main__":
    sys.exit(main())
