import argparse
import copy
import os
import signal
import sys
import yaml

from utils.morai_sim_bridge import MoraiSimBridge
from utils.mgeo_map_loader import MGeoMapLoader
from utils.shutdown_state import clear_shutdown, is_shutdown_requested, request_shutdown
from zones.urban_drive_cases import UrbanRouteDriveCase

RUNNER_ROOT = os.path.dirname(os.path.abspath(__file__))


def handle_sigint(_signum, _frame):
    request_shutdown()
    raise KeyboardInterrupt


def runner_path(*parts):
    return os.path.join(RUNNER_ROOT, *parts)


def load_yaml(path):
    print(f"[DEBUG] loading yaml: {path}")
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    if data is None:
        raise RuntimeError(f"YAML is empty: {path}")
    return data


def resolve_paths(cfg):
    paths = cfg.get("paths", {})
    for key, val in paths.items():
        if isinstance(val, str) and not os.path.isabs(val):
            paths[key] = os.path.join(RUNNER_ROOT, val)


def get_scenario_cfg(zone_cfg, zone, scenario):
    scenarios = zone_cfg.get("scenarios", {})
    if scenario in scenarios:
        return copy.deepcopy(scenarios[scenario])

    if zone == "urban" and scenario == "accident_stall_test":
        cfg = copy.deepcopy(scenarios["random_route_drive"])
        cfg["accident_detection_enabled"] = True
        cfg["accident_stall_duration_sec"] = 1.0
        cfg["accident_min_elapsed_sec"] = 2.0
        cfg["accident_near_npc_distance_m"] = 12.0
        cfg["accident_near_npc_forward_m"] = 12.0
        cfg["accident_near_npc_side_m"] = 4.5
        return cfg

    raise ValueError(f"Unknown scenario: {zone}/{scenario}")


def main():
    clear_shutdown()
    signal.signal(signal.SIGINT, handle_sigint)
    print("[DEBUG] main() start")

    parser = argparse.ArgumentParser()
    parser.add_argument("--zone", default="urban")
    parser.add_argument("--scenario", default="random_route_drive")
    args = parser.parse_args()

    print(f"[DEBUG] args: zone={args.zone}, scenario={args.scenario}")

    global_cfg = load_yaml(runner_path("config", "runtime.yaml"))

    local_cfg_path = runner_path("config", "local_override.yaml")
    if os.path.exists(local_cfg_path):
        local_cfg = load_yaml(local_cfg_path)
        for section, values in local_cfg.items():
            if isinstance(values, dict) and isinstance(global_cfg.get(section), dict):
                global_cfg[section].update(values)
            else:
                global_cfg[section] = values

    resolve_paths(global_cfg)

    zone_cfg = load_yaml(runner_path("config", f"{args.zone}_scenarios.yaml"))

    scenario_cfg = get_scenario_cfg(zone_cfg, args.zone, args.scenario)

    print("[DEBUG] creating MoraiSimBridge")
    sim_bridge = MoraiSimBridge(global_cfg)

    print("[DEBUG] connecting gRPC")
    sim_bridge.connect()

    print("[DEBUG] loading MGeo")
    map_loader = MGeoMapLoader(global_cfg["paths"]["mgeo_root"])

    if args.zone == "urban" and args.scenario in ("random_route_drive", "accident_stall_test"):
        scenario = UrbanRouteDriveCase(
            sim_bridge=sim_bridge,
            map_loader=map_loader,
            global_cfg=global_cfg,
            scenario_cfg=scenario_cfg,
        )
        scenario.scenario_name = args.scenario
    else:
        raise ValueError(f"Unknown scenario: {args.zone}/{args.scenario}")

    try:
        print("[DEBUG] running scenario")
        scenario.run()
    except KeyboardInterrupt:
        print("\n[DEBUG] interrupted by user")
        return 130
    except RuntimeError as exc:
        if is_shutdown_requested() and "Failed to get ego actor state" in str(exc):
            print("\n[DEBUG] interrupted by user")
            return 130
        raise
    finally:
        print("[DEBUG] stopping MoraiSimBridge")
        sim_bridge.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
