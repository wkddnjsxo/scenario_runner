import json
import random
import time
import yaml
import math
import os
import re
import shutil

from scenario_base import ScenarioBase
from utils.npc_vehicle_manager import NpcVehicleManager
from utils.pedestrian_manager import PedestrianManager
from utils.route_bev_visualizer_controller import RouteBEVVisualizerController
from utils.route_link_groups import build_link_group_lookup, flatten_candidate_links
from utils.route_planner import build_route_between
from utils.scene_dataset import SceneDatasetManager
from utils.geometry_utils import (
    interpolate_on_polyline,
    nearest_point_index,
    polyline_length,
    project_distance_on_polyline,
    dist_xy,
)

RUNNER_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKSPACE_ROOT = os.path.abspath(
    os.path.expanduser(os.path.expandvars(os.environ.get("AIM_WS_ROOT", "$HOME/aim_ws")))
)


def runner_relative_path(path):
    if not path:
        return path
    path = os.path.expanduser(os.path.expandvars(path))
    if os.path.isabs(path):
        return path
    return os.path.join(RUNNER_ROOT, path)


def workspace_relative_path(path):
    if not path:
        return path
    path = os.path.expanduser(os.path.expandvars(path))
    if os.path.isabs(path):
        return path
    return os.path.join(WORKSPACE_ROOT, path)


def append_point_if_distinct(points, point, eps=1e-3):
    if not points or dist_xy(points[-1][0], points[-1][1], point[0], point[1]) > eps:
        points.append(point)


def slice_polyline_by_distance(points, start_s, end_s):
    if len(points) < 2:
        return list(points)

    total = polyline_length(points)
    start_s = max(0.0, min(float(start_s), total))
    end_s = max(0.0, min(float(end_s), total))
    if end_s < start_s:
        start_s, end_s = end_s, start_s

    out = []
    append_point_if_distinct(out, interpolate_on_polyline(points, start_s))

    cumulative = 0.0
    for p0, p1 in zip(points[:-1], points[1:]):
        cumulative += dist_xy(p0[0], p0[1], p1[0], p1[1])
        if start_s + 1e-3 < cumulative < end_s - 1e-3:
            append_point_if_distinct(out, p1)

    append_point_if_distinct(out, interpolate_on_polyline(points, end_s))
    return out


class UrbanRouteDriveCase(ScenarioBase):
    zone_name = "urban"
    scenario_name = "random_route_drive"

    def setup(self):
        self.ego_spawn_offset_m = self.cfg.get("ego_spawn_offset_m", 5.0)
        self.decision_range = self.cfg.get("decision_range_m", 100.0)
        self.dataset_limit_reached_before_start = False

        if self.dataset_stop_target_reached():
            self.dataset_limit_reached_before_start = True
            return

        self.randomize_links = self.cfg.get("randomize_links", False)
        self.random = random.Random(self.cfg.get("random_seed"))
        self.zone_allowed_links = set()
        self.zone_link_group_lookup = {}
        self.saved_candidate_poses = self.load_saved_candidate_poses()
        self.zone_route_links = self.load_zone_route_links()
        self.route_configured = False
        self.route_setup_mode = None
        self.random_route_pool = None
        self.recent_start_links = []
        self.recent_route_keys = []
        self.last_fast_traffic_light_id = None
        self.last_fast_traffic_light_time = 0.0

        self.select_route_for_next_drive()
        self.ensure_route_bev_visualizer()

        # 첫 world 시작
        self.grpc.start_world(self.start_tf)
        self.configure_drive_with_retries()

    def load_zone_route_links(self):
        if not self.randomize_links:
            return []

        path = runner_relative_path(self.cfg.get("zone_links_path", "config/urban_route_links.yaml"))
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        zone_data = data.get(self.zone_name, {})
        route_links = flatten_candidate_links(zone_data)
        exclude_links = set(zone_data.get("exclude_links", []))
        self.zone_link_group_lookup = build_link_group_lookup(zone_data)

        allowed_route_links = zone_data.get("allowed_route_links")
        if allowed_route_links:
            allowed_source_links = allowed_route_links
        elif zone_data.get("route_links"):
            allowed_source_links = zone_data.get("route_links", [])
        else:
            allowed_source_links = list(self.map_loader.link_set.keys())

        self.zone_allowed_links = {
            link_id
            for link_id in allowed_source_links
            if link_id not in exclude_links and link_id in self.map_loader.link_set
        }

        allow_connector_links = self.cfg.get("allow_connector_links", False)
        candidates = []
        for link_id in route_links:
            if link_id not in self.zone_allowed_links:
                continue
            if not allow_connector_links and "-" in link_id:
                continue
            candidates.append(link_id)

        if len(candidates) < 2:
            raise RuntimeError(f"Need at least 2 urban route links for random mode: {path}")

        print(f"[UrbanBasicDrive] random link candidates={len(candidates)} from {path}")
        return candidates

    def select_route_for_next_drive(self):
        if self.randomize_links:
            self.select_random_route()
        else:
            self.start_link = self.cfg["start_link"]
            self.end_link = self.cfg["end_link"]
            self.prepare_route()

    def load_saved_candidate_poses(self):
        path = self.cfg.get("saved_ego_positions_path", "config/saved_ego_positions.yaml")
        path = runner_relative_path(path)
        if not os.path.exists(path):
            return {}

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            print(f"[UrbanBasicDrive] saved ego poses skipped: {exc}")
            return {}

        poses_by_link = {}
        for pose in data.get("poses", []) or []:
            if not pose.get("confirmed", False):
                continue
            link_id = pose.get("candidate_link") or pose.get("current_link")
            if not link_id:
                continue
            poses_by_link[link_id] = pose

        if poses_by_link:
            print(f"[UrbanBasicDrive] saved ego poses loaded: {len(poses_by_link)} from {path}")
        return poses_by_link

    def get_saved_candidate_pose(self, link_id):
        return getattr(self, "saved_candidate_poses", {}).get(link_id)

    def planned_start_xy_for_link(self, link_id):
        if self.cfg.get("use_saved_candidate_start_pose", True):
            pose = self.get_saved_candidate_pose(link_id)
            if pose:
                return float(pose["x"]), float(pose["y"])

        points = self.map_loader.get_link_points(link_id)
        x, y, _, _ = interpolate_on_polyline(points, self.ego_spawn_offset_m)
        return float(x), float(y)

    def planned_goal_xy_for_link(self, link_id):
        if self.cfg.get("use_saved_candidate_goal_pose", True):
            pose = self.get_saved_candidate_pose(link_id)
            if pose:
                return float(pose["x"]), float(pose["y"])

        points = self.map_loader.get_link_points(link_id)
        goal_offset_from_end_m = self.cfg.get("goal_offset_from_end_m", 3.0)
        goal_offset_m = max(0.0, polyline_length(points) - goal_offset_from_end_m)
        x, y, _, _ = interpolate_on_polyline(points, goal_offset_m)
        return float(x), float(y)

    def estimate_actual_drive_span(self, route_points, start_link, end_link):
        start_x, start_y = self.planned_start_xy_for_link(start_link)
        goal_x, goal_y = self.planned_goal_xy_for_link(end_link)
        start_s = project_distance_on_polyline(route_points, start_x, start_y)
        goal_s = project_distance_on_polyline(route_points, goal_x, goal_y)
        return {
            "start_s": float(start_s),
            "goal_s": float(goal_s),
            "drive_length_m": float(goal_s - start_s),
        }

    def route_actual_drive_span_ok(self, route_points, start_link, end_link):
        span = self.estimate_actual_drive_span(route_points, start_link, end_link)
        drive_length_m = span["drive_length_m"]
        min_length_m = float(
            self.cfg.get(
                "random_min_actual_drive_length_m",
                self.cfg.get("random_min_route_length_m", 20.0),
            )
        )
        max_length_m = float(
            self.cfg.get(
                "random_max_actual_drive_length_m",
                self.cfg.get("random_max_route_length_m", 0.0),
            )
        )

        if drive_length_m < min_length_m:
            return False, "actual_short", span
        if max_length_m > 0.0 and drive_length_m > max_length_m:
            return False, "actual_long", span
        return True, "ok", span

    def select_random_route(self):
        if self.cfg.get("random_route_pool_enabled", False):
            self.select_random_route_from_pool()
            return

        attempts = int(self.cfg.get("random_route_attempts", 200))
        min_length_m = float(self.cfg.get("random_min_route_length_m", 20.0))
        max_length_m = float(self.cfg.get("random_max_route_length_m", 0.0))
        max_route_links = int(self.cfg.get("random_max_route_links", 0))
        max_route_point_gap_m = float(self.cfg.get("random_max_route_point_gap_m", 0.0))
        stay_in_zone = self.cfg.get("random_route_stay_in_zone", True)
        zone_link_set = self.zone_allowed_links

        last_error = None
        last_reject_reason = None
        for attempt in range(1, attempts + 1):
            start_link, end_link = self.random.sample(self.zone_route_links, 2)

            try:
                route_links, route_length_m = build_route_between(
                    self.map_loader,
                    start_link,
                    end_link,
                )
            except Exception as e:
                last_error = e
                continue

            if route_length_m < min_length_m:
                continue
            if max_length_m > 0.0 and route_length_m > max_length_m:
                continue
            if max_route_links > 0 and len(route_links) > max_route_links:
                continue
            if stay_in_zone and any(link_id not in zone_link_set for link_id in route_links):
                continue
            if max_route_point_gap_m > 0.0:
                route_points = self.build_route_points(route_links)
                max_gap = self.max_route_point_gap(route_points)
                if max_gap > max_route_point_gap_m:
                    last_reject_reason = f"max route point gap {max_gap:.1f}m"
                    continue
            else:
                route_points = self.build_route_points(route_links)

            ok, reason, span = self.route_actual_drive_span_ok(
                route_points,
                start_link,
                end_link,
            )
            if not ok:
                last_reject_reason = (
                    f"{reason} actual_drive={span['drive_length_m']:.1f}m "
                    f"start_s={span['start_s']:.1f} goal_s={span['goal_s']:.1f}"
                )
                continue

            self.start_link = start_link
            self.end_link = end_link
            self.prepare_route(route_links=route_links, route_length_m=route_length_m)
            print(f"[UrbanBasicDrive] random route selected on attempt={attempt}")
            return

        raise RuntimeError(
            "Failed to select connected random urban route. "
            f"attempts={attempts}, last_error={last_error}, last_reject={last_reject_reason}"
        )

    def build_random_route_pool(self):
        min_length_m = float(self.cfg.get("random_min_route_length_m", 20.0))
        max_length_m = float(self.cfg.get("random_max_route_length_m", 0.0))
        max_route_links = int(self.cfg.get("random_max_route_links", 0))
        max_route_point_gap_m = float(self.cfg.get("random_max_route_point_gap_m", 0.0))
        stay_in_zone = self.cfg.get("random_route_stay_in_zone", True)
        zone_link_set = self.zone_allowed_links

        pool = []
        reject_counts = {
            "no_path": 0,
            "short": 0,
            "long": 0,
            "too_many_links": 0,
            "out_of_zone": 0,
            "gap": 0,
            "actual_short": 0,
            "actual_long": 0,
        }

        for start_link in self.zone_route_links:
            for end_link in self.zone_route_links:
                if start_link == end_link:
                    continue

                try:
                    route_links, route_length_m = build_route_between(
                        self.map_loader,
                        start_link,
                        end_link,
                    )
                except Exception:
                    reject_counts["no_path"] += 1
                    continue

                if route_length_m < min_length_m:
                    reject_counts["short"] += 1
                    continue
                if max_length_m > 0.0 and route_length_m > max_length_m:
                    reject_counts["long"] += 1
                    continue
                if max_route_links > 0 and len(route_links) > max_route_links:
                    reject_counts["too_many_links"] += 1
                    continue
                if stay_in_zone and any(link_id not in zone_link_set for link_id in route_links):
                    reject_counts["out_of_zone"] += 1
                    continue

                route_points = self.build_route_points(route_links)
                max_gap = self.max_route_point_gap(route_points)
                if max_route_point_gap_m > 0.0 and max_gap > max_route_point_gap_m:
                    reject_counts["gap"] += 1
                    continue

                ok, reason, span = self.route_actual_drive_span_ok(
                    route_points,
                    start_link,
                    end_link,
                )
                if not ok:
                    reject_counts[reason] += 1
                    continue

                pool.append(
                    {
                        "start_link": start_link,
                        "end_link": end_link,
                        "route_links": route_links,
                        "route_length_m": polyline_length(route_points),
                        "actual_drive_length_m": span["drive_length_m"],
                        "actual_start_s": span["start_s"],
                        "actual_goal_s": span["goal_s"],
                        "max_gap": max_gap,
                    }
                )

        if not pool:
            raise RuntimeError(f"No valid random route pool. rejects={reject_counts}")

        self.random.shuffle(pool)
        print(
            f"[UrbanBasicDrive] random route pool built: routes={len(pool)}, "
            f"unique_starts={len(set(item['start_link'] for item in pool))}, "
            f"unique_ends={len(set(item['end_link'] for item in pool))}, "
            f"rejects={reject_counts}"
        )
        return pool

    def select_random_route_from_pool(self):
        if self.random_route_pool is None:
            self.random_route_pool = self.build_random_route_pool()

        start_memory = int(self.cfg.get("random_recent_start_memory", 0))
        route_memory = int(self.cfg.get("random_recent_route_memory", 0))

        candidates = [
            item
            for item in self.random_route_pool
            if item["start_link"] not in self.recent_start_links
            and tuple(item["route_links"]) not in self.recent_route_keys
        ]
        if not candidates:
            candidates = [
                item
                for item in self.random_route_pool
                if tuple(item["route_links"]) not in self.recent_route_keys
            ]
        if not candidates:
            candidates = self.random_route_pool

        item = self.random.choice(candidates)
        self.start_link = item["start_link"]
        self.end_link = item["end_link"]

        self.recent_start_links.append(self.start_link)
        if start_memory > 0:
            self.recent_start_links = self.recent_start_links[-start_memory:]
        else:
            self.recent_start_links = []

        route_key = tuple(item["route_links"])
        self.recent_route_keys.append(route_key)
        if route_memory > 0:
            self.recent_route_keys = self.recent_route_keys[-route_memory:]
        else:
            self.recent_route_keys = []

        self.prepare_route(
            route_links=item["route_links"],
            route_length_m=item["route_length_m"],
        )
        print(
            f"[UrbanBasicDrive] route pool selected "
            f"candidates={len(candidates)}/{len(self.random_route_pool)}, "
            f"actual_drive={item.get('actual_drive_length_m', -1.0):.1f}m, "
            f"max_gap={item['max_gap']:.2f}m"
        )

    def prepare_route(self, route_links=None, route_length_m=None):
        # 시작 위치 계산
        start_pose = None
        if self.cfg.get("use_saved_candidate_start_pose", True):
            start_pose = self.get_saved_candidate_pose(self.start_link)

        if start_pose:
            x = float(start_pose["x"])
            y = float(start_pose["y"])
            z = float(start_pose["z"])
            yaw = float(start_pose["yaw_deg"])
            start_source = f"saved_pose:{start_pose.get('name', self.start_link)}"
        else:
            start_points = self.map_loader.get_link_points(self.start_link)
            x, y, z, yaw = interpolate_on_polyline(start_points, self.ego_spawn_offset_m)
            start_source = f"link_offset:{self.ego_spawn_offset_m:.1f}m"

        self.start_x = x
        self.start_y = y
        self.start_z = z
        self.start_yaw = yaw
        self.start_tf = self.grpc.make_transform(x, y, z, yaw)

        # 목표 위치 계산.
        # 링크의 마지막 점은 다음 링크와 공유되는 node인 경우가 많아 MORAI destination
        # 매칭이 다음 링크로 튈 수 있으므로 end_link 내부로 조금 당긴 점을 사용한다.
        goal_pose = None
        if self.cfg.get("use_saved_candidate_goal_pose", True):
            goal_pose = self.get_saved_candidate_pose(self.end_link)

        if goal_pose:
            self.goal_x = float(goal_pose["x"])
            self.goal_y = float(goal_pose["y"])
            self.goal_z = float(goal_pose["z"])
            self.goal_yaw = float(goal_pose["yaw_deg"])
            goal_source = f"saved_pose:{goal_pose.get('name', self.end_link)}"
        else:
            end_points = self.map_loader.get_link_points(self.end_link)
            goal_offset_from_end_m = self.cfg.get("goal_offset_from_end_m", 3.0)
            goal_offset_m = max(0.0, polyline_length(end_points) - goal_offset_from_end_m)
            self.goal_x, self.goal_y, self.goal_z, self.goal_yaw = interpolate_on_polyline(
                end_points,
                goal_offset_m,
            )
            goal_source = f"link_end_offset:{goal_offset_from_end_m:.1f}m"

        print(f"[UrbanBasicDrive] start_link={self.start_link}")
        print(f"[UrbanBasicDrive] end_link={self.end_link}")
        print(
            f"[UrbanBasicDrive] start=({x:.3f}, {y:.3f}, {z:.3f}, "
            f"yaw={yaw:.3f}, source={start_source})"
        )
        print(
            f"[UrbanBasicDrive] goal=({self.goal_x:.3f}, {self.goal_y:.3f}, {self.goal_z:.3f}, "
            f"yaw={self.goal_yaw:.3f}, source={goal_source})"
        )

        if route_links is None or route_length_m is None:
            route_links, route_length_m = build_route_between(
                self.map_loader,
                self.start_link,
                self.end_link,
            )

        self.route_links = route_links
        self.route_points = self.build_route_points(self.route_links)
        self.route_length_m = polyline_length(self.route_points)
        try:
            actual_span = self.estimate_actual_drive_span(
                self.route_points,
                self.start_link,
                self.end_link,
            )
            self.actual_drive_start_s = actual_span["start_s"]
            self.actual_drive_goal_s = actual_span["goal_s"]
            self.actual_drive_length_m = actual_span["drive_length_m"]
        except Exception as exc:
            print(f"[UrbanBasicDrive] actual drive span estimate failed: {exc}")
            self.actual_drive_start_s = 0.0
            self.actual_drive_goal_s = self.route_length_m
            self.actual_drive_length_m = self.route_length_m
        self.pure_pursuit_last_s = 0.0
        self.export_route_bev_route()

        print(f"[UrbanBasicDrive] route_links={len(self.route_links)}, length={self.route_length_m:.1f}m")
        print(
            f"[UrbanBasicDrive] actual_drive={self.actual_drive_length_m:.1f}m "
            f"(start_s={self.actual_drive_start_s:.1f}, goal_s={self.actual_drive_goal_s:.1f})"
        )
        print(f"[UrbanBasicDrive] route_max_point_gap={self.max_route_point_gap(self.route_points):.2f}m")
        print(f"[UrbanBasicDrive] route_end_link={self.route_links[-1]}")
        print("[UrbanBasicDrive] route:")
        for i, link_id in enumerate(self.route_links):
            print(f"  {i:02d}: {link_id}")

        self.route_waypoint_indices = self.build_route_waypoint_indices()
        print("[UrbanBasicDrive] route waypoints:")
        for link_id, waypoint_idx in zip(self.route_links, self.route_waypoint_indices):
            print(f"  {link_id}: waypoint_idx={waypoint_idx}")

        if self.cfg.get("route_debug_enabled", False):
            self.dump_route_debug()

    def route_bev_route_json_path(self):
        path = self.cfg.get(
            "route_bev_visualizer_route_json",
            "runtime/current_route.json",
        )
        return runner_relative_path(path)

    def build_route_visual_points(self):
        if len(getattr(self, "route_points", [])) < 2:
            return list(getattr(self, "route_points", []))
        if not all(hasattr(self, name) for name in ("start_x", "start_y", "goal_x", "goal_y")):
            return list(self.route_points)

        try:
            start_s = project_distance_on_polyline(
                self.route_points,
                self.start_x,
                self.start_y,
            )
            goal_s = project_distance_on_polyline(
                self.route_points,
                self.goal_x,
                self.goal_y,
            )
            return slice_polyline_by_distance(self.route_points, start_s, goal_s)
        except Exception as exc:
            print(f"[LBCBEV-Viz] route visual clipping skipped: {exc}")
            return list(self.route_points)

    def build_route_payload(self):
        visual_route_points = self.build_route_visual_points()
        visual_route_length_m = polyline_length(visual_route_points)

        return {
            "updated_at": time.time(),
            "zone": self.zone_name,
            "scenario": self.scenario_name,
            "start_link": self.start_link,
            "end_link": self.end_link,
            "route_links": list(self.route_links),
            "route_length_m": float(visual_route_length_m),
            "route_full_length_m": float(self.route_length_m),
            "start_xy": [
                float(getattr(self, "start_x", 0.0)),
                float(getattr(self, "start_y", 0.0)),
                float(getattr(self, "start_z", 0.0)),
            ],
            "goal_xy": [
                float(self.goal_x),
                float(self.goal_y),
                float(self.goal_z),
            ],
            "route_points": [
                [
                    float(point[0]),
                    float(point[1]),
                    float(point[2]) if len(point) >= 3 else 0.0,
                ]
                for point in visual_route_points
            ],
        }

    def export_route_bev_route(self):
        if not self.cfg.get("route_bev_visualizer_enabled", True):
            return

        route_json = self.route_bev_route_json_path()
        os.makedirs(os.path.dirname(route_json), exist_ok=True)
        payload = self.build_route_payload()
        with open(route_json, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        print(f"[LBCBEV-Viz] route exported: {route_json}")

    def publish_dataset_control(self, command, extra=None):
        payload = {
            "command": command,
            "sent_time": time.time(),
            "zone": self.zone_name,
            "scenario": self.scenario_name,
            "start_link": getattr(self, "start_link", None),
            "end_link": getattr(self, "end_link", None),
            "route_links": list(getattr(self, "route_links", [])),
            "route_length_m": float(getattr(self, "route_length_m", 0.0)),
            "start_xy": [
                float(getattr(self, "start_x", 0.0)),
                float(getattr(self, "start_y", 0.0)),
                float(getattr(self, "start_z", 0.0)),
            ],
            "goal_xy": [
                float(getattr(self, "goal_x", 0.0)),
                float(getattr(self, "goal_y", 0.0)),
                float(getattr(self, "goal_z", 0.0)),
            ],
        }
        if extra:
            payload.update(extra)

        try:
            import rospy
            from std_msgs.msg import String

            if not rospy.core.is_initialized():
                rospy.init_node(
                    "aim_scenario_dataset_control",
                    anonymous=True,
                    disable_signals=True,
                )

            pub = getattr(self, "dataset_control_pub", None)
            if pub is None:
                pub = rospy.Publisher("/dataset_control", String, queue_size=1)
                self.dataset_control_pub = pub
                self.dataset_status_sub = rospy.Subscriber(
                    "/dataset_status",
                    String,
                    self.dataset_status_callback,
                    queue_size=10,
                )
                time.sleep(0.2)

            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False, default=str)
            pub.publish(msg)
            print(f"[DatasetControl] published {command}")
        except Exception as exc:
            print(f"[DatasetControl] publish failed command={command}: {exc}")

    def dataset_status_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.dataset_last_status = payload

    def dataset_collect_root(self):
        return self.cfg.get("dataset_collect_root", "/data/dataset")

    def dataset_collect_prefix(self):
        return self.cfg.get("dataset_collect_scene_prefix", "scen")

    def dataset_stop_target_index(self):
        target = self.cfg.get("dataset_stop_scene_index", 0)
        try:
            return int(target)
        except (TypeError, ValueError):
            return 0

    def latest_dataset_scene_index(self):
        root = self.dataset_collect_root()
        prefix = self.dataset_collect_prefix()
        pattern = re.compile(rf"^{re.escape(prefix)}(\d+)$")
        max_idx = 0

        if not root or not os.path.isdir(root):
            return 0

        for name in os.listdir(root):
            path = os.path.join(root, name)
            if not os.path.isdir(path):
                continue

            match = pattern.match(name)
            if match:
                max_idx = max(max_idx, int(match.group(1)))

        return max_idx

    def dataset_stop_target_reached(self, settle=False):
        target = self.dataset_stop_target_index()
        if target <= 0:
            return False

        if settle:
            deadline = time.time() + float(self.cfg.get("dataset_stop_settle_sec", 1.0))
            while time.time() < deadline:
                status = getattr(self, "dataset_last_status", None)
                if (
                    isinstance(status, dict)
                    and status.get("event") == "SCENE_STOPPED"
                    and int(status.get("scene_index") or 0) >= target
                ):
                    print(
                        f"[Dataset] collector confirmed target scene stopped: "
                        f"{status.get('scenario_name')} frames={status.get('frame_count')}"
                    )
                    return True
                time.sleep(0.05)

        current = self.latest_dataset_scene_index()
        if current >= target:
            print(
                f"[Dataset] target scene reached: "
                f"{self.dataset_collect_prefix()}{current:02d} >= "
                f"{self.dataset_collect_prefix()}{target:02d}. finish scenario runner."
            )
            return True

        return False

    def get_scene_dataset_manager(self):
        if not self.cfg.get("dataset_enabled", False):
            return None

        if getattr(self, "scene_dataset_manager", None) is None:
            root = workspace_relative_path(self.cfg.get("dataset_root", "dataset"))
            self.scene_dataset_manager = SceneDatasetManager(
                root_dir=root,
                scene_prefix=self.cfg.get("dataset_scene_prefix", "scene"),
                scene_digits=self.cfg.get("dataset_scene_digits", 2),
            )
        return self.scene_dataset_manager

    def save_success_scene(
        self,
        result,
        lap,
        elapsed_sec,
        dist_to_goal_m,
        ego_state=None,
        extra=None,
    ):
        payload = {
            "result": result,
            "lap": int(lap),
            "elapsed_sec": float(elapsed_sec),
            "distance_to_goal_m": float(dist_to_goal_m),
        }
        if ego_state is not None:
            payload["ego_state"] = dict(ego_state)
        if extra:
            payload.update(extra)

        self.publish_dataset_control("STOP_SUCCESS", payload)
        self.scene_collection_active = False
        return None

    def is_moving_pedestrian_accident(self, accident_event):
        if not bool(self.cfg.get("discard_moving_pedestrian_accidents", False)):
            return False

        event = dict(accident_event or {})
        actor_type = str(event.get("actor_type", ""))

        if actor_type == "pedestrian":
            return self.is_moving_pedestrian_item(event)

        if actor_type == "lead_npc_pedestrian_block":
            return self.is_moving_pedestrian_item(
                event,
                slot_key="blocked_by_slot",
                speed_key="blocked_by_speed_mps",
                initial_speed_key="blocked_by_initial_speed_mps",
                stopped_key="blocked_by_stopped",
                stop_reason_key="blocked_by_stop_reason",
            )

        return False

    def is_moving_pedestrian_item(
        self,
        item,
        slot_key="slot",
        speed_key="speed_mps",
        initial_speed_key="initial_speed_mps",
        stopped_key="stopped",
        stop_reason_key="stop_reason",
    ):
        ped = dict(item or {})
        moving_speed_mps = float(self.cfg.get("moving_pedestrian_speed_mps", 0.1))
        moving_slots = set(self.cfg.get("moving_pedestrian_slots", []) or [])
        slot = str(ped.get(slot_key, ""))
        speed_mps = float(ped.get(speed_key, 0.0))
        initial_speed_mps = float(ped.get(initial_speed_key, 0.0))
        stop_reason = str(ped.get(stop_reason_key, ""))
        stopped = bool(ped.get(stopped_key, False))

        if stopped:
            if stop_reason in ("target_reached", "target_passed", "non_walking"):
                return False
            return (
                stop_reason in ("position_stuck", "blocked_no_progress")
                or initial_speed_mps > moving_speed_mps
                or slot in moving_slots
            )

        return speed_mps > moving_speed_mps or slot in moving_slots

    def save_accident_scene(
        self,
        accident_title,
        accident_event,
        lap,
        elapsed_sec,
        dist_to_goal_m,
        ego_state=None,
    ):
        event = dict(accident_event or {})
        # 변경(2026-07-21): 사고는 종류 무관 항상 폐기 (기존엔 움직이는 보행자 사고만 폐기)
        is_moving_ped = self.is_moving_pedestrian_accident(event)
        discard_scene = True
        payload = {
            "scene_outcome": "accident",
            "success": False,
            "result": accident_title,
            "lap": int(lap),
            "elapsed_sec": float(elapsed_sec),
            "distance_to_goal_m": float(dist_to_goal_m),
            "accident_title": accident_title,
            "accident_event": event,
        }
        if discard_scene:
            payload["discard_scene"] = True
            payload["discard_reason"] = "moving_pedestrian_accident" if is_moving_ped else "accident"
        if ego_state is not None:
            payload["ego_state"] = dict(ego_state)

        self.publish_dataset_control("STOP_ACCIDENT", payload)
        self.scene_collection_active = False
        return None

    def start_scene_collection(self):
        self.discard_scene_collection()
        self.scene_collection_active = True
        self.publish_dataset_control("START_SCENE")
        print("[Dataset] scene start flag sent")
        self.scene_collector = None

    def stop_scene_collection_for_success(self):
        if getattr(self, "scene_collection_active", False):
            self.publish_dataset_control("STOP_SUCCESS")
        scene_dir = getattr(self, "scene_collection_dir", None)
        self.scene_collection_active = False
        self.scene_collector = None
        self.scene_collection_dir = None
        self.scene_collection_name = None
        return scene_dir

    def discard_scene_collection(self):
        if getattr(self, "scene_collection_active", False):
            self.publish_dataset_control(
                "STOP_SUCCESS",
                {
                    "result": "discard_or_restart_without_meta",
                    "discard_scene": True,
                    "discard_reason": "scenario_interrupted_or_restarted",
                },
            )
        self.scene_collection_active = False
        self.scene_collector = None
        self.scene_collection_dir = None
        self.scene_collection_name = None

    def ensure_route_bev_visualizer(self):
        if not self.cfg.get("route_bev_visualizer_enabled", True):
            return

        if getattr(self, "route_bev_visualizer", None) is None:
            grpc_cfg = self.global_cfg.get("grpc", {})
            path_cfg = self.global_cfg.get("paths", {})
            state_source = self.cfg.get("route_bev_visualizer_state_source", "grpc")
            self.route_bev_visualizer = RouteBEVVisualizerController(
                workspace_root=WORKSPACE_ROOT,
                runner_root=RUNNER_ROOT,
                route_json=self.route_bev_route_json_path(),
                use_imshow=self.cfg.get("route_bev_visualizer_use_imshow", True),
                publish_image=(
                    self.cfg.get("route_bev_visualizer_publish_image", True)
                    and str(state_source).lower() == "ros"
                ),
                window_title=self.cfg.get(
                    "route_bev_visualizer_window_title",
                    "AIM Route",
                ),
                state_source=state_source,
                grpc_host=grpc_cfg.get("host", "127.0.0.1"),
                grpc_port=grpc_cfg.get("port", 7789),
                grpc_client_key=grpc_cfg.get("client_key", "aim_scenario_runner"),
                grpc_src=path_cfg.get(
                    "grpc_src",
                    os.path.join(RUNNER_ROOT, "grpc_inha_univ", "src"),
                ),
                python_executable=self.cfg.get("route_bev_visualizer_python"),
            )

        if not self.route_bev_visualizer.is_running():
            self.route_bev_visualizer.start()

    def stop_route_bev_visualizer_controller(self):
        if getattr(self, "route_bev_visualizer", None) is not None:
            self.route_bev_visualizer.stop()
            self.route_bev_visualizer = None

    def get_route_road_group(self):
        lookup = getattr(self, "zone_link_group_lookup", {})
        start_link = getattr(self, "start_link", None)
        if start_link in lookup:
            return lookup[start_link]
        for link_id in getattr(self, "route_links", []):
            if link_id in lookup:
                return lookup[link_id]
        return "default"

    def setup_npc_vehicle_manager(self):
        npc_cfg = self.cfg.get("npc", {})
        if not npc_cfg.get("enabled", False):
            return
        npc_cfg = self.build_npc_runtime_cfg(npc_cfg)

        if getattr(self, "npc_vehicle_manager", None) is None:
            self.npc_vehicle_manager = NpcVehicleManager(
                sim_bridge=self.grpc,
                cfg=npc_cfg,
                rng=self.random,
                map_loader=self.map_loader,
            )

        self.npc_vehicle_manager.set_route(
            route_points=self.route_points,
            route_links=self.route_links,
            road_group=self.get_route_road_group(),
            route_link_spans=self.build_route_link_spans(self.route_links),
        )
        self.npc_vehicles = self.npc_vehicle_manager.live_npcs

    def get_ego_target_speed_for_npc(self):
        drive_control_mode = str(self.cfg.get("drive_control_mode", "morai_cruise")).lower()
        if drive_control_mode in ("pure_pursuit", "ros_pure_pursuit"):
            return float(self.cfg.get("pure_pursuit_target_speed", self.cfg.get("constant_velocity", 12.0)))
        return float(self.cfg.get("constant_velocity", self.cfg.get("pure_pursuit_target_speed", 12.0)))

    def build_npc_runtime_cfg(self, npc_cfg):
        cfg = dict(npc_cfg)
        speed_cfg = dict(cfg.get("speed", {}))
        cfg["speed"] = speed_cfg
        return cfg

    def update_npc_vehicle_manager(self, ego_s, ego_state=None):
        manager = getattr(self, "npc_vehicle_manager", None)
        if manager is None:
            return
        manager.update(ego_s=ego_s, ego_state=ego_state)
        self.npc_vehicles = manager.live_npcs

    def spawn_initial_npc_vehicles(self):
        manager = getattr(self, "npc_vehicle_manager", None)
        if manager is None:
            return
        ego_s = self.get_initial_ego_route_s()
        print(f"[NPC] initial ego route_s={ego_s:.1f}m")
        manager.spawn_initial_npcs(
            ego_s=ego_s,
            ego_state={"x": self.start_x, "y": self.start_y, "yaw_deg": self.start_yaw},
        )
        self.npc_vehicles = manager.live_npcs

    def setup_pedestrian_manager(self):
        ped_cfg = self.cfg.get("pedestrians", {})
        if not ped_cfg.get("enabled", False):
            return

        if getattr(self, "pedestrian_manager", None) is None:
            self.pedestrian_manager = PedestrianManager(
                sim_bridge=self.grpc,
                cfg=ped_cfg,
                rng=self.random,
                map_loader=self.map_loader,
            )

        self.pedestrian_manager.set_route(
            route_points=self.route_points,
            route_links=self.route_links,
            route_link_spans=self.build_route_link_spans(self.route_links),
        )
        self.pedestrians = self.pedestrian_manager.live_pedestrians

    def update_pedestrian_manager(self, ego_s, ego_state=None):
        manager = getattr(self, "pedestrian_manager", None)
        if manager is None:
            return
        manager.update(ego_s=ego_s, ego_state=ego_state)
        self.pedestrians = manager.live_pedestrians

    def spawn_initial_pedestrians(self):
        manager = getattr(self, "pedestrian_manager", None)
        if manager is None:
            return
        ego_s = self.get_initial_ego_route_s()
        print(f"[Ped] initial ego route_s={ego_s:.1f}m")
        manager.update(
            ego_s=ego_s,
            ego_state={"x": self.start_x, "y": self.start_y, "yaw_deg": self.start_yaw},
            fill_all=True,
        )
        self.pedestrians = manager.live_pedestrians

    def hold_ego_stopped_before_start(self, phase="pre_start"):
        if not hasattr(self.grpc, "hold_ego_stopped"):
            return
        duration_sec = float(self.cfg.get("ego_pre_start_hold_sec", 0.8))
        interval_sec = float(self.cfg.get("ego_pre_start_hold_interval_sec", 0.05))
        if duration_sec <= 0.0:
            return
        print(
            f"[UrbanBasicDrive] holding ego stopped phase={phase} "
            f"duration={duration_sec:.2f}s"
        )
        self.grpc.hold_ego_stopped(
            duration_sec=duration_sec,
            interval_sec=interval_sec,
        )

    def get_initial_ego_route_s(self):
        try:
            ego_s, _, _, _ = self.project_on_route_near_progress(
                self.start_x,
                self.start_y,
                prev_s=None,
            )
            return ego_s
        except Exception as exc:
            print(f"[NPC] failed to project initial ego pose on route: {exc}")

        try:
            return project_distance_on_polyline(self.route_points, self.start_x, self.start_y)
        except Exception as exc:
            print(f"[NPC] failed to fallback project initial ego pose: {exc}")
            return 0.0

    def reset_npc_vehicle_manager_refs(self):
        manager = getattr(self, "npc_vehicle_manager", None)
        if manager is not None:
            manager.reset_live_refs()
        self.npc_vehicles = []

    def reset_pedestrian_manager_refs(self):
        manager = getattr(self, "pedestrian_manager", None)
        if manager is not None:
            manager.reset_live_refs()
        self.pedestrians = []

    def stop_npc_vehicle_manager(self):
        manager = getattr(self, "npc_vehicle_manager", None)
        if manager is not None:
            manager.destroy_all()
        self.npc_vehicle_manager = None
        self.npc_vehicles = []

    def stop_pedestrian_manager(self):
        manager = getattr(self, "pedestrian_manager", None)
        if manager is not None:
            manager.destroy_all()
        self.pedestrian_manager = None
        self.pedestrians = []

    def build_route_points(self, route_links):
        points = []
        for link_id in route_links:
            link_points = self.map_loader.get_link_points(link_id)
            if points and link_points:
                points.extend(link_points[1:])
            else:
                points.extend(link_points)
        return points

    def build_route_link_spans(self, route_links):
        spans = []
        cursor = 0.0
        for link_id in route_links:
            points = self.map_loader.get_link_points(link_id)
            length_m = polyline_length(points)
            spans.append(
                {
                    "link_id": link_id,
                    "start_s": cursor,
                    "end_s": cursor + length_m,
                }
            )
            cursor += length_m
        return spans

    def max_route_point_gap(self, points):
        if len(points) < 2:
            return 0.0

        return max(
            dist_xy(p0[0], p0[1], p1[0], p1[1])
            for p0, p1 in zip(points[:-1], points[1:])
        )

    def dump_route_debug(self):
        debug_dir = runner_relative_path(self.cfg.get("route_debug_dir", "debug_routes"))
        os.makedirs(debug_dir, exist_ok=True)

        route_id = time.strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(debug_dir, f"random_route_drive_route_{route_id}.csv")
        png_path = os.path.join(debug_dir, f"random_route_drive_route_{route_id}.png")

        with open(csv_path, "w") as f:
            f.write("idx,x,y,z\n")
            for i, point in enumerate(self.route_points):
                z = point[2] if len(point) >= 3 else 0.0
                f.write(f"{i},{point[0]},{point[1]},{z}\n")

        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(figsize=(10, 10))
            for link_id in self.zone_allowed_links:
                points = self.map_loader.get_link_points(link_id)
                xs = [p[0] for p in points]
                ys = [p[1] for p in points]
                ax.plot(xs, ys, color="#bbbbbb", linewidth=0.7, alpha=0.7)

            xs = [p[0] for p in self.route_points]
            ys = [p[1] for p in self.route_points]
            ax.plot(xs, ys, color="#d62728", linewidth=2.5, marker=".", markersize=3)
            ax.scatter([xs[0]], [ys[0]], color="#2ca02c", s=80, label="start")
            ax.scatter([self.goal_x], [self.goal_y], color="#1f77b4", s=80, label="goal")
            ax.set_aspect("equal", adjustable="box")
            ax.set_title(
                f"{self.start_link} -> {self.end_link}, "
                f"{len(self.route_links)} links, {self.route_length_m:.1f}m"
            )
            ax.legend()
            ax.grid(True, linewidth=0.3)
            fig.tight_layout()
            fig.savefig(png_path, dpi=150)
            plt.close(fig)
            print(f"[UrbanBasicDrive] route debug png: {png_path}")
        except Exception as e:
            print(f"[UrbanBasicDrive] route debug plot skipped: {e}")

        print(f"[UrbanBasicDrive] route debug csv: {csv_path}")

    def build_route_waypoint_indices(self):
        waypoint_indices = []
        for link_id in self.route_links:
            points = self.map_loader.get_link_points(link_id)
            waypoint_indices.append(len(points) - 1)

        if self.route_links:
            end_points = self.map_loader.get_link_points(self.route_links[-1])
            waypoint_indices[-1] = nearest_point_index(end_points, self.goal_x, self.goal_y)

        return waypoint_indices

    def configure_drive_from_start(self):
        """
        현재 MORAI world에서 Ego를 시작점에 두고 route/cruise 설정.
        """
        print("[UrbanBasicDrive] configure drive from start")
        self.route_configured = False
        self.route_setup_mode = None
        self.ensure_route_bev_visualizer()

        start_settle_sec = float(self.cfg.get("ego_start_settle_sec", 0.2))
        if hasattr(self.grpc, "place_ego_stopped"):
            self.grpc.place_ego_stopped(self.start_tf, settle_sec=start_settle_sec)
        else:
            self.grpc.set_ego_transform(self.start_tf)
            time.sleep(start_settle_sec)

        drive_control_mode = str(self.cfg.get("drive_control_mode", "morai_cruise")).lower()
        if drive_control_mode in ("pure_pursuit", "ros_pure_pursuit"):
            if hasattr(self.grpc, "stop_ego_cruise"):
                self.grpc.stop_ego_cruise()
            auto_ok = self.grpc.set_ego_control_mode_auto()
            if hasattr(self.grpc, "set_ego_gear_drive"):
                self.grpc.set_ego_gear_drive()
            if drive_control_mode == "ros_pure_pursuit":
                self.init_ros_ctrl_cmd_publisher()
                self.publish_ros_route_path()
            self.route_configured = auto_ok
            self.route_setup_mode = drive_control_mode
            print(f"[UrbanBasicDrive] using {drive_control_mode} low-level route following")
            return auto_ok

        use_ego_destination = self.cfg.get("use_ego_destination", False)
        if use_ego_destination and hasattr(self.grpc, "set_ego_destination"):
            self.grpc.set_ego_destination(
                self.goal_x,
                self.goal_y,
                self.goal_z,
                decision_range=self.decision_range,
            )
        else:
            print("[UrbanBasicDrive] set_vehicle_destination skipped; using explicit route links only")

        route_ok = False
        use_route_waypoints = self.cfg.get("use_route_waypoints", False)
        if use_route_waypoints:
            route_ok = self.grpc.set_ego_route(
                self.route_links,
                decision_range=self.decision_range,
                waypoint_indices=self.route_waypoint_indices,
            )
            if route_ok:
                self.route_setup_mode = "waypoints"

        if not route_ok:
            if use_route_waypoints:
                print("[UrbanBasicDrive] waypoint route rejected; retrying plain route")
            route_ok = self.grpc.set_ego_route(
                self.route_links,
                decision_range=self.decision_range,
            )
            if route_ok:
                self.route_setup_mode = "plain"

        if not route_ok:
            print("[UrbanBasicDrive] route setup failed; cruise will not start")
            return False

        cruise_mode_ok = self.grpc.set_ego_control_mode_cruise()
        self.route_configured = bool(route_ok and cruise_mode_ok)
        return self.route_configured

    def start_configured_drive(self):
        if self.route_setup_mode in ("plain", "waypoints"):
            cruise_ok = self.grpc.set_ego_cruise(
                enable=True,
                link_speed_ratio=self.cfg.get("link_speed_ratio", 40),
                constant_velocity=self.cfg.get("constant_velocity", 20),
                cruise_type=self.cfg.get("cruise_type", "link"),
            )
            self.route_configured = cruise_ok
            return cruise_ok
        return True

    def configure_drive_with_retries(self):
        attempts = int(self.cfg.get("route_setup_attempts", 5))

        for attempt in range(1, attempts + 1):
            if self.configure_drive_from_start():
                if attempt > 1:
                    print(f"[UrbanBasicDrive] route setup recovered on attempt={attempt}")
                self.hold_ego_stopped_before_start(phase="before_npc_spawn")
                self.setup_npc_vehicle_manager()
                self.spawn_initial_npc_vehicles()
                self.setup_pedestrian_manager()
                self.spawn_initial_pedestrians()
                self.hold_ego_stopped_before_start(phase="before_drive_start")
                self.start_scene_collection()
                self.start_configured_drive()
                return

            print(f"[UrbanBasicDrive] route setup retry {attempt}/{attempts}")
            if self.randomize_links:
                self.select_route_for_next_drive()
            time.sleep(0.5)

        raise RuntimeError(f"Failed to configure MORAI route after {attempts} attempts")

    def restart_to_start_and_drive(self):
        """
        목표 도착 후 다음 시작/목표를 준비하고 MORAI world 자체를 재시작해 다시 주행 시작.
        teleport만 하면 built-in cruise 내부 route 상태가 남아서 경로가 꼬일 수 있음.
        """
        print("[UrbanBasicDrive] restart to start")
        self.discard_scene_collection()
        self.reset_npc_vehicle_manager_refs()
        self.reset_pedestrian_manager_refs()
        self.reset_accident_detection()

        if hasattr(self.grpc, "stop_ego_motion"):
            self.grpc.stop_ego_motion(settle_sec=0.0)
        self.select_route_for_next_drive()
        self.grpc.restart_world(self.start_tf)
        time.sleep(0.5)

        self.configure_drive_with_retries()

    def run_timeline(self):
        if getattr(self, "dataset_limit_reached_before_start", False):
            return

        if self.route_setup_mode in ("pure_pursuit", "ros_pure_pursuit"):
            self.run_pure_pursuit_timeline()
            return

        timeout_sec = self.cfg.get("timeout_sec", 0.0)
        use_timeout = timeout_sec is not None and float(timeout_sec) > 0.0

        goal_tolerance_m = self.cfg.get("goal_tolerance_m", 10.0)
        check_period_sec = self.cfg.get("check_period_sec", 0.2)
        restart_on_route_complete = self.cfg.get("restart_on_morai_route_complete", True)
        route_complete_goal_tolerance_m = self.cfg.get(
            "route_complete_goal_tolerance_m",
            max(goal_tolerance_m, 15.0),
        )
        complete_on_end_link = self.cfg.get("complete_on_end_link", True)
        end_link_hold_sec = float(self.cfg.get("end_link_hold_sec", 0.5))
        off_route_timeout_sec = float(self.cfg.get("off_route_timeout_sec", 3.0))
        off_route_grace_sec = float(self.cfg.get("off_route_grace_sec", 2.0))

        # 0이면 무한 반복
        max_laps = int(self.cfg.get("max_laps", 0))

        timeout_msg = f"{timeout_sec}s" if use_timeout else "disabled"
        max_laps_msg = "infinite" if max_laps <= 0 else str(max_laps)

        print(
            f"[UrbanBasicDrive] running until goal. "
            f"timeout={timeout_msg}, tolerance={goal_tolerance_m}m, max_laps={max_laps_msg}"
        )

        lap = 0
        lap_start_time = time.time()
        last_print_time = 0.0
        end_link_enter_time = None
        off_route_enter_time = None

        while True:
            elapsed = time.time() - lap_start_time

            ego_state = None
            if hasattr(self.grpc, "get_ego_motion_state"):
                try:
                    ego_state = self.grpc.get_ego_motion_state()
                except Exception:
                    ego_state = None
            if ego_state is not None:
                ego_x = ego_state["x"]
                ego_y = ego_state["y"]
            else:
                ego_x, ego_y = self.grpc.get_ego_xy()
                ego_state = {"x": ego_x, "y": ego_y}
            dist_to_goal = dist_xy(ego_x, ego_y, self.goal_x, self.goal_y)
            try:
                current_s, _, _, _ = self.project_on_route_near_progress(ego_x, ego_y)
                self.update_npc_vehicle_manager(current_s, ego_state=ego_state)
                self.update_pedestrian_manager(current_s, ego_state=ego_state)
            except Exception as exc:
                print(f"[UrbanBasicDrive] actor update skipped: {exc}")

            debug_state = None
            if hasattr(self.grpc, "get_ego_state_debug"):
                debug_state = self.grpc.get_ego_state_debug()

            current_link = debug_state["current_link"] if debug_state is not None else None
            on_route = current_link in self.route_links if current_link is not None else True
            on_end_link = current_link == self.end_link
            if on_end_link:
                if end_link_enter_time is None:
                    end_link_enter_time = time.time()
            else:
                end_link_enter_time = None

            if self.route_configured and elapsed >= off_route_grace_sec and not on_route:
                if off_route_enter_time is None:
                    off_route_enter_time = time.time()
            else:
                off_route_enter_time = None

            if elapsed - last_print_time >= 1.0:
                if debug_state is not None:
                    print(
                        f"[UrbanBasicDrive] lap={lap + 1} "
                        f"t={elapsed:.1f}s "
                        f"ego=({ego_x:.2f}, {ego_y:.2f}) "
                        f"link={debug_state['current_link']} "
                        f"route_end={self.route_links[-1]} "
                        f"route_mode={self.route_setup_mode} "
                        f"on_route={on_route} "
                        f"on_end={on_end_link} "
                        f"remain_dist={debug_state['remaining_distance']:.1f} "
                        f"remain_links={debug_state['remaining_link_count']} "
                        f"pass_dest={debug_state['is_pass_des_pos']} "
                        f"dist_to_goal={dist_to_goal:.2f}m"
                    )
                else:
                    print(
                        f"[UrbanBasicDrive] lap={lap + 1} "
                        f"t={elapsed:.1f}s "
                        f"ego=({ego_x:.2f}, {ego_y:.2f}) "
                        f"dist_to_goal={dist_to_goal:.2f}m"
                    )

                last_print_time = elapsed

            if (
                off_route_enter_time is not None
                and time.time() - off_route_enter_time >= off_route_timeout_sec
            ):
                print(
                    f"[UrbanBasicDrive] OFF ROUTE - restart "
                    f"current_link={current_link}, route={self.route_links}, "
                    f"elapsed={elapsed:.1f}s"
                )

                if hasattr(self.grpc, "stop_ego_cruise"):
                    self.grpc.stop_ego_cruise()

                self.restart_to_start_and_drive()
                lap_start_time = time.time()
                last_print_time = 0.0
                end_link_enter_time = None
                off_route_enter_time = None
                continue

            if (
                complete_on_end_link
                and end_link_enter_time is not None
                and time.time() - end_link_enter_time >= end_link_hold_sec
            ):
                lap += 1
                print(
                    f"[UrbanBasicDrive] END LINK REACHED "
                    f"lap={lap}, current_link={current_link}, "
                    f"route_end={self.route_links[-1]}, "
                    f"dist={dist_to_goal:.2f}m, elapsed={elapsed:.1f}s"
                )
                if hasattr(self.grpc, "stop_ego_cruise"):
                    self.grpc.stop_ego_cruise()
                self.save_success_scene(
                    result="END LINK REACHED",
                    lap=lap,
                    elapsed_sec=elapsed,
                    dist_to_goal_m=dist_to_goal,
                    ego_state=debug_state,
                )
                if self.dataset_stop_target_reached(settle=True):
                    break

                if max_laps > 0 and lap >= max_laps:
                    print("[UrbanBasicDrive] max_laps reached. finish scenario.")
                    break

                self.restart_to_start_and_drive()
                lap_start_time = time.time()
                last_print_time = 0.0
                end_link_enter_time = None
                off_route_enter_time = None
                continue

            if (
                restart_on_route_complete
                and self.route_configured
                and debug_state is not None
                and debug_state["is_pass_des_pos"]
            ):
                lap += 1
                reached = dist_to_goal <= route_complete_goal_tolerance_m
                result = "GOAL REACHED" if reached else "MORAI ROUTE COMPLETE BEFORE GOAL"
                print(
                    f"[UrbanBasicDrive] {result} "
                    f"lap={lap}, current_link={debug_state['current_link']}, "
                    f"route_end={self.route_links[-1]}, "
                    f"dist={dist_to_goal:.2f}m, elapsed={elapsed:.1f}s"
                )
                if hasattr(self.grpc, "stop_ego_cruise"):
                    self.grpc.stop_ego_cruise()
                if reached:
                    self.save_success_scene(
                        result=result,
                        lap=lap,
                        elapsed_sec=elapsed,
                        dist_to_goal_m=dist_to_goal,
                        ego_state=debug_state,
                    )
                    if self.dataset_stop_target_reached(settle=True):
                        break

                if max_laps > 0 and lap >= max_laps:
                    print("[UrbanBasicDrive] max_laps reached. finish scenario.")
                    break

                self.restart_to_start_and_drive()
                lap_start_time = time.time()
                last_print_time = 0.0
                end_link_enter_time = None
                off_route_enter_time = None
                continue

            # 목표점 도착
            if not complete_on_end_link and dist_to_goal <= goal_tolerance_m:
                lap += 1
                print(
                    f"[UrbanBasicDrive] GOAL REACHED "
                    f"lap={lap}, dist={dist_to_goal:.2f}m, elapsed={elapsed:.1f}s"
                )
                if hasattr(self.grpc, "stop_ego_cruise"):
                    self.grpc.stop_ego_cruise()
                self.save_success_scene(
                    result="GOAL REACHED",
                    lap=lap,
                    elapsed_sec=elapsed,
                    dist_to_goal_m=dist_to_goal,
                    extra={"success_trigger": "goal_tolerance"},
                )
                if self.dataset_stop_target_reached(settle=True):
                    break

                if max_laps > 0 and lap >= max_laps:
                    print("[UrbanBasicDrive] max_laps reached. finish scenario.")
                    break

                # world 자체를 재시작해서 route/cruise 상태 초기화
                self.restart_to_start_and_drive()
                lap_start_time = time.time()
                last_print_time = 0.0
                end_link_enter_time = None
                off_route_enter_time = None
                continue

            # timeout
            if use_timeout and elapsed >= float(timeout_sec):
                print(
                    f"[UrbanBasicDrive] TIMEOUT "
                    f"lap={lap + 1}, dist={dist_to_goal:.2f}m, elapsed={elapsed:.1f}s"
                )
                break

            time.sleep(check_period_sec)

    def cleanup(self):
        self.discard_scene_collection()
        self.stop_npc_vehicle_manager()
        self.stop_pedestrian_manager()
        self.stop_route_bev_visualizer_controller()

    def normalize_angle_rad(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def world_to_ego_xy(self, x, y, ego_pose):
        ego_x, ego_y, ego_yaw_deg = ego_pose
        dx = float(x) - float(ego_x)
        dy = float(y) - float(ego_y)
        yaw = math.radians(float(ego_yaw_deg))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        forward = dx * cos_yaw + dy * sin_yaw
        left = -dx * sin_yaw + dy * cos_yaw
        return forward, left

    def init_ros_ctrl_cmd_publisher(self):
        if hasattr(self, "ros_ctrl_cmd_pub"):
            return

        import rospy
        from morai_msgs.msg import CtrlCmd

        if not rospy.core.is_initialized():
            rospy.init_node("aim_scenario_runner_random_route_drive", anonymous=True, disable_signals=True)

        topic = self.cfg.get("ros_ctrl_cmd_topic", "/ctrl_cmd_0")
        self.ros_ctrl_cmd_msg_type = CtrlCmd
        self.ros_ctrl_cmd_pub = rospy.Publisher(topic, CtrlCmd, queue_size=1)
        self.init_ros_route_path_publisher(rospy)
        time.sleep(0.2)
        print(f"[UrbanBasicDrive] ROS CtrlCmd publisher ready: {topic}")

    def init_ros_route_path_publisher(self, rospy):
        if hasattr(self, "ros_route_path_pub"):
            return

        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped

        topic = self.cfg.get("ros_route_path_topic", "/scenario_route_path")
        self.ros_path_msg_type = Path
        self.ros_pose_stamped_msg_type = PoseStamped
        self.ros_route_path_pub = rospy.Publisher(topic, Path, queue_size=1, latch=True)
        print(f"[UrbanBasicDrive] ROS route Path publisher ready: {topic}")

    def publish_ros_route_path(self):
        if not hasattr(self, "ros_route_path_pub"):
            return

        import rospy

        path_msg = self.ros_path_msg_type()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.cfg.get("ros_route_frame_id", "map")

        for point in self.route_points:
            pose = self.ros_pose_stamped_msg_type()
            pose.header = path_msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2]) if len(point) >= 3 else 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.ros_route_path_pub.publish(path_msg)
        print(
            f"[UrbanBasicDrive] published route Path: "
            f"{self.cfg.get('ros_route_path_topic', '/scenario_route_path')} "
            f"points={len(path_msg.poses)}"
        )

    def publish_ros_ctrl_cmd(self, steer, target_speed, current_speed, brake_override=None):
        cmd = self.ros_ctrl_cmd_msg_type()
        long_cmd_type = int(self.cfg.get("ros_ctrl_cmd_longitudinal_type", 1))
        cmd.longlCmdType = long_cmd_type
        cmd.steering = float(steer)

        if brake_override is not None:
            cmd.accel = 0.0
            cmd.brake = float(brake_override)
            cmd.velocity = 0.0
            self.ros_ctrl_cmd_pub.publish(cmd)
            return

        if long_cmd_type == 2:
            cmd.velocity = float(target_speed)
            cmd.accel = 0.0
            cmd.brake = 0.0
            self.ros_ctrl_cmd_pub.publish(cmd)
            return

        kp = float(self.cfg.get("pure_pursuit_speed_kp", 0.3))
        speed_error = float(target_speed) - float(current_speed)
        accel_cmd = kp * speed_error
        if accel_cmd >= 0.0:
            cmd.accel = self.clamp(accel_cmd, 0.0, 1.0)
            cmd.brake = 0.0
        else:
            cmd.accel = 0.0
            cmd.brake = self.clamp(-accel_cmd, 0.0, 1.0)

        self.ros_ctrl_cmd_pub.publish(cmd)

    def stop_pure_pursuit_control(self):
        if self.route_setup_mode == "pure_pursuit":
            self.grpc.stop_ego_control()
        elif self.route_setup_mode == "ros_pure_pursuit" and hasattr(self, "ros_ctrl_cmd_pub"):
            self.publish_ros_ctrl_cmd(steer=0.0, target_speed=0.0, current_speed=0.0, brake_override=1.0)
        elif self.route_setup_mode in ("plain", "waypoints"):
            self.grpc.stop_ego_cruise()
            self.grpc.set_ego_velocity(0.0)
        else:
            self.grpc.stop_ego_motion(settle_sec=0.0)

    def is_npc_state_recent(self, npc, now=None):
        if not getattr(npc, "has_first_state", False):
            return False
        if now is None:
            now = time.time()
        max_state_age_sec = float(self.cfg.get("npc_follow_max_state_age_sec", 1.0))
        return now - float(getattr(npc, "last_state_time", 0.0)) <= max_state_age_sec

    def update_fast_traffic_light(self, ego_state):
        mode = str(self.cfg.get("traffic_light_mode", "general")).strip().lower()
        if mode != "fast":
            return

        tl_id = str(ego_state.get("tl_id") or "").strip()
        if not tl_id:
            return

        now = time.time()
        interval = float(self.cfg.get("traffic_light_fast_update_interval_sec", 0.5))
        if (
            tl_id == self.last_fast_traffic_light_id
            and interval > 0.0
            and now - self.last_fast_traffic_light_time < interval
        ):
            return

        color = self.cfg.get("traffic_light_fast_color", "G_WITH_GLEFT")
        impulse = bool(self.cfg.get("traffic_light_fast_impulse", False))
        sibling = bool(self.cfg.get("traffic_light_fast_set_sibling", True))
        ok = self.grpc.set_traffic_light_state(
            tl_id,
            color=color,
            impulse=impulse,
            sibling=sibling,
            quiet=True,
        )
        self.last_fast_traffic_light_id = tl_id
        self.last_fast_traffic_light_time = now
        if ok:
            print(
                f"[TrafficLight] fast mode green tl_id={tl_id} "
                f"color={color} ego_color={ego_state.get('tl_color')}"
            )

    def project_on_route_near_progress(self, x, y, prev_s=None):
        if len(self.route_points) < 2:
            raise ValueError("Route must have at least 2 points")

        lookahead_m = float(self.cfg.get("pure_pursuit_lookahead_m", 8.0))
        back_window_m = float(self.cfg.get("pure_pursuit_projection_back_window_m", 8.0))
        front_window_m = float(
            self.cfg.get(
                "pure_pursuit_projection_front_window_m",
                max(40.0, lookahead_m * 5.0),
            )
        )

        min_s = 0.0
        max_s = float("inf")
        if prev_s is not None:
            min_s = max(0.0, prev_s - back_window_m)
            max_s = min(self.route_length_m, prev_s + front_window_m)

        best_s = 0.0
        best_x = self.route_points[0][0]
        best_y = self.route_points[0][1]
        best_dist = float("inf")
        cumulative = 0.0

        for p0, p1 in zip(self.route_points[:-1], self.route_points[1:]):
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-9:
                continue

            seg_len = math.sqrt(seg_len_sq)
            seg_start_s = cumulative
            seg_end_s = cumulative + seg_len
            cumulative = seg_end_s

            if seg_end_s < min_s or seg_start_s > max_s:
                continue

            t = ((x - p0[0]) * dx + (y - p0[1]) * dy) / seg_len_sq
            t = max(0.0, min(1.0, t))
            proj_s = seg_start_s + t * seg_len
            if proj_s < min_s or proj_s > max_s:
                continue

            proj_x = p0[0] + t * dx
            proj_y = p0[1] + t * dy
            dist = dist_xy(x, y, proj_x, proj_y)
            if dist < best_dist:
                best_dist = dist
                best_s = proj_s
                best_x = proj_x
                best_y = proj_y

        if best_dist == float("inf"):
            return self.project_on_route_near_progress(x, y, prev_s=None)

        return best_s, best_x, best_y, best_dist

    def compute_pure_pursuit_command(self, ego_state):
        x = ego_state["x"]
        y = ego_state["y"]
        yaw_rad = math.radians(ego_state["yaw_deg"])

        lookahead_m = float(self.cfg.get("pure_pursuit_lookahead_m", 8.0))
        wheel_base_m = float(self.cfg.get("pure_pursuit_wheel_base_m", 2.9))
        max_steer_rad = math.radians(float(self.cfg.get("pure_pursuit_max_steer_deg", 35.0)))
        target_speed = float(self.cfg.get("pure_pursuit_target_speed", 8.0))
        min_speed = float(self.cfg.get("pure_pursuit_min_speed", min(target_speed, 3.0)))
        slowdown_distance_m = float(self.cfg.get("pure_pursuit_slowdown_distance_m", 15.0))

        prev_s = getattr(self, "pure_pursuit_last_s", None)
        current_s, nearest_x, nearest_y, cross_track_error = self.project_on_route_near_progress(
            x,
            y,
            prev_s=prev_s,
        )
        if prev_s is not None:
            monotonic_s = max(prev_s, current_s)
            if monotonic_s != current_s:
                current_s = monotonic_s
                nearest_x, nearest_y, _, _ = interpolate_on_polyline(self.route_points, current_s)
                cross_track_error = dist_xy(x, y, nearest_x, nearest_y)
        self.pure_pursuit_last_s = current_s

        target_s = min(current_s + lookahead_m, self.route_length_m)
        target_x, target_y, _, _ = interpolate_on_polyline(self.route_points, target_s)

        dx = target_x - x
        dy = target_y - y
        target_dist = max(1e-3, math.hypot(dx, dy))
        alpha = self.normalize_angle_rad(math.atan2(dy, dx) - yaw_rad)
        steer_angle = math.atan2(2.0 * wheel_base_m * math.sin(alpha), target_dist)
        turn_delta_deg = 0.0
        passed_turn_deg = 0.0
        entry_turn_deg = 0.0
        steer_gain = 1.0
        is_corner_entry = False
        corner_phase = "normal"
        if bool(self.cfg.get("pure_pursuit_mid_corner_steer_boost_enabled", True)):
            back_m = float(self.cfg.get("pure_pursuit_mid_corner_back_m", 6.0))
            ahead_m = float(self.cfg.get("pure_pursuit_mid_corner_ahead_m", 10.0))
            turn_threshold_deg = float(
                self.cfg.get("pure_pursuit_mid_corner_turn_threshold_deg", 25.0)
            )
            passed_threshold_deg = float(
                self.cfg.get("pure_pursuit_mid_corner_passed_turn_threshold_deg", 12.0)
            )
            entry_suppress_threshold_deg = float(
                self.cfg.get("pure_pursuit_mid_corner_entry_suppress_threshold_deg", 20.0)
            )
            alpha_threshold_deg = float(
                self.cfg.get("pure_pursuit_mid_corner_alpha_threshold_deg", 5.0)
            )
            behind_s = max(0.0, current_s - max(0.0, back_m))
            ahead_s = min(self.route_length_m, current_s + max(0.0, ahead_m))
            try:
                _, _, _, behind_yaw = interpolate_on_polyline(self.route_points, behind_s)
                _, _, _, current_route_yaw = interpolate_on_polyline(
                    self.route_points,
                    current_s,
                )
                _, _, _, ahead_yaw = interpolate_on_polyline(self.route_points, ahead_s)
                turn_delta_deg = abs(
                    math.degrees(
                        self.normalize_angle_rad(math.radians(ahead_yaw - behind_yaw))
                    )
                )
                passed_turn_deg = abs(
                    math.degrees(
                        self.normalize_angle_rad(
                            math.radians(current_route_yaw - behind_yaw)
                        )
                    )
                )
                entry_turn_deg = abs(
                    math.degrees(
                        self.normalize_angle_rad(
                            math.radians(ahead_yaw - current_route_yaw)
                        )
                    )
                )
            except Exception:
                turn_delta_deg = 0.0
                passed_turn_deg = 0.0
                entry_turn_deg = 0.0
            is_corner_entry = (
                entry_turn_deg >= entry_suppress_threshold_deg
                and passed_turn_deg < passed_threshold_deg
            )
            if is_corner_entry:
                corner_phase = "entry"
                entry_lookahead_m = float(
                    self.cfg.get("pure_pursuit_corner_entry_lookahead_m", 6.0)
                )
                entry_steer_gain = float(
                    self.cfg.get("pure_pursuit_corner_entry_steer_gain", 0.75)
                )
                lookahead_m = min(lookahead_m, max(1.0, entry_lookahead_m))
                target_s = min(current_s + lookahead_m, self.route_length_m)
                target_x, target_y, _, _ = interpolate_on_polyline(
                    self.route_points,
                    target_s,
                )
                dx = target_x - x
                dy = target_y - y
                target_dist = max(1e-3, math.hypot(dx, dy))
                alpha = self.normalize_angle_rad(math.atan2(dy, dx) - yaw_rad)
                steer_angle = math.atan2(
                    2.0 * wheel_base_m * math.sin(alpha),
                    target_dist,
                )
                steer_gain = entry_steer_gain
                steer_angle *= steer_gain
            elif (
                turn_delta_deg >= turn_threshold_deg
                and passed_turn_deg >= passed_threshold_deg
                and abs(math.degrees(alpha)) >= alpha_threshold_deg
            ):
                corner_phase = "mid"
                steer_gain = float(self.cfg.get("pure_pursuit_mid_corner_steer_gain", 1.25))
                steer_angle *= steer_gain
        steer_angle = self.clamp(steer_angle, -max_steer_rad, max_steer_rad)

        steer_mode = str(self.cfg.get("pure_pursuit_steer_mode", "angle")).lower()
        if steer_mode == "normalized":
            steer_cmd = steer_angle / max_steer_rad if max_steer_rad > 1e-6 else 0.0
        else:
            steer_cmd = steer_angle
        steer_cmd *= float(self.cfg.get("pure_pursuit_steer_sign", 1.0))
        steer_cmd = self.clamp(steer_cmd, -1.0, 1.0)

        remaining_s = max(0.0, self.route_length_m - current_s)
        if slowdown_distance_m > 0.0 and remaining_s < slowdown_distance_m:
            speed_ratio = max(0.0, remaining_s / slowdown_distance_m)
            target_speed = max(min_speed, target_speed * speed_ratio)

        return {
            "steer": steer_cmd,
            "target_speed": target_speed,
            "target_x": target_x,
            "target_y": target_y,
            "current_s": current_s,
            "target_s": target_s,
            "remaining_s": remaining_s,
            "cross_track_error": cross_track_error,
            "alpha_deg": math.degrees(alpha),
            "lookahead_m": lookahead_m,
            "turn_delta_deg": turn_delta_deg,
            "passed_turn_deg": passed_turn_deg,
            "entry_turn_deg": entry_turn_deg,
            "steer_gain": steer_gain,
            "corner_phase": corner_phase,
        }

    def apply_npc_follow_brake(self, command, ego_state):
        command["unblocked_target_speed"] = float(command.get("target_speed", 0.0))
        if not bool(self.cfg.get("enable_npc_follow_brake", True)):
            command["brake"] = 0.0
            command["lead_npc"] = None
            command["lead_gap_m"] = None
            return command

        ego_s = float(command["current_s"])
        ego_x = float(ego_state["x"])
        ego_y = float(ego_state["y"])
        ego_yaw = math.radians(float(ego_state["yaw_deg"]))
        cos_yaw = math.cos(ego_yaw)
        sin_yaw = math.sin(ego_yaw)

        detect_distance = float(self.cfg.get("npc_follow_detect_distance_m", 45.0))
        slow_distance = float(self.cfg.get("npc_follow_slow_distance_m", 30.0))
        brake_distance = float(self.cfg.get("npc_follow_brake_distance_m", 12.0))
        lane_width = float(self.cfg.get("npc_follow_same_lane_lateral_m", 3.0))
        max_brake = float(self.cfg.get("npc_follow_max_brake", 1.0))
        min_speed = float(self.cfg.get("npc_follow_min_speed", 0.0))
        ego_front_offset = float(self.cfg.get("npc_follow_ego_front_offset_m", 3.7))
        npc_rear_offset = float(self.cfg.get("npc_follow_npc_rear_offset_m", 1.0))
        stop_ttc_sec = float(self.cfg.get("npc_follow_stop_ttc_sec", 1.0))
        current_speed_mps = max(0.0, float(ego_state.get("speed", 0.0)) / 3.6)
        now = time.time()

        lead = None
        lead_gap = None
        lead_forward = None
        lead_bumper_gap = None
        for npc in getattr(self, "npc_vehicles", []) or []:
            if getattr(npc, "opposite", False):
                continue
            if not self.is_npc_state_recent(npc, now=now):
                continue

            rel_s = float(npc.route_s) - ego_s
            if rel_s <= 0.0 or rel_s > detect_distance:
                continue

            dx = float(npc.x) - ego_x
            dy = float(npc.y) - ego_y
            forward = dx * cos_yaw + dy * sin_yaw
            left = -dx * sin_yaw + dy * cos_yaw
            if forward <= 0.0 or abs(left) > lane_width:
                continue

            bumper_gap = max(0.0, forward - ego_front_offset - npc_rear_offset)
            if lead_bumper_gap is None or bumper_gap < lead_bumper_gap:
                lead = npc
                lead_gap = rel_s
                lead_forward = forward
                lead_bumper_gap = bumper_gap

        command["brake"] = 0.0
        command["lead_npc"] = getattr(lead, "label", None) if lead is not None else None
        command["lead_gap_m"] = lead_gap
        command["lead_forward_m"] = lead_forward
        command["lead_bumper_gap_m"] = lead_bumper_gap
        if lead is None:
            return command

        target_speed = float(command["target_speed"])
        ttc_sec = None
        if lead_bumper_gap is not None and current_speed_mps > 0.1:
            ttc_sec = lead_bumper_gap / current_speed_mps
        command["lead_ttc_sec"] = ttc_sec

        should_full_brake = lead_bumper_gap <= brake_distance or (
            ttc_sec is not None and ttc_sec <= stop_ttc_sec
        )
        if should_full_brake:
            command["target_speed"] = 0.0
            command["brake"] = max_brake
        elif lead_bumper_gap <= slow_distance:
            ratio = (lead_bumper_gap - brake_distance) / max(1e-3, slow_distance - brake_distance)
            command["target_speed"] = max(min_speed, target_speed * self.clamp(ratio, 0.0, 1.0))
            brake_ratio = 1.0 - self.clamp(ratio, 0.0, 1.0)
            command["brake"] = self.clamp(brake_ratio * max_brake, 0.0, max_brake)

        return command

    def reset_accident_detection(self):
        self.accident_stall_enter_time = None
        self.accident_stall_candidate = None
        self.lead_block_stall_enter_time = None
        self.lead_block_candidate_key = None
        self.front_moving_pedestrian_block_enter_time = None
        self.front_moving_pedestrian_block_candidate_key = None
        self.lead_npc_stall_enter_time = None
        self.lead_npc_stall_candidate_key = None
        self.cross_lane_npc_block_enter_time = None
        self.cross_lane_npc_block_candidate_key = None
        self.static_obstacle_stall_enter_time = None
        self.static_obstacle_start_s = None
        self.static_obstacle_start_x = None
        self.static_obstacle_start_y = None

    def reset_direct_accident_detection(self):
        self.accident_stall_enter_time = None
        self.accident_stall_candidate = None

    def reset_lead_block_detection(self):
        self.lead_block_stall_enter_time = None
        self.lead_block_candidate_key = None

    def reset_front_moving_pedestrian_block_detection(self):
        self.front_moving_pedestrian_block_enter_time = None
        self.front_moving_pedestrian_block_candidate_key = None

    def reset_lead_npc_stalled_block_detection(self):
        self.lead_npc_stall_enter_time = None
        self.lead_npc_stall_candidate_key = None

    def reset_cross_lane_npc_block_detection(self):
        self.cross_lane_npc_block_enter_time = None
        self.cross_lane_npc_block_candidate_key = None

    def reset_static_obstacle_detection(self):
        self.static_obstacle_stall_enter_time = None
        self.static_obstacle_start_s = None
        self.static_obstacle_start_x = None
        self.static_obstacle_start_y = None

    def start_static_obstacle_detection(self, command, ego_state, now):
        self.static_obstacle_stall_enter_time = float(now)
        self.static_obstacle_start_s = float(command.get("current_s", 0.0))
        self.static_obstacle_start_x = float(ego_state["x"])
        self.static_obstacle_start_y = float(ego_state["y"])

    def find_npc_by_label(self, label):
        if not label:
            return None
        label = str(label)
        for npc in getattr(self, "npc_vehicles", []) or []:
            if str(getattr(npc, "label", "")) == label:
                return npc
        return None

    def find_near_pedestrian_to_point(self, x, y, max_distance_m):
        nearest = None
        for ped in getattr(self, "pedestrians", []) or []:
            distance = dist_xy(float(x), float(y), float(ped.x), float(ped.y))
            if distance > max_distance_m:
                continue
            item = {
                "label": getattr(ped, "label", ""),
                "slot": getattr(ped, "behavior", ""),
                "distance_m": float(distance),
                "x": float(ped.x),
                "y": float(ped.y),
                "speed_mps": float(getattr(ped, "walking_speed_mps", 0.0)),
                "initial_speed_mps": float(getattr(ped, "initial_walking_speed_mps", 0.0)),
                "stopped": bool(getattr(ped, "stopped", False)),
                "stop_reason": getattr(ped, "stop_reason", None),
            }
            if nearest is None or item["distance_m"] < nearest["distance_m"]:
                nearest = item
        return nearest

    def find_near_accident_npc(
        self,
        ego_state,
        max_distance_m,
        max_forward_m,
        max_side_m,
    ):
        now = time.time()
        ego_x = float(ego_state["x"])
        ego_y = float(ego_state["y"])
        ego_pose = (
            ego_x,
            ego_y,
            float(ego_state.get("yaw_deg", ego_state.get("yaw", 0.0))),
        )

        nearest = None
        for npc in getattr(self, "npc_vehicles", []) or []:
            if not self.is_npc_state_recent(npc, now=now):
                continue

            distance = dist_xy(ego_x, ego_y, float(npc.x), float(npc.y))
            forward, left = self.world_to_ego_xy(float(npc.x), float(npc.y), ego_pose)
            in_distance = distance <= max_distance_m
            in_front_contact_box = (
                -1.0 <= forward <= max_forward_m
                and abs(left) <= max_side_m
            )
            if not in_distance and not in_front_contact_box:
                continue

            item = {
                "actor_type": "npc",
                "label": getattr(npc, "label", ""),
                "slot": getattr(npc, "slot", ""),
                "opposite": bool(getattr(npc, "opposite", False)),
                "distance_m": float(distance),
                "forward_m": float(forward),
                "left_m": float(left),
                "in_distance": bool(in_distance),
                "in_front_contact_box": bool(in_front_contact_box),
                "speed_mps": float(getattr(npc, "estimated_speed_mps", 0.0)),
                "state_source": getattr(npc, "last_state_source", "unknown"),
            }
            if nearest is None or item["distance_m"] < nearest["distance_m"]:
                nearest = item
        return nearest

    def find_near_accident_pedestrian(
        self,
        ego_state,
        max_distance_m,
        max_forward_m,
        max_side_m,
    ):
        ego_x = float(ego_state["x"])
        ego_y = float(ego_state["y"])
        ego_pose = (
            ego_x,
            ego_y,
            float(ego_state.get("yaw_deg", ego_state.get("yaw", 0.0))),
        )

        nearest = None
        for ped in getattr(self, "pedestrians", []) or []:
            distance = dist_xy(ego_x, ego_y, float(ped.x), float(ped.y))
            forward, left = self.world_to_ego_xy(float(ped.x), float(ped.y), ego_pose)
            in_distance = distance <= max_distance_m
            in_front_contact_box = (
                -1.0 <= forward <= max_forward_m
                and abs(left) <= max_side_m
            )
            if not in_distance and not in_front_contact_box:
                continue

            item = {
                "actor_type": "pedestrian",
                "label": getattr(ped, "label", ""),
                "slot": getattr(ped, "behavior", ""),
                "opposite": False,
                "distance_m": float(distance),
                "forward_m": float(forward),
                "left_m": float(left),
                "in_distance": bool(in_distance),
                "in_front_contact_box": bool(in_front_contact_box),
                "speed_mps": float(getattr(ped, "walking_speed_mps", 0.0)),
                "initial_speed_mps": float(getattr(ped, "initial_walking_speed_mps", 0.0)),
                "stopped": bool(getattr(ped, "stopped", False)),
                "stop_reason": getattr(ped, "stop_reason", None),
                "state_source": "pedestrian_manager",
            }
            if nearest is None or item["distance_m"] < nearest["distance_m"]:
                nearest = item
        return nearest

    def iter_front_npcs_for_moving_pedestrian_check(self, command):
        current_s = command.get("current_s")
        if current_s is None:
            return []

        current_s = float(current_s)
        front_count = int(self.cfg.get("moving_pedestrian_accident_front_npc_count", 3))
        max_front_m = float(self.cfg.get("moving_pedestrian_accident_front_max_m", 60.0))
        now = time.time()
        candidates = []
        for npc in getattr(self, "npc_vehicles", []) or []:
            if bool(getattr(npc, "opposite", False)):
                continue
            if not self.is_npc_state_recent(npc, now=now):
                continue
            rel_s = float(getattr(npc, "route_s", 0.0)) - current_s
            if rel_s < 0.0 or rel_s > max_front_m:
                continue
            candidates.append((rel_s, npc))

        candidates.sort(key=lambda item: item[0])
        return candidates[:max(1, front_count)]

    def detect_front_moving_pedestrian_block(
        self,
        command,
        ego_state,
        desired_command_speed_kmh,
    ):
        if not bool(self.cfg.get("discard_moving_pedestrian_accidents", False)):
            self.reset_front_moving_pedestrian_block_detection()
            return None

        min_command_speed_kmh = float(
            self.cfg.get(
                "moving_pedestrian_accident_min_command_speed_kmh",
                self.cfg.get("accident_min_command_speed_kmh", 5.0),
            )
        )
        if float(desired_command_speed_kmh) < min_command_speed_kmh:
            self.reset_front_moving_pedestrian_block_detection()
            return None

        max_npc_speed_mps = float(
            self.cfg.get(
                "moving_pedestrian_accident_npc_speed_mps",
                self.cfg.get("lead_npc_pedestrian_block_npc_speed_mps", 0.5),
            )
        )
        ped_distance_m = float(
            self.cfg.get(
                "moving_pedestrian_accident_ped_distance_m",
                self.cfg.get("lead_npc_pedestrian_block_ped_distance_m", 3.0),
            )
        )

        now = time.time()
        best = None
        for front_rank, (rel_s, npc) in enumerate(
            self.iter_front_npcs_for_moving_pedestrian_check(command),
            start=1,
        ):
            npc_speed_mps = float(getattr(npc, "estimated_speed_mps", 0.0))
            if npc_speed_mps > max_npc_speed_mps:
                continue

            near_pedestrian = self.find_near_pedestrian_to_point(
                float(npc.x),
                float(npc.y),
                ped_distance_m,
            )
            if near_pedestrian is None:
                continue
            if not self.is_moving_pedestrian_item(near_pedestrian):
                continue

            best = (front_rank, rel_s, npc, npc_speed_mps, near_pedestrian)
            break

        if best is None:
            self.reset_front_moving_pedestrian_block_detection()
            return None

        front_rank, rel_s, npc, npc_speed_mps, near_pedestrian = best
        candidate_key = (str(getattr(npc, "label", "")), str(near_pedestrian["label"]))
        if (
            getattr(self, "front_moving_pedestrian_block_candidate_key", None)
            != candidate_key
        ):
            self.front_moving_pedestrian_block_candidate_key = candidate_key
            self.front_moving_pedestrian_block_enter_time = now

        if getattr(self, "front_moving_pedestrian_block_enter_time", None) is None:
            self.front_moving_pedestrian_block_enter_time = now

        stall_for = now - self.front_moving_pedestrian_block_enter_time
        required_stall_sec = float(
            self.cfg.get("moving_pedestrian_accident_duration_sec", 2.0)
        )
        if stall_for < required_stall_sec:
            return None

        ego_x = float(ego_state["x"])
        ego_y = float(ego_state["y"])
        ego_pose = (
            ego_x,
            ego_y,
            float(ego_state.get("yaw_deg", ego_state.get("yaw", 0.0))),
        )
        forward, left = self.world_to_ego_xy(float(npc.x), float(npc.y), ego_pose)
        distance = dist_xy(ego_x, ego_y, float(npc.x), float(npc.y))

        return {
            "actor_type": "lead_npc_pedestrian_block",
            "label": getattr(npc, "label", ""),
            "slot": getattr(npc, "slot", ""),
            "opposite": bool(getattr(npc, "opposite", False)),
            "distance_m": float(distance),
            "forward_m": float(forward),
            "left_m": float(left),
            "route_delta_m": float(rel_s),
            "front_scan_rank": int(front_rank),
            "in_distance": True,
            "in_front_contact_box": True,
            "speed_mps": float(npc_speed_mps),
            "state_source": getattr(npc, "last_state_source", "unknown"),
            "blocked_by_label": near_pedestrian["label"],
            "blocked_by_slot": near_pedestrian["slot"],
            "blocked_by_distance_m": float(near_pedestrian["distance_m"]),
            "blocked_by_speed_mps": float(near_pedestrian.get("speed_mps", 0.0)),
            "blocked_by_initial_speed_mps": float(
                near_pedestrian.get("initial_speed_mps", 0.0)
            ),
            "blocked_by_stopped": bool(near_pedestrian.get("stopped", False)),
            "blocked_by_stop_reason": near_pedestrian.get("stop_reason"),
            "stall_for_sec": float(stall_for),
            "ego_speed_kmh": float(ego_state.get("speed", 0.0)),
            "command_speed_kmh": float(desired_command_speed_kmh),
            "command_brake": float(command.get("brake", 0.0)),
            "front_moving_pedestrian_discard": True,
        }

    def find_cross_lane_blocking_npc(self, ego_state):
        max_distance_m = float(self.cfg.get("cross_lane_npc_block_distance_m", 7.0))
        forward_min_m = float(self.cfg.get("cross_lane_npc_block_forward_min_m", -2.0))
        forward_max_m = float(self.cfg.get("cross_lane_npc_block_forward_max_m", 10.0))
        max_side_m = float(self.cfg.get("cross_lane_npc_block_side_m", 5.5))
        max_speed_mps = float(self.cfg.get("cross_lane_npc_block_npc_speed_mps", 0.5))
        require_opposite = bool(self.cfg.get("cross_lane_npc_block_require_opposite", True))

        now = time.time()
        ego_x = float(ego_state["x"])
        ego_y = float(ego_state["y"])
        ego_pose = (
            ego_x,
            ego_y,
            float(ego_state.get("yaw_deg", ego_state.get("yaw", 0.0))),
        )

        nearest = None
        for npc in getattr(self, "npc_vehicles", []) or []:
            if require_opposite and not bool(getattr(npc, "opposite", False)):
                continue
            if not self.is_npc_state_recent(npc, now=now):
                continue

            speed_mps = float(getattr(npc, "estimated_speed_mps", 0.0))
            if speed_mps > max_speed_mps:
                continue

            distance = dist_xy(ego_x, ego_y, float(npc.x), float(npc.y))
            forward, left = self.world_to_ego_xy(float(npc.x), float(npc.y), ego_pose)
            in_block_zone = (
                distance <= max_distance_m
                and forward_min_m <= forward <= forward_max_m
                and abs(left) <= max_side_m
            )
            if not in_block_zone:
                continue

            item = {
                "actor_type": "cross_lane_npc_block",
                "label": getattr(npc, "label", ""),
                "slot": getattr(npc, "slot", ""),
                "opposite": bool(getattr(npc, "opposite", False)),
                "distance_m": float(distance),
                "forward_m": float(forward),
                "left_m": float(left),
                "in_distance": True,
                "in_front_contact_box": True,
                "speed_mps": float(speed_mps),
                "state_source": getattr(npc, "last_state_source", "unknown"),
            }
            if nearest is None or item["distance_m"] < nearest["distance_m"]:
                nearest = item

        return nearest

    def detect_lead_npc_pedestrian_block(
        self,
        command,
        ego_state,
        desired_command_speed_kmh,
    ):
        if not bool(self.cfg.get("lead_npc_pedestrian_block_accident_enabled", True)):
            self.reset_lead_block_detection()
            return None

        min_command_speed_kmh = float(
            self.cfg.get(
                "lead_npc_pedestrian_block_min_command_speed_kmh",
                self.cfg.get("accident_min_command_speed_kmh", 5.0),
            )
        )
        if float(desired_command_speed_kmh) < min_command_speed_kmh:
            self.reset_lead_block_detection()
            return None

        lead_label = command.get("lead_npc")
        lead = self.find_npc_by_label(lead_label)
        if lead is None:
            self.reset_lead_block_detection()
            return None

        now = time.time()
        if not self.is_npc_state_recent(lead, now=now):
            self.reset_lead_block_detection()
            return None

        lead_forward = command.get("lead_forward_m")
        if lead_forward is None:
            self.reset_lead_block_detection()
            return None
        lead_forward = float(lead_forward)
        lead_min_m = float(self.cfg.get("lead_npc_pedestrian_block_lead_min_m", 0.0))
        lead_max_m = float(self.cfg.get("lead_npc_pedestrian_block_lead_max_m", 25.0))
        if lead_forward < lead_min_m or lead_forward > lead_max_m:
            self.reset_lead_block_detection()
            return None

        lead_speed_mps = float(getattr(lead, "estimated_speed_mps", 0.0))
        max_lead_speed_mps = float(
            self.cfg.get("lead_npc_pedestrian_block_npc_speed_mps", 0.5)
        )
        if lead_speed_mps > max_lead_speed_mps:
            self.reset_lead_block_detection()
            return None

        ped_distance_m = float(
            self.cfg.get("lead_npc_pedestrian_block_ped_distance_m", 3.0)
        )
        near_pedestrian = self.find_near_pedestrian_to_point(
            float(lead.x),
            float(lead.y),
            ped_distance_m,
        )
        if near_pedestrian is None:
            self.reset_lead_block_detection()
            return None

        ego_x = float(ego_state["x"])
        ego_y = float(ego_state["y"])
        ego_pose = (
            ego_x,
            ego_y,
            float(ego_state.get("yaw_deg", ego_state.get("yaw", 0.0))),
        )
        forward, left = self.world_to_ego_xy(float(lead.x), float(lead.y), ego_pose)
        distance = dist_xy(ego_x, ego_y, float(lead.x), float(lead.y))

        candidate_key = (str(getattr(lead, "label", "")), str(near_pedestrian["label"]))
        if getattr(self, "lead_block_candidate_key", None) != candidate_key:
            self.lead_block_candidate_key = candidate_key
            self.lead_block_stall_enter_time = now

        if getattr(self, "lead_block_stall_enter_time", None) is None:
            self.lead_block_stall_enter_time = now

        stall_for = now - self.lead_block_stall_enter_time
        required_stall_sec = float(
            self.cfg.get("lead_npc_pedestrian_block_duration_sec", 6.0)
        )
        if stall_for < required_stall_sec:
            return None

        return {
            "actor_type": "lead_npc_pedestrian_block",
            "label": getattr(lead, "label", ""),
            "slot": getattr(lead, "slot", ""),
            "opposite": bool(getattr(lead, "opposite", False)),
            "distance_m": float(distance),
            "forward_m": float(forward),
            "left_m": float(left),
            "in_distance": True,
            "in_front_contact_box": True,
            "speed_mps": float(lead_speed_mps),
            "state_source": getattr(lead, "last_state_source", "unknown"),
            "blocked_by_label": near_pedestrian["label"],
            "blocked_by_slot": near_pedestrian["slot"],
            "blocked_by_distance_m": float(near_pedestrian["distance_m"]),
            "blocked_by_speed_mps": float(near_pedestrian.get("speed_mps", 0.0)),
            "blocked_by_initial_speed_mps": float(near_pedestrian.get("initial_speed_mps", 0.0)),
            "blocked_by_stopped": bool(near_pedestrian.get("stopped", False)),
            "blocked_by_stop_reason": near_pedestrian.get("stop_reason"),
            "stall_for_sec": float(stall_for),
            "ego_speed_kmh": float(ego_state.get("speed", 0.0)),
            "command_speed_kmh": float(desired_command_speed_kmh),
            "command_brake": float(command.get("brake", 0.0)),
        }

    def detect_cross_lane_npc_block(
        self,
        command,
        ego_state,
        desired_command_speed_kmh,
    ):
        if not bool(self.cfg.get("cross_lane_npc_block_accident_enabled", True)):
            self.reset_cross_lane_npc_block_detection()
            return None

        min_command_speed_kmh = float(
            self.cfg.get(
                "cross_lane_npc_block_min_command_speed_kmh",
                self.cfg.get("accident_min_command_speed_kmh", 5.0),
            )
        )
        if float(desired_command_speed_kmh) < min_command_speed_kmh:
            self.reset_cross_lane_npc_block_detection()
            return None

        near_npc = self.find_cross_lane_blocking_npc(ego_state)
        if near_npc is None:
            self.reset_cross_lane_npc_block_detection()
            return None

        now = time.time()
        candidate_key = str(near_npc.get("label", ""))
        if getattr(self, "cross_lane_npc_block_candidate_key", None) != candidate_key:
            self.cross_lane_npc_block_candidate_key = candidate_key
            self.cross_lane_npc_block_enter_time = now

        if getattr(self, "cross_lane_npc_block_enter_time", None) is None:
            self.cross_lane_npc_block_enter_time = now

        stall_for = now - self.cross_lane_npc_block_enter_time
        required_stall_sec = float(
            self.cfg.get("cross_lane_npc_block_duration_sec", 4.0)
        )
        if stall_for < required_stall_sec:
            return None

        event = dict(near_npc)
        event.update(
            {
                "stall_for_sec": float(stall_for),
                "ego_speed_kmh": float(ego_state.get("speed", 0.0)),
                "command_speed_kmh": float(desired_command_speed_kmh),
                "command_brake": float(command.get("brake", 0.0)),
            }
        )
        return event

    def detect_lead_npc_stalled_block(
        self,
        command,
        ego_state,
        desired_command_speed_kmh,
    ):
        if not bool(self.cfg.get("lead_npc_stalled_block_accident_enabled", True)):
            self.reset_lead_npc_stalled_block_detection()
            return None

        min_command_speed_kmh = float(
            self.cfg.get(
                "lead_npc_stalled_block_min_command_speed_kmh",
                self.cfg.get("accident_min_command_speed_kmh", 5.0),
            )
        )
        if float(desired_command_speed_kmh) < min_command_speed_kmh:
            self.reset_lead_npc_stalled_block_detection()
            return None

        min_brake = float(self.cfg.get("lead_npc_stalled_block_min_brake", 0.4))
        if float(command.get("brake", 0.0)) < min_brake:
            self.reset_lead_npc_stalled_block_detection()
            return None

        lead_label = command.get("lead_npc")
        lead = self.find_npc_by_label(lead_label)
        if lead is None:
            self.reset_lead_npc_stalled_block_detection()
            return None

        now = time.time()
        if not self.is_npc_state_recent(lead, now=now):
            self.reset_lead_npc_stalled_block_detection()
            return None

        lead_forward = command.get("lead_forward_m")
        if lead_forward is None:
            self.reset_lead_npc_stalled_block_detection()
            return None
        lead_forward = float(lead_forward)
        lead_min_m = float(self.cfg.get("lead_npc_stalled_block_lead_min_m", 0.0))
        lead_max_m = float(self.cfg.get("lead_npc_stalled_block_lead_max_m", 25.0))
        if lead_forward < lead_min_m or lead_forward > lead_max_m:
            self.reset_lead_npc_stalled_block_detection()
            return None

        max_lead_speed_mps = float(
            self.cfg.get("lead_npc_stalled_block_npc_speed_mps", 0.5)
        )
        lead_speed_mps = float(getattr(lead, "estimated_speed_mps", 0.0))
        if lead_speed_mps > max_lead_speed_mps:
            self.reset_lead_npc_stalled_block_detection()
            return None

        candidate_key = str(getattr(lead, "label", ""))
        if getattr(self, "lead_npc_stall_candidate_key", None) != candidate_key:
            self.lead_npc_stall_candidate_key = candidate_key
            self.lead_npc_stall_enter_time = now

        if getattr(self, "lead_npc_stall_enter_time", None) is None:
            self.lead_npc_stall_enter_time = now

        stall_for = now - self.lead_npc_stall_enter_time
        required_stall_sec = float(
            self.cfg.get("lead_npc_stalled_block_duration_sec", 6.0)
        )
        if stall_for < required_stall_sec:
            return None

        ego_x = float(ego_state["x"])
        ego_y = float(ego_state["y"])
        ego_pose = (
            ego_x,
            ego_y,
            float(ego_state.get("yaw_deg", ego_state.get("yaw", 0.0))),
        )
        forward, left = self.world_to_ego_xy(float(lead.x), float(lead.y), ego_pose)
        distance = dist_xy(ego_x, ego_y, float(lead.x), float(lead.y))

        return {
            "actor_type": "lead_npc_stalled_block",
            "label": getattr(lead, "label", ""),
            "slot": getattr(lead, "slot", ""),
            "opposite": bool(getattr(lead, "opposite", False)),
            "distance_m": float(distance),
            "forward_m": float(forward),
            "left_m": float(left),
            "in_distance": True,
            "in_front_contact_box": True,
            "speed_mps": float(lead_speed_mps),
            "state_source": getattr(lead, "last_state_source", "unknown"),
            "stall_for_sec": float(stall_for),
            "ego_speed_kmh": float(ego_state.get("speed", 0.0)),
            "command_speed_kmh": float(desired_command_speed_kmh),
            "command_brake": float(command.get("brake", 0.0)),
        }

    def detect_static_obstacle_stall(
        self,
        command,
        ego_state,
        desired_command_speed_kmh,
        goal_reached=False,
    ):
        if not bool(self.cfg.get("static_obstacle_accident_enabled", True)):
            self.reset_static_obstacle_detection()
            return None
        if bool(goal_reached):
            self.reset_static_obstacle_detection()
            return None

        min_command_speed_kmh = float(
            self.cfg.get("static_obstacle_min_command_speed_kmh", 2.0)
        )
        if float(desired_command_speed_kmh) < min_command_speed_kmh:
            self.reset_static_obstacle_detection()
            return None

        ignore_brake_above = float(
            self.cfg.get(
                "static_obstacle_ignore_brake_above",
                self.cfg.get("accident_ignore_brake_above", 0.4),
            )
        )
        if float(command.get("brake", 0.0)) > ignore_brake_above:
            self.reset_static_obstacle_detection()
            return None

        now = time.time()
        if getattr(self, "static_obstacle_stall_enter_time", None) is None:
            self.start_static_obstacle_detection(command, ego_state, now)
            return None

        start_s_raw = getattr(self, "static_obstacle_start_s", None)
        start_x_raw = getattr(self, "static_obstacle_start_x", None)
        start_y_raw = getattr(self, "static_obstacle_start_y", None)
        if start_s_raw is None or start_x_raw is None or start_y_raw is None:
            self.start_static_obstacle_detection(command, ego_state, now)
            return None

        start_s = float(start_s_raw)
        start_x = float(start_x_raw)
        start_y = float(start_y_raw)
        current_s = float(command.get("current_s", 0.0))
        ego_x = float(ego_state["x"])
        ego_y = float(ego_state["y"])
        progress_m = abs(current_s - start_s)
        movement_m = dist_xy(ego_x, ego_y, start_x, start_y)
        max_progress_m = float(self.cfg.get("static_obstacle_progress_m", 0.5))
        max_movement_m = float(self.cfg.get("static_obstacle_movement_m", 0.5))
        if progress_m > max_progress_m or movement_m > max_movement_m:
            self.start_static_obstacle_detection(command, ego_state, now)
            return None

        stall_for = now - float(self.static_obstacle_stall_enter_time)
        required_stall_sec = float(
            self.cfg.get("static_obstacle_stall_duration_sec", 4.0)
        )
        if stall_for < required_stall_sec:
            return None

        return {
            "actor_type": "static_obstacle",
            "label": "static_obstacle",
            "slot": "unknown",
            "opposite": False,
            "distance_m": 0.0,
            "forward_m": 0.0,
            "left_m": 0.0,
            "in_distance": False,
            "in_front_contact_box": False,
            "speed_mps": 0.0,
            "state_source": "ego_stall",
            "stall_for_sec": float(stall_for),
            "ego_speed_kmh": float(ego_state.get("speed", 0.0)),
            "command_speed_kmh": float(desired_command_speed_kmh),
            "command_brake": float(command.get("brake", 0.0)),
            "progress_m": float(progress_m),
            "movement_m": float(movement_m),
            "remaining_s": float(command.get("remaining_s", -1.0)),
            "cross_track_error": float(command.get("cross_track_error", -1.0)),
        }

    def detect_accident_stall(self, command, ego_state, elapsed_sec, goal_reached=False):
        if not bool(self.cfg.get("accident_detection_enabled", True)):
            self.reset_accident_detection()
            return None

        min_elapsed_sec = float(self.cfg.get("accident_min_elapsed_sec", 4.0))
        if float(elapsed_sec) < min_elapsed_sec:
            self.reset_accident_detection()
            return None

        ego_speed_kmh = float(ego_state.get("speed", 0.0))
        stall_speed_kmh = float(self.cfg.get("accident_ego_stall_speed_kmh", 1.0))
        min_command_speed_kmh = float(self.cfg.get("accident_min_command_speed_kmh", 5.0))
        ignore_brake_above = float(self.cfg.get("accident_ignore_brake_above", 0.4))
        near_distance_m = float(self.cfg.get("accident_near_npc_distance_m", 4.5))
        near_forward_m = float(self.cfg.get("accident_near_npc_forward_m", 10.0))
        near_side_m = float(self.cfg.get("accident_near_npc_side_m", 4.0))
        near_ped_distance_m = float(
            self.cfg.get("accident_near_pedestrian_distance_m", 3.0)
        )
        near_ped_forward_m = float(
            self.cfg.get("accident_near_pedestrian_forward_m", 6.0)
        )
        near_ped_side_m = float(
            self.cfg.get("accident_near_pedestrian_side_m", 2.5)
        )
        desired_command_speed_kmh = float(
            command.get("unblocked_target_speed", command.get("target_speed", 0.0))
        )

        front_moving_pedestrian_event = self.detect_front_moving_pedestrian_block(
            command,
            ego_state,
            desired_command_speed_kmh,
        )
        if front_moving_pedestrian_event is not None:
            self.reset_direct_accident_detection()
            self.reset_lead_block_detection()
            self.reset_lead_npc_stalled_block_detection()
            self.reset_cross_lane_npc_block_detection()
            self.reset_static_obstacle_detection()
            return front_moving_pedestrian_event

        # Normal follow braking, red-light queueing, and planned stops should not be
        # treated as accidents.
        if ego_speed_kmh > stall_speed_kmh:
            self.reset_direct_accident_detection()
            self.reset_lead_block_detection()
            self.reset_lead_npc_stalled_block_detection()
            self.reset_cross_lane_npc_block_detection()
            self.reset_static_obstacle_detection()
            return None

        lead_block_event = self.detect_lead_npc_pedestrian_block(
            command,
            ego_state,
            desired_command_speed_kmh,
        )
        if lead_block_event is not None:
            self.reset_direct_accident_detection()
            self.reset_lead_npc_stalled_block_detection()
            self.reset_cross_lane_npc_block_detection()
            self.reset_static_obstacle_detection()
            return lead_block_event

        lead_stalled_event = self.detect_lead_npc_stalled_block(
            command,
            ego_state,
            desired_command_speed_kmh,
        )
        if lead_stalled_event is not None:
            self.reset_direct_accident_detection()
            self.reset_lead_block_detection()
            self.reset_cross_lane_npc_block_detection()
            self.reset_static_obstacle_detection()
            return lead_stalled_event

        cross_lane_event = self.detect_cross_lane_npc_block(
            command,
            ego_state,
            desired_command_speed_kmh,
        )
        if cross_lane_event is not None:
            self.reset_direct_accident_detection()
            self.reset_lead_block_detection()
            self.reset_lead_npc_stalled_block_detection()
            self.reset_static_obstacle_detection()
            return cross_lane_event

        near_npc = self.find_near_accident_npc(
            ego_state,
            near_distance_m,
            near_forward_m,
            near_side_m,
        )
        near_pedestrian = self.find_near_accident_pedestrian(
            ego_state,
            near_ped_distance_m,
            near_ped_forward_m,
            near_ped_side_m,
        )
        candidates = [item for item in (near_npc, near_pedestrian) if item is not None]
        if not candidates:
            self.reset_direct_accident_detection()
            return self.detect_static_obstacle_stall(
                command,
                ego_state,
                desired_command_speed_kmh,
                goal_reached=goal_reached,
            )

        self.reset_static_obstacle_detection()
        if desired_command_speed_kmh < min_command_speed_kmh:
            self.reset_direct_accident_detection()
            return None
        near_actor = min(candidates, key=lambda item: item["distance_m"])
        moving_pedestrian_candidate = self.is_moving_pedestrian_accident(near_actor)
        if (
            float(command.get("brake", 0.0)) > ignore_brake_above
            and not moving_pedestrian_candidate
        ):
            self.reset_direct_accident_detection()
            return None

        now = time.time()
        if getattr(self, "accident_stall_enter_time", None) is None:
            self.accident_stall_enter_time = now
        self.accident_stall_candidate = near_actor

        stall_for = now - self.accident_stall_enter_time
        required_stall_sec = float(self.cfg.get("accident_stall_duration_sec", 3.0))
        if stall_for < required_stall_sec:
            return None

        event = dict(near_actor)
        event.setdefault("actor_type", "npc")
        event.update(
            {
                "stall_for_sec": float(stall_for),
                "ego_speed_kmh": float(ego_speed_kmh),
                "command_speed_kmh": float(command.get("target_speed", 0.0)),
                "command_brake": float(command.get("brake", 0.0)),
            }
        )
        return event

    def run_pure_pursuit_timeline(self):
        if getattr(self, "dataset_limit_reached_before_start", False):
            return

        timeout_sec = self.cfg.get("timeout_sec", 0.0)
        use_timeout = timeout_sec is not None and float(timeout_sec) > 0.0
        goal_tolerance_m = float(self.cfg.get("goal_tolerance_m", 4.0))
        check_period_sec = float(self.cfg.get("check_period_sec", 0.05))
        max_cross_track_error_m = float(self.cfg.get("pure_pursuit_max_cross_track_error_m", 8.0))
        arrival_stop_distance_m = float(
            self.cfg.get(
                "pure_pursuit_arrival_stop_distance_m",
                max(goal_tolerance_m, 6.0),
            )
        )
        off_route_timeout_sec = float(self.cfg.get("off_route_timeout_sec", 3.0))
        off_route_grace_sec = float(self.cfg.get("off_route_grace_sec", 2.0))
        max_laps = int(self.cfg.get("max_laps", 0))

        print(
            f"[UrbanBasicDrive] pure pursuit running. "
            f"speed={self.cfg.get('pure_pursuit_target_speed', 8.0)}, "
            f"lookahead={self.cfg.get('pure_pursuit_lookahead_m', 8.0)}m, "
            f"arrival_stop={arrival_stop_distance_m}m, "
            f"timeout={'disabled' if not use_timeout else str(timeout_sec) + 's'}"
        )

        lap = 0
        lap_start_time = time.time()
        last_print_time = 0.0
        off_route_enter_time = None

        while True:
            elapsed = time.time() - lap_start_time
            ego_state = self.grpc.get_ego_motion_state()
            self.update_fast_traffic_light(ego_state)
            ego_x = ego_state["x"]
            ego_y = ego_state["y"]
            dist_to_goal = dist_xy(ego_x, ego_y, self.goal_x, self.goal_y)
            command = self.compute_pure_pursuit_command(ego_state)
            current_link = ego_state["current_link"]
            self.update_npc_vehicle_manager(command["current_s"], ego_state=ego_state)
            self.update_pedestrian_manager(command["current_s"], ego_state=ego_state)
            command = self.apply_npc_follow_brake(command, ego_state)

            near_route_end = command["remaining_s"] <= arrival_stop_distance_m
            route_end_reached = near_route_end and command["cross_track_error"] <= max_cross_track_error_m
            goal_reached = route_end_reached or dist_to_goal <= goal_tolerance_m
            accident_event = self.detect_accident_stall(
                command,
                ego_state,
                elapsed,
                goal_reached=goal_reached,
            )

            if elapsed >= off_route_grace_sec and command["cross_track_error"] > max_cross_track_error_m:
                if off_route_enter_time is None:
                    off_route_enter_time = time.time()
            else:
                off_route_enter_time = None

            if elapsed - last_print_time >= 1.0:
                print(
                    f"[UrbanBasicDrive] lap={lap + 1} "
                    f"t={elapsed:.1f}s "
                    f"ego=({ego_x:.2f}, {ego_y:.2f}) "
                    f"yaw={ego_state['yaw_deg']:.1f} "
                    f"speed={ego_state['speed']:.1f} "
                    f"link={current_link} "
                    f"s={command['current_s']:.1f}/{self.route_length_m:.1f} "
                    f"cte={command['cross_track_error']:.2f} "
                    f"remain={command['remaining_s']:.1f} "
                    f"steer={command['steer']:.3f} "
                    f"la={command.get('lookahead_m', -1.0):.1f} "
                    f"turn={command.get('turn_delta_deg', 0.0):.0f} "
                    f"passed={command.get('passed_turn_deg', 0.0):.0f} "
                    f"entry={command.get('entry_turn_deg', 0.0):.0f} "
                    f"phase={command.get('corner_phase', '-')} "
                    f"sgain={command.get('steer_gain', 1.0):.2f} "
                    f"cmd_speed={command['target_speed']:.1f} "
                    f"brake={command.get('brake', 0.0):.2f} "
                    f"lead={command.get('lead_npc') or '-'} "
                    f"gap={command.get('lead_gap_m') if command.get('lead_gap_m') is not None else -1:.1f} "
                    f"fwd={command.get('lead_forward_m') if command.get('lead_forward_m') is not None else -1:.1f} "
                    f"bumper={command.get('lead_bumper_gap_m') if command.get('lead_bumper_gap_m') is not None else -1:.1f} "
                    f"ttc={command.get('lead_ttc_sec') if command.get('lead_ttc_sec') is not None else -1:.1f} "
                    f"dist_to_goal={dist_to_goal:.2f}m"
                )
                last_print_time = elapsed

            if accident_event is not None:
                scene_name = getattr(self, "scene_collection_name", None) or "-"
                scene_dir = getattr(self, "scene_collection_dir", None) or "-"
                actor_type = accident_event.get("actor_type", "npc")
                if actor_type == "static_obstacle":
                    accident_title = "STATIC OBSTACLE STALL"
                elif actor_type == "cross_lane_npc_block":
                    accident_title = "CROSS LANE NPC BLOCK"
                elif actor_type == "lead_npc_pedestrian_block":
                    accident_title = "LEAD NPC PEDESTRIAN BLOCK"
                elif actor_type == "lead_npc_stalled_block":
                    accident_title = "LEAD NPC STALLED BLOCK"
                else:
                    accident_title = "COLLISION STALL"
                print(
                    f"[Accident] {accident_title} - save scene and restart "
                    f"scene={scene_name} dir={scene_dir} "
                    f"actor={actor_type} "
                    f"near={accident_event['label']} "
                    f"slot={accident_event['slot']} opposite={accident_event['opposite']} "
                    f"dist={accident_event['distance_m']:.2f}m "
                    f"forward={accident_event['forward_m']:.2f}m "
                    f"left={accident_event['left_m']:.2f}m "
                    f"box={int(accident_event.get('in_front_contact_box', False))} "
                    f"radius={int(accident_event.get('in_distance', False))} "
                    f"stall={accident_event['stall_for_sec']:.1f}s "
                    f"ego_speed={accident_event['ego_speed_kmh']:.1f} "
                    f"cmd_speed={accident_event['command_speed_kmh']:.1f} "
                    f"brake={accident_event['command_brake']:.2f} "
                    f"blocked_by={accident_event.get('blocked_by_label', '-')} "
                    f"blocked_by_dist={accident_event.get('blocked_by_distance_m', -1.0):.2f}m "
                    f"progress={accident_event.get('progress_m', -1.0):.2f}m "
                    f"move={accident_event.get('movement_m', -1.0):.2f}m"
                )
                self.save_accident_scene(
                    accident_title=accident_title,
                    accident_event=accident_event,
                    lap=lap + 1,
                    elapsed_sec=elapsed,
                    dist_to_goal_m=dist_to_goal,
                    ego_state=ego_state,
                )
                self.stop_pure_pursuit_control()
                if self.dataset_stop_target_reached(settle=True):
                    break
                self.restart_to_start_and_drive()
                lap_start_time = time.time()
                last_print_time = 0.0
                off_route_enter_time = None
                self.reset_accident_detection()
                continue

            if (
                off_route_enter_time is not None
                and time.time() - off_route_enter_time >= off_route_timeout_sec
            ):
                print(
                    f"[UrbanBasicDrive] PURE PURSUIT OFF ROUTE - restart "
                    f"cte={command['cross_track_error']:.2f}m, link={current_link}, "
                    f"elapsed={elapsed:.1f}s"
                )
                self.stop_pure_pursuit_control()
                self.restart_to_start_and_drive()
                lap_start_time = time.time()
                last_print_time = 0.0
                off_route_enter_time = None
                continue

            if goal_reached:
                lap += 1
                print(
                    f"[UrbanBasicDrive] PURE PURSUIT GOAL REACHED "
                    f"lap={lap}, link={current_link}, dist={dist_to_goal:.2f}m, "
                    f"s={command['current_s']:.1f}/{self.route_length_m:.1f}, "
                    f"elapsed={elapsed:.1f}s"
                )
                self.stop_pure_pursuit_control()
                self.save_success_scene(
                    result="PURE PURSUIT GOAL REACHED",
                    lap=lap,
                    elapsed_sec=elapsed,
                    dist_to_goal_m=dist_to_goal,
                    ego_state=ego_state,
                    extra={
                        "route_s_m": float(command["current_s"]),
                        "remaining_s_m": float(command["remaining_s"]),
                        "cross_track_error_m": float(command["cross_track_error"]),
                    },
                )
                if self.dataset_stop_target_reached(settle=True):
                    break

                if max_laps > 0 and lap >= max_laps:
                    print("[UrbanBasicDrive] max_laps reached. finish scenario.")
                    break

                self.restart_to_start_and_drive()
                lap_start_time = time.time()
                last_print_time = 0.0
                off_route_enter_time = None
                continue

            if use_timeout and elapsed >= float(timeout_sec):
                print(
                    f"[UrbanBasicDrive] PURE PURSUIT TIMEOUT "
                    f"lap={lap + 1}, dist={dist_to_goal:.2f}m, elapsed={elapsed:.1f}s"
                )
                self.stop_pure_pursuit_control()
                break

            if self.route_setup_mode == "ros_pure_pursuit":
                self.publish_ros_ctrl_cmd(
                    steer=command["steer"],
                    target_speed=command["target_speed"],
                    current_speed=ego_state["speed"],
                    brake_override=command.get("brake") if command.get("brake", 0.0) > 0.0 else None,
                )
            else:
                self.grpc.control_ego(
                    steer=command["steer"],
                    target_speed=command["target_speed"],
                    brake=command.get("brake", 0.0),
                    throttle=0.0,
                )
            time.sleep(check_period_sec)
