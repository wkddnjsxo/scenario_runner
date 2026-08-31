# -*- coding: utf-8 -*-

import json
import math
import os
import random
import time

from utils.geometry_utils import (
    dist_xy,
    interpolate_on_polyline,
    polyline_length,
    project_distance_on_polyline,
)


DEFAULT_PEDESTRIAN_CONFIG = {
    "enabled": False,
    "update_interval_sec": 0.5,
    "target_count": [2, 4],
    "max_spawn_attempts": 20,
    "spawn_ahead_m": [35.0, 120.0],
    "side_offset_m": [6.0, 9.0],
    "min_distance_from_ego_m": 18.0,
    "min_distance_between_pedestrians_m": 10.0,
    "runtime_max_spawn_per_update": 1,
    "reject_spawn_if_visible": True,
    "visible_spawn_front_m": 90.0,
    "visible_spawn_rear_m": 5.0,
    "visible_spawn_side_m": 18.0,
    "despawn_behind_m": 35.0,
    "despawn_ahead_m": 160.0,
    "protect_moving_pedestrians_from_distance_despawn": True,
    "walking_despawn_use_ego_frame": True,
    "status_log_interval_sec": 5.0,
    "control_interval_sec": 0.5,
    "walking_speed_mps": [0.8, 1.3],
    "walking_stop_check_ahead_m": 40.0,
    "walking_stop_if_position_stuck": True,
    "walking_position_stuck_duration_sec": 2.0,
    "walking_position_stuck_move_m": 0.15,
    "walking_position_stuck_speed_mps": 0.1,
    "standing_yaw_mode": "random",
    "standing_yaw_offset_deg": [-180.0, 180.0],
    "standing_spawn_source": "lane_boundary",
    "standing_fallback_to_route": False,
    "standing_boundary_offset_m": [0.8, 2.0],
    "standing_candidate_interval_m": 5.0,
    "standing_use_outer_lane_only": True,
    "standing_include_opposite_outer_lane": True,
    "opposite_link_search_max_distance_m": 25.0,
    "opposite_link_min_overlap_ratio": 0.25,
    "opposite_link_min_yaw_diff_deg": 135.0,
    "standing_include_lane_colors": ["white"],
    "standing_include_lane_shapes": ["Solid"],
    "standing_exclude_lane_types": [530],
    "standing_exclude_signal_point_radius_m": 5.0,
    "walking_spawn_source": "crosswalk",
    "walking_fallback_to_route": False,
    "crosswalk_route_max_dist_m": 18.0,
    "crosswalk_min_width_m": 2.0,
    "behavior_weights": {
        "roadside_standing": 0.55,
        "crosswalk_walking": 0.45,
    },
    "model": {
        "whitelist": [],
        "exclude_names": ["Bicycle_Female1", "Ped_Pet1"],
        "exclude_prefixes": [
            "NCAP",
            "Animal",
            "Dog",
            "Deer",
            "Cat",
            "Horse",
            "Cow",
        ],
        "exclude_contains": [],
    },
}


def deep_merge_dict(base, override):
    out = dict(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge_dict(out[key], value)
        else:
            out[key] = value
    return out


class RoutePositionHelper:
    def __init__(self, route_points):
        self.route_points = list(route_points or [])
        self.route_length_m = polyline_length(self.route_points) if len(self.route_points) >= 2 else 0.0

    def pose_at(self, route_s, lateral_offset_m=0.0):
        route_s = max(0.0, min(float(route_s), self.route_length_m))
        x, y, z, yaw_deg = interpolate_on_polyline(self.route_points, route_s)
        if abs(float(lateral_offset_m)) > 1e-6:
            yaw_rad = math.radians(yaw_deg)
            x += -math.sin(yaw_rad) * float(lateral_offset_m)
            y += math.cos(yaw_rad) * float(lateral_offset_m)
        return x, y, z, yaw_deg

    def project_s(self, x, y):
        return project_distance_on_polyline(self.route_points, x, y)


class PedestrianModelSelector:
    def __init__(self, cfg, rng):
        self.cfg = cfg or {}
        self.rng = rng or random.Random()
        self.models = []

    def load(self, sim_bridge):
        whitelist = [str(name) for name in self.cfg.get("whitelist", []) if str(name)]
        if whitelist:
            self.models = whitelist
            return

        models = sim_bridge.get_available_pedestrian_models()
        exclude_names = set(
            str(name).lower() for name in self.cfg.get("exclude_names", []) or []
        )
        if exclude_names:
            models = [name for name in models if str(name).lower() not in exclude_names]
        exclude_prefixes = tuple(
            str(prefix).lower() for prefix in self.cfg.get("exclude_prefixes", []) or []
        )
        if exclude_prefixes:
            models = [
                name
                for name in models
                if not str(name).lower().startswith(exclude_prefixes)
            ]
        exclude_contains = tuple(
            str(text).lower() for text in self.cfg.get("exclude_contains", []) or []
        )
        if exclude_contains:
            models = [
                name
                for name in models
                if not any(text in str(name).lower() for text in exclude_contains)
            ]
        self.models = [str(name) for name in models if str(name)]

    def choose(self):
        if not self.models:
            return None
        return self.rng.choice(self.models)


class ManagedPedestrian:
    def __init__(
        self,
        label,
        actor,
        route_s,
        x,
        y,
        behavior,
        side_offset_m,
        walking_speed_mps=0.0,
        control_dir_x=None,
        control_dir_y=None,
        target_x=None,
        target_y=None,
    ):
        self.label = label
        self.actor = actor
        self.route_s = float(route_s)
        self.x = float(x)
        self.y = float(y)
        self.behavior = str(behavior)
        self.side_offset_m = float(side_offset_m)
        self.walking_speed_mps = float(walking_speed_mps)
        self.control_dir_x = control_dir_x
        self.control_dir_y = control_dir_y
        self.target_x = None if target_x is None else float(target_x)
        self.target_y = None if target_y is None else float(target_y)
        self.initial_walking_speed_mps = float(walking_speed_mps)
        self.stopped = float(walking_speed_mps) <= 0.0
        self.stop_reason = "non_walking" if self.stopped else None
        self.spawn_time = time.time()
        self.last_state_time = 0.0
        self.state_miss_count = 0
        self.last_control_time = 0.0
        self.last_progress_check_time = self.spawn_time
        self.last_position_check_time = self.spawn_time
        self.last_position_check_x = self.x
        self.last_position_check_y = self.y
        if self.target_x is not None and self.target_y is not None:
            self.last_target_distance = dist_xy(self.x, self.y, self.target_x, self.target_y)
        else:
            self.last_target_distance = None


class PedestrianManager:
    def __init__(self, sim_bridge, cfg, rng=None, map_loader=None):
        self.sim_bridge = sim_bridge
        self.cfg = deep_merge_dict(DEFAULT_PEDESTRIAN_CONFIG, cfg or {})
        self.rng = rng or random.Random()
        self.map_loader = map_loader
        self.enabled = bool(self.cfg.get("enabled", False))
        self.route_helper = None
        self.route_links = []
        self.route_link_spans = []
        self.standing_candidates = []
        self.crosswalk_candidates = []
        self.opposite_link_cache = {}
        self.live_pedestrians = []
        self.target_count = 0
        self.next_id = 1
        self.last_update_time = 0.0
        self.last_status_log_time = 0.0
        self.model_selector = PedestrianModelSelector(self.cfg.get("model", {}), self.rng)

    def set_route(self, route_points, route_links=None, route_link_spans=None):
        self.route_helper = RoutePositionHelper(route_points)
        self.route_links = list(route_links or [])
        self.route_link_spans = list(route_link_spans or [])
        if not self.route_link_spans and self.map_loader is not None and self.route_links:
            self.route_link_spans = self.build_route_link_spans(self.route_links)
        self.target_count = self.select_target_count()
        self.last_update_time = 0.0
        self.last_status_log_time = 0.0
        self.opposite_link_cache = {}
        self.model_selector.load(self.sim_bridge)
        self.build_spawn_candidates()
        if not self.model_selector.models:
            self.target_count = 0
        print(
            f"[Ped] route configured length={self.route_helper.route_length_m:.1f}m "
            f"target_count={self.target_count} models={len(self.model_selector.models)} "
            f"standing_candidates={len(self.standing_candidates)} "
            f"crosswalk_candidates={len(self.crosswalk_candidates)}"
        )

    def build_route_link_spans(self, route_links):
        spans = []
        cursor = 0.0
        for link_id in route_links:
            try:
                points = self.map_loader.get_link_points(link_id)
            except Exception:
                continue
            length_m = polyline_length(points)
            spans.append({"link_id": link_id, "start_s": cursor, "end_s": cursor + length_m})
            cursor += length_m
        return spans

    def select_target_count(self):
        value = self.cfg.get("target_count", [2, 4])
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            lo = max(0, int(value[0]))
            hi = max(lo, int(value[1]))
            return self.rng.randint(lo, hi)
        return max(0, int(value))

    def load_mgeo_list(self, filename):
        if self.map_loader is None:
            return []
        path = os.path.join(self.map_loader.mgeo_root, filename)
        if not os.path.exists(path):
            return []
        try:
            with open(path, "r") as f:
                data = json.load(f)
        except Exception as exc:
            print(f"[Ped] failed to load {filename}: {exc}")
            return []
        if isinstance(data, list):
            return data
        if isinstance(data, dict):
            return list(data.values())
        return []

    def build_spawn_candidates(self):
        self.standing_candidates = []
        self.crosswalk_candidates = []
        if self.route_helper is None or self.map_loader is None:
            return
        if str(self.cfg.get("standing_spawn_source", "lane_boundary")).lower() == "lane_boundary":
            self.standing_candidates = self.build_standing_boundary_candidates()
        if str(self.cfg.get("walking_spawn_source", "crosswalk")).lower() == "crosswalk":
            self.crosswalk_candidates = self.build_crosswalk_candidates()

    def normalize_lane_types(self, value):
        if isinstance(value, (list, tuple)):
            return {str(item) for item in value}
        if value is None:
            return set()
        return {str(value)}

    def normalize_string_list(self, value):
        if isinstance(value, (list, tuple)):
            return {str(item).lower() for item in value}
        if value is None:
            return set()
        return {str(value).lower()}

    def is_excluded_standing_boundary(self, boundary):
        include_colors = self.normalize_string_list(
            self.cfg.get("standing_include_lane_colors", ["white"])
        )
        if include_colors:
            colors = self.normalize_string_list(boundary.get("lane_color"))
            if not (colors & include_colors):
                return True

        include_shapes = self.normalize_string_list(
            self.cfg.get("standing_include_lane_shapes", ["Solid"])
        )
        if include_shapes:
            shapes = self.normalize_string_list(boundary.get("lane_shape"))
            if not (shapes & include_shapes):
                return True

        excluded = {
            str(item)
            for item in self.cfg.get("standing_exclude_lane_types", [530]) or []
        }
        if not excluded:
            return False
        return bool(self.normalize_lane_types(boundary.get("lane_type")) & excluded)

    def load_signal_exclusion_points(self):
        radius = float(self.cfg.get("standing_exclude_signal_point_radius_m", 5.0))
        if radius <= 0.0:
            return []
        items = self.load_mgeo_list("synced_traffic_light_set.json")
        points = []
        for item in items:
            for point in item.get("point", []) if isinstance(item, dict) else []:
                if len(point) >= 2:
                    points.append((float(point[0]), float(point[1])))
        return points

    def near_signal_exclusion_point(self, x, y, signal_points, radius):
        if radius <= 0.0 or not signal_points:
            return False
        radius_sq = radius * radius
        for px, py in signal_points:
            dx = float(x) - px
            dy = float(y) - py
            if dx * dx + dy * dy <= radius_sq:
                return True
        return False

    def append_standing_boundary_candidates_for_link(
        self,
        candidates,
        link_id,
        boundary_by_id,
        interval,
        outer_only,
        signal_points,
        signal_radius,
        source="lane_boundary",
    ):
        if not link_id or link_id not in self.map_loader.link_set:
            return

        link = self.map_loader.link_set[link_id]
        sides = []
        if (not outer_only) or (not bool(link.get("can_move_left_lane", False))):
            sides.append(("left", 1.0, link.get("lane_mark_left", []) or []))
        if (not outer_only) or (not bool(link.get("can_move_right_lane", False))):
            sides.append(("right", -1.0, link.get("lane_mark_right", []) or []))

        link_points = link.get("points", [])
        link_yaw_deg = self.link_yaw(link_points) if len(link_points) >= 2 else None
        for side_name, side_sign, mark_ids in sides:
            for mark_id in mark_ids:
                boundary = boundary_by_id.get(str(mark_id))
                points = boundary.get("points", []) if boundary else []
                if len(points) < 2:
                    continue
                if self.is_excluded_standing_boundary(boundary):
                    continue
                length_m = polyline_length(points)
                sample_s = 0.0
                while sample_s <= length_m:
                    try:
                        bx, by, bz, _ = interpolate_on_polyline(points, sample_s)
                        route_s = self.route_helper.project_s(bx, by)
                        _, _, _, route_yaw = self.route_helper.pose_at(route_s, 0.0)
                    except Exception:
                        sample_s += interval
                        continue
                    if self.near_signal_exclusion_point(
                        bx, by, signal_points, signal_radius
                    ):
                        sample_s += interval
                        continue
                    yaw_for_normal = link_yaw_deg if link_yaw_deg is not None else route_yaw
                    yaw_rad = math.radians(yaw_for_normal)
                    normal_x = -math.sin(yaw_rad) * side_sign
                    normal_y = math.cos(yaw_rad) * side_sign
                    candidates.append(
                        {
                            "source": source,
                            "link_id": link_id,
                            "boundary_id": str(mark_id),
                            "side_name": side_name,
                            "side_sign": side_sign,
                            "route_s": route_s,
                            "x": bx,
                            "y": by,
                            "z": bz,
                            "route_yaw_deg": route_yaw,
                            "normal_x": normal_x,
                            "normal_y": normal_y,
                        }
                    )
                    sample_s += interval

    def build_standing_boundary_candidates(self):
        boundary_items = self.load_mgeo_list("lane_boundary_set.json")
        boundary_by_id = {
            str(item.get("idx")): item
            for item in boundary_items
            if isinstance(item, dict) and item.get("idx") is not None
        }
        if not boundary_by_id:
            return []

        candidates = []
        interval = max(1.0, float(self.cfg.get("standing_candidate_interval_m", 5.0)))
        outer_only = bool(self.cfg.get("standing_use_outer_lane_only", True))
        include_opposite = bool(self.cfg.get("standing_include_opposite_outer_lane", True))
        signal_points = self.load_signal_exclusion_points()
        signal_radius = float(self.cfg.get("standing_exclude_signal_point_radius_m", 5.0))
        for span in self.route_link_spans:
            link_id = span.get("link_id")
            self.append_standing_boundary_candidates_for_link(
                candidates,
                link_id,
                boundary_by_id,
                interval,
                outer_only,
                signal_points,
                signal_radius,
                source="lane_boundary",
            )
            if include_opposite:
                for opposite_link in self.find_opposite_links(link_id):
                    self.append_standing_boundary_candidates_for_link(
                        candidates,
                        opposite_link,
                        boundary_by_id,
                        interval,
                        outer_only,
                        signal_points,
                        signal_radius,
                        source="opposite_lane_boundary",
                    )
        return candidates

    def find_opposite_links(self, route_link):
        if self.map_loader is None:
            return []
        if not route_link or route_link not in self.map_loader.link_set:
            return []
        if route_link in self.opposite_link_cache:
            return list(self.opposite_link_cache[route_link])

        target_points = self.map_loader.get_link_points(route_link)
        if len(target_points) < 2:
            self.opposite_link_cache[route_link] = []
            return []

        target_yaw = self.link_yaw(target_points)
        target_origin = self.centroid(target_points)
        ux = math.cos(math.radians(target_yaw))
        uy = math.sin(math.radians(target_yaw))
        target_range = self.projection_range(target_points, target_origin, ux, uy)
        target_len = max(1e-6, target_range[1] - target_range[0])

        max_distance = float(self.cfg.get("opposite_link_search_max_distance_m", 25.0))
        min_overlap = float(self.cfg.get("opposite_link_min_overlap_ratio", 0.25))
        min_yaw_diff = float(self.cfg.get("opposite_link_min_yaw_diff_deg", 135.0))

        candidates = []
        for link_id, link in self.map_loader.link_set.items():
            if link_id == route_link:
                continue
            points = link.get("points", [])
            if len(points) < 2:
                continue

            yaw = self.link_yaw(points)
            yaw_diff = abs(self.normalize_angle_deg(yaw - target_yaw))
            if yaw_diff < min_yaw_diff:
                continue

            other_range = self.projection_range(points, target_origin, ux, uy)
            overlap = max(
                0.0,
                min(target_range[1], other_range[1])
                - max(target_range[0], other_range[0]),
            )
            other_len = max(1e-6, other_range[1] - other_range[0])
            overlap_ratio = overlap / min(target_len, other_len)
            if overlap_ratio < min_overlap:
                continue

            distance = self.polyline_min_distance(target_points, points)
            if distance > max_distance:
                continue

            candidates.append((distance, -overlap_ratio, link_id))

        candidates.sort()
        result = [item[2] for item in candidates[:3]]
        self.opposite_link_cache[route_link] = result
        return list(result)

    def link_yaw(self, points):
        p0 = points[0]
        p1 = points[-1]
        return math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))

    def centroid(self, points):
        return (
            sum(point[0] for point in points) / len(points),
            sum(point[1] for point in points) / len(points),
        )

    def projection_range(self, points, origin, ux, uy):
        values = [
            (point[0] - origin[0]) * ux + (point[1] - origin[1]) * uy
            for point in points
        ]
        return min(values), max(values)

    def polyline_min_distance(self, a_points, b_points):
        best = float("inf")
        a_step = max(1, len(a_points) // 20)
        b_step = max(1, len(b_points) // 20)
        for point in a_points[::a_step]:
            for p0, p1 in zip(b_points[:-1], b_points[1:]):
                best = min(best, self.point_segment_distance(point[0], point[1], p0, p1))
        for point in b_points[::b_step]:
            for p0, p1 in zip(a_points[:-1], a_points[1:]):
                best = min(best, self.point_segment_distance(point[0], point[1], p0, p1))
        return best

    def point_segment_distance(self, px, py, p0, p1):
        x0, y0 = p0[0], p0[1]
        x1, y1 = p1[0], p1[1]
        dx = x1 - x0
        dy = y1 - y0
        denom = dx * dx + dy * dy
        if denom <= 1e-12:
            return math.hypot(px - x0, py - y0)
        t = ((px - x0) * dx + (py - y0) * dy) / denom
        t = max(0.0, min(1.0, t))
        cx = x0 + t * dx
        cy = y0 + t * dy
        return math.hypot(px - cx, py - cy)

    def normalize_angle_deg(self, angle):
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    def unique_polygon_points(self, points):
        out = []
        seen = set()
        for point in points or []:
            if len(point) < 2:
                continue
            key = (round(float(point[0]), 3), round(float(point[1]), 3))
            if key in seen:
                continue
            seen.add(key)
            out.append(point)
        return out

    def build_crosswalk_candidates(self):
        crosswalk_items = self.load_mgeo_list("singlecrosswalk_set.json")
        if not crosswalk_items:
            return []

        candidates = []
        max_dist = float(self.cfg.get("crosswalk_route_max_dist_m", 18.0))
        min_width = float(self.cfg.get("crosswalk_min_width_m", 2.0))
        for item in crosswalk_items:
            points = self.unique_polygon_points(item.get("points", []) if isinstance(item, dict) else [])
            if len(points) < 3:
                continue
            cx = sum(float(p[0]) for p in points) / len(points)
            cy = sum(float(p[1]) for p in points) / len(points)
            cz = sum(float(p[2]) for p in points if len(p) >= 3) / max(1, sum(1 for p in points if len(p) >= 3))
            try:
                route_s = self.route_helper.project_s(cx, cy)
                rx, ry, _, route_yaw = self.route_helper.pose_at(route_s, 0.0)
            except Exception:
                continue
            if dist_xy(cx, cy, rx, ry) > max_dist:
                continue

            yaw_rad = math.radians(route_yaw)
            scored = []
            for point in points:
                dx = float(point[0]) - rx
                dy = float(point[1]) - ry
                lateral = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
                scored.append((lateral, point))
            scored.sort(key=lambda item_pair: item_pair[0])
            start = scored[0][1]
            end = scored[-1][1]
            width_m = dist_xy(start[0], start[1], end[0], end[1])
            if width_m < min_width:
                continue

            candidates.append(
                {
                    "source": "crosswalk",
                    "crosswalk_id": str(item.get("idx", "")),
                    "route_s": route_s,
                    "center_x": cx,
                    "center_y": cy,
                    "z": cz,
                    "start": start,
                    "end": end,
                    "width_m": width_m,
                }
            )
        return candidates

    def reset_live_refs(self):
        self.live_pedestrians = []

    def destroy_all(self, reason="cleanup"):
        for ped in list(self.live_pedestrians):
            self.destroy_pedestrian(ped, reason=reason)
        self.live_pedestrians = []

    def destroy_pedestrian(self, ped, reason="cleanup"):
        try:
            ped.actor.destroy()
        except Exception as exc:
            print(f"[Ped] destroy failed label={ped.label}: {exc}")
        print(f"[Ped] despawn label={ped.label} reason={reason}")

    def update(self, ego_s, ego_state=None, fill_all=False):
        if not self.enabled or self.route_helper is None:
            return
        interval = float(self.cfg.get("update_interval_sec", 0.5))
        now = time.time()
        if interval > 0.0 and now - self.last_update_time < interval:
            return
        self.last_update_time = now

        self.refresh_pedestrian_positions()
        self.control_walking_pedestrians(float(ego_s), ego_state=ego_state)
        self.despawn_old_pedestrians(float(ego_s), ego_state=ego_state)

        spawn_budget = self.target_count
        if not fill_all:
            spawn_budget = max(0, int(self.cfg.get("runtime_max_spawn_per_update", 1)))

        spawned = 0
        while len(self.live_pedestrians) < self.target_count and spawned < spawn_budget:
            if not self.try_spawn_one(float(ego_s), ego_state=ego_state):
                break
            spawned += 1

        self.log_status(float(ego_s))

    def refresh_pedestrian_positions(self):
        keep = []
        for ped in self.live_pedestrians:
            try:
                state = ped.actor.get_actor_state()
            except Exception:
                state = None
            if state is None:
                ped.state_miss_count += 1
                if ped.state_miss_count >= int(self.cfg.get("state_miss_remove_count", 10)):
                    self.destroy_pedestrian(ped, reason="state_missing")
                    continue
                keep.append(ped)
                continue

            ped.x = float(state.transform.location.x)
            ped.y = float(state.transform.location.y)
            ped.route_s = self.route_helper.project_s(ped.x, ped.y)
            ped.last_state_time = time.time()
            ped.state_miss_count = 0
            keep.append(ped)
        self.live_pedestrians = keep

    def control_walking_pedestrians(self, ego_s=None, ego_state=None):
        interval = float(self.cfg.get("control_interval_sec", 0.5))
        now = time.time()
        for ped in self.live_pedestrians:
            if not self.is_walking_behavior(ped.behavior):
                continue
            if interval > 0.0 and now - ped.last_control_time < interval:
                continue
            if ped.control_dir_x is not None and ped.control_dir_y is not None:
                direction_x = float(ped.control_dir_x)
                direction_y = float(ped.control_dir_y)
            else:
                _, _, _, yaw_deg = self.route_helper.pose_at(ped.route_s, ped.side_offset_m)
                yaw_rad = math.radians(yaw_deg)
                direction_x = math.cos(yaw_rad)
                direction_y = math.sin(yaw_rad)

            if self.should_stop_walking_pedestrian(
                ped,
                direction_x,
                direction_y,
                now,
                ego_s=ego_s,
                ego_state=ego_state,
            ):
                self.stop_walking_pedestrian(ped, reason=ped.stop_reason or "target")
                ped.last_control_time = now
                continue

            self.sim_bridge.control_pedestrian(
                ped.actor,
                direction_x=direction_x,
                direction_y=direction_y,
                speed_mps=ped.walking_speed_mps,
                quiet=True,
            )
            ped.last_control_time = now

    def walking_stop_checks_active(self, ped, ego_s, ego_state, now):
        if ego_s is None:
            return True
        ahead_m = float(
            self.cfg.get(
                "walking_stop_check_ahead_m",
                float(self.cfg.get("active_dist_m", 30.0)) + 10.0,
            )
        )
        rel_s = float(ped.route_s) - float(ego_s)
        frame = self.pedestrian_ego_frame(ped, ego_state)
        forward = frame[0] if frame is not None else None
        if rel_s <= ahead_m or (forward is not None and forward <= ahead_m):
            return True

        ped.last_target_distance = (
            None
            if ped.target_x is None or ped.target_y is None
            else dist_xy(ped.x, ped.y, ped.target_x, ped.target_y)
        )
        ped.last_progress_check_time = now
        ped.last_position_check_x = float(ped.x)
        ped.last_position_check_y = float(ped.y)
        ped.last_position_check_time = now
        return False

    def should_stop_walking_pedestrian(
        self,
        ped,
        direction_x,
        direction_y,
        now,
        ego_s=None,
        ego_state=None,
    ):
        if getattr(ped, "stopped", False):
            ped.stop_reason = ped.stop_reason or "already_stopped"
            return True

        if not self.walking_stop_checks_active(ped, ego_s, ego_state, now):
            return False

        if self.should_stop_position_stuck_pedestrian(ped, now):
            return True

        if ped.target_x is None or ped.target_y is None:
            return False

        if not bool(self.cfg.get("walking_stop_at_target", True)):
            return False

        target_dist = dist_xy(ped.x, ped.y, ped.target_x, ped.target_y)
        reach_m = float(self.cfg.get("walking_target_reach_m", 0.8))
        if target_dist <= reach_m:
            ped.stop_reason = "target_reached"
            return True

        to_target_x = ped.target_x - ped.x
        to_target_y = ped.target_y - ped.y
        if to_target_x * float(direction_x) + to_target_y * float(direction_y) <= 0.0:
            ped.stop_reason = "target_passed"
            return True

        if not bool(self.cfg.get("walking_stop_if_blocked", True)):
            return False

        blocked_duration = float(self.cfg.get("walking_blocked_duration_sec", 2.0))
        progress_m = float(self.cfg.get("walking_blocked_progress_m", 0.25))
        last_dist = getattr(ped, "last_target_distance", None)
        last_time = float(getattr(ped, "last_progress_check_time", now))
        if last_dist is None:
            ped.last_target_distance = target_dist
            ped.last_progress_check_time = now
            return False

        if last_dist - target_dist >= progress_m:
            ped.last_target_distance = target_dist
            ped.last_progress_check_time = now
            return False

        if now - last_time >= blocked_duration:
            ped.stop_reason = "blocked_no_progress"
            return True

        return False

    def should_stop_position_stuck_pedestrian(self, ped, now):
        if not bool(self.cfg.get("walking_stop_if_position_stuck", True)):
            return False

        speed_mps = float(getattr(ped, "walking_speed_mps", 0.0))
        min_speed_mps = float(self.cfg.get("walking_position_stuck_speed_mps", 0.1))
        if speed_mps <= min_speed_mps:
            return False

        min_move_m = float(self.cfg.get("walking_position_stuck_move_m", 0.15))
        stuck_duration_sec = float(
            self.cfg.get("walking_position_stuck_duration_sec", 2.0)
        )
        last_x = float(getattr(ped, "last_position_check_x", ped.x))
        last_y = float(getattr(ped, "last_position_check_y", ped.y))
        last_time = float(getattr(ped, "last_position_check_time", now))
        moved_m = dist_xy(float(ped.x), float(ped.y), last_x, last_y)

        if moved_m >= min_move_m:
            ped.last_position_check_x = float(ped.x)
            ped.last_position_check_y = float(ped.y)
            ped.last_position_check_time = now
            return False

        if now - last_time >= stuck_duration_sec:
            ped.stop_reason = "position_stuck"
            return True

        return False

    def stop_walking_pedestrian(self, ped, reason="target"):
        ped.stopped = True
        ped.stop_reason = reason
        ped.walking_speed_mps = 0.0
        if ped.control_dir_x is not None and ped.control_dir_y is not None:
            direction_x = float(ped.control_dir_x)
            direction_y = float(ped.control_dir_y)
        else:
            direction_x = 0.0
            direction_y = 0.0
        self.sim_bridge.control_pedestrian(
            ped.actor,
            direction_x=direction_x,
            direction_y=direction_y,
            speed_mps=0.0,
            quiet=True,
        )

    def pedestrian_ego_frame(self, ped, ego_state):
        if ego_state is None:
            return None
        try:
            ego_x = float(ego_state["x"])
            ego_y = float(ego_state["y"])
            yaw_deg = float(ego_state.get("yaw_deg", ego_state.get("yaw", 0.0)))
        except (KeyError, TypeError, ValueError):
            return None

        dx = float(ped.x) - ego_x
        dy = float(ped.y) - ego_y
        yaw_rad = math.radians(yaw_deg)
        forward = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
        left = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
        distance = math.hypot(dx, dy)
        return forward, left, distance

    def despawn_old_pedestrians(self, ego_s, ego_state=None):
        behind = float(self.cfg.get("despawn_behind_m", 35.0))
        ahead = float(self.cfg.get("despawn_ahead_m", 160.0))
        protect_moving = bool(
            self.cfg.get("protect_moving_pedestrians_from_distance_despawn", True)
        )
        walking_use_ego_frame = bool(self.cfg.get("walking_despawn_use_ego_frame", True))
        keep = []
        for ped in self.live_pedestrians:
            rel_s = float(ped.route_s) - float(ego_s)
            is_walking = self.is_walking_behavior(ped.behavior)
            is_moving = is_walking and not getattr(ped, "stopped", False)
            if is_moving and protect_moving:
                keep.append(ped)
                continue

            frame = self.pedestrian_ego_frame(ped, ego_state)
            if is_walking and walking_use_ego_frame and frame is not None:
                forward, left, distance = frame
                should_despawn = forward < -behind or forward > ahead
                reason = (
                    f"distance forward={forward:.1f} left={left:.1f} "
                    f"xy={distance:.1f} rel_s={rel_s:.1f}"
                )
            else:
                should_despawn = rel_s < -behind or rel_s > ahead
                reason = f"distance rel_s={rel_s:.1f}"

            if should_despawn:
                self.destroy_pedestrian(ped, reason=reason)
            else:
                keep.append(ped)
        self.live_pedestrians = keep

    def sample_behavior(self):
        weights = self.cfg.get("behavior_weights", {}) or {}
        items = [(str(k), max(0.0, float(v))) for k, v in weights.items()]
        total = sum(v for _, v in items)
        if total <= 0.0:
            return "roadside_standing"
        pick = self.rng.uniform(0.0, total)
        acc = 0.0
        for name, weight in items:
            acc += weight
            if pick <= acc:
                return name
        return items[-1][0]

    def is_walking_behavior(self, behavior):
        return str(behavior) in ("roadside_walking", "crosswalk_walking")

    def sample_range(self, name, default):
        value = self.cfg.get(name, default)
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            lo = float(value[0])
            hi = float(value[1])
            if hi < lo:
                lo, hi = hi, lo
            return self.rng.uniform(lo, hi)
        return float(value)

    def normalize_yaw_deg(self, yaw_deg):
        return (float(yaw_deg) + 180.0) % 360.0 - 180.0

    def sample_spawn_yaw_deg(self, route_yaw_deg, behavior):
        if behavior != "roadside_standing":
            return float(route_yaw_deg)

        mode = str(self.cfg.get("standing_yaw_mode", "random")).lower()
        if mode == "route":
            return float(route_yaw_deg)
        if mode == "random":
            return self.rng.uniform(-180.0, 180.0)

        offset = self.sample_range("standing_yaw_offset_deg", [-180.0, 180.0])
        return self.normalize_yaw_deg(float(route_yaw_deg) + offset)

    def is_visible_spawn_candidate(self, x, y, ego_state):
        if not bool(self.cfg.get("reject_spawn_if_visible", True)):
            return False
        if ego_state is None:
            return False
        try:
            ego_x = float(ego_state["x"])
            ego_y = float(ego_state["y"])
            yaw_deg = float(ego_state["yaw_deg"])
        except (KeyError, TypeError, ValueError):
            return False

        dx = float(x) - ego_x
        dy = float(y) - ego_y
        yaw_rad = math.radians(yaw_deg)
        forward = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
        left = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        front = float(self.cfg.get("visible_spawn_front_m", 90.0))
        rear = float(self.cfg.get("visible_spawn_rear_m", 5.0))
        side = float(self.cfg.get("visible_spawn_side_m", 18.0))
        return -rear <= forward <= front and abs(left) <= side

    def candidates_in_spawn_window(self, candidates, ego_s):
        ahead_cfg = self.cfg.get("spawn_ahead_m", [35.0, 120.0])
        if isinstance(ahead_cfg, (list, tuple)) and len(ahead_cfg) >= 2:
            ahead_min = float(ahead_cfg[0])
            ahead_max = float(ahead_cfg[1])
        else:
            ahead_min = ahead_max = float(ahead_cfg)
        if ahead_max < ahead_min:
            ahead_min, ahead_max = ahead_max, ahead_min
        start_s = float(ego_s) + ahead_min
        end_s = float(ego_s) + ahead_max
        return [
            candidate
            for candidate in candidates
            if start_s <= float(candidate.get("route_s", -1.0)) <= end_s
        ]

    def sample_legacy_route_candidate(self, ego_s, behavior):
        ahead = self.sample_range("spawn_ahead_m", [35.0, 120.0])
        route_s = float(ego_s) + ahead
        if route_s >= self.route_helper.route_length_m - 2.0:
            return None
        side_mag = self.sample_range("side_offset_m", [3.0, 6.0])
        side_offset = side_mag * self.rng.choice([-1.0, 1.0])
        x, y, z, yaw_deg = self.route_helper.pose_at(route_s, side_offset)
        return {
            "source": "route_offset",
            "route_s": route_s,
            "x": x,
            "y": y,
            "z": z,
            "yaw_deg": yaw_deg,
            "side_offset": side_offset,
            "control_dir_x": None,
            "control_dir_y": None,
        }

    def sample_standing_candidate(self, ego_s):
        choices = self.candidates_in_spawn_window(self.standing_candidates, ego_s)
        if choices:
            candidate = self.rng.choice(choices)
            offset = self.sample_range("standing_boundary_offset_m", [0.8, 2.0])
            side_sign = float(candidate["side_sign"])
            return {
                "source": candidate["source"],
                "route_s": float(candidate["route_s"]),
                "x": float(candidate["x"]) + float(candidate["normal_x"]) * offset,
                "y": float(candidate["y"]) + float(candidate["normal_y"]) * offset,
                "z": float(candidate["z"]),
                "yaw_deg": float(candidate["route_yaw_deg"]),
                "side_offset": side_sign * offset,
                "control_dir_x": None,
                "control_dir_y": None,
            }
        if not bool(self.cfg.get("standing_fallback_to_route", False)):
            return None
        return self.sample_legacy_route_candidate(ego_s, "roadside_standing")

    def sample_crosswalk_candidate(self, ego_s):
        choices = self.candidates_in_spawn_window(self.crosswalk_candidates, ego_s)
        if not choices:
            return None
        candidate = self.rng.choice(choices)
        start = candidate["start"]
        end = candidate["end"]
        if self.rng.random() < 0.5:
            start, end = end, start
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return None
        direction_x = dx / length
        direction_y = dy / length
        yaw_deg = math.degrees(math.atan2(direction_y, direction_x))
        route_s = float(candidate["route_s"])
        side_offset = dist_xy(float(start[0]), float(start[1]), candidate["center_x"], candidate["center_y"])
        return {
            "source": "crosswalk",
            "route_s": route_s,
            "x": float(start[0]),
            "y": float(start[1]),
            "z": float(start[2]) if len(start) >= 3 else float(candidate["z"]),
            "yaw_deg": yaw_deg,
            "side_offset": side_offset,
            "control_dir_x": direction_x,
            "control_dir_y": direction_y,
            "target_x": float(end[0]),
            "target_y": float(end[1]),
        }

    def sample_spawn_candidate(self, ego_s, behavior):
        if behavior == "roadside_standing":
            return self.sample_standing_candidate(ego_s)
        if self.is_walking_behavior(behavior):
            candidate = self.sample_crosswalk_candidate(ego_s)
            if candidate is not None:
                return candidate
            if not bool(self.cfg.get("walking_fallback_to_route", False)):
                return None
            return self.sample_legacy_route_candidate(ego_s, behavior)
        return self.sample_legacy_route_candidate(ego_s, behavior)

    def try_spawn_one(self, ego_s, ego_state=None):
        if self.route_helper.route_length_m <= 0.0:
            return False
        model_name = self.model_selector.choose()
        if not model_name:
            print("[Ped] spawn skipped: no pedestrian models available")
            return False

        attempts = int(self.cfg.get("max_spawn_attempts", 20))
        min_ego = float(self.cfg.get("min_distance_from_ego_m", 18.0))
        min_between = float(self.cfg.get("min_distance_between_pedestrians_m", 10.0))
        for _ in range(max(1, attempts)):
            behavior = self.sample_behavior()
            spawn = self.sample_spawn_candidate(float(ego_s), behavior)
            if spawn is None:
                continue
            route_s = float(spawn["route_s"])
            x = float(spawn["x"])
            y = float(spawn["y"])
            z = float(spawn["z"])
            yaw_deg = float(spawn["yaw_deg"])
            side_offset = float(spawn["side_offset"])

            if ego_state is not None:
                if dist_xy(x, y, float(ego_state["x"]), float(ego_state["y"])) < min_ego:
                    continue
                if self.is_visible_spawn_candidate(x, y, ego_state):
                    continue
            too_close = False
            for ped in self.live_pedestrians:
                if dist_xy(x, y, ped.x, ped.y) < min_between:
                    too_close = True
                    break
            if too_close:
                continue

            speed = 0.0
            if self.is_walking_behavior(behavior):
                speed = self.sample_range("walking_speed_mps", [0.8, 1.3])
            spawn_yaw_deg = self.sample_spawn_yaw_deg(yaw_deg, behavior)
            label = f"AIM_PED_{self.next_id:04d}"
            self.next_id += 1
            transform = self.sim_bridge.make_transform(x, y, z, spawn_yaw_deg)
            actor = self.sim_bridge.spawn_pedestrian(
                transform,
                model_name=model_name,
                label=label,
                velocity=speed,
                active_dist=float(self.cfg.get("active_dist_m", 30.0)),
                move_dist=float(self.cfg.get("move_dist_m", 30.0)),
                start_action=self.is_walking_behavior(behavior),
            )
            if actor is None:
                continue

            ped = ManagedPedestrian(
                label=label,
                actor=actor,
                route_s=route_s,
                x=x,
                y=y,
                behavior=behavior,
                side_offset_m=side_offset,
                walking_speed_mps=speed,
                control_dir_x=spawn.get("control_dir_x"),
                control_dir_y=spawn.get("control_dir_y"),
                target_x=spawn.get("target_x"),
                target_y=spawn.get("target_y"),
            )
            self.live_pedestrians.append(ped)
            if self.is_walking_behavior(behavior):
                ped.last_control_time = 0.0
            print(
                f"[Ped] spawn label={label} model={model_name} "
                f"behavior={behavior} source={spawn.get('source')} "
                f"s={route_s:.1f} side={side_offset:.1f} "
                f"yaw={spawn_yaw_deg:.1f} speed={speed:.1f} pos=({x:.1f},{y:.1f})"
            )
            return True
        return False

    def log_status(self, ego_s):
        interval = float(self.cfg.get("status_log_interval_sec", 5.0))
        now = time.time()
        if interval <= 0.0 or now - self.last_status_log_time < interval:
            return
        self.last_status_log_time = now
        detail = " | ".join(self.format_status_item(ped, ego_s) for ped in self.live_pedestrians)
        print(f"[Ped] status live={len(self.live_pedestrians)}/{self.target_count} {detail}")

    def format_status_item(self, ped, ego_s):
        moving = self.is_walking_behavior(ped.behavior) and not getattr(
            ped, "stopped", False
        )
        state = "moving" if moving else "stopped"
        if getattr(ped, "stopped", False) and getattr(ped, "stop_reason", None):
            state = f"{state}:{ped.stop_reason}"
        return (
            f"{ped.label}:{ped.behavior}:s={ped.route_s - ego_s:.1f}:"
            f"side={ped.side_offset_m:.1f}:"
            f"{state}"
        )
