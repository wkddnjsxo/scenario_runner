# -*- coding: utf-8 -*-

import math
import os
import random
import socket
import threading
import time
from urllib.parse import urlparse

from utils.geometry_utils import (
    dist_xy,
    interpolate_on_polyline,
    polyline_length,
    project_distance_on_polyline,
)


DEFAULT_NPC_CONFIG = {
    "enabled": False,
    "update_interval_sec": 0.5,
    "max_spawn_attempts": 20,
    "spawn_margin_from_route_end_m": 8.0,
    "vehicle_model": {
        "allowed_categories": ["sedan", "suv", "mpv", "wagon"],
        "exclude_prefixes": ["Default_", "Defalut_", "DEFAULT_"],
        "whitelist": [],
    },
    "spawn_slots": {
        "front": {"s_offset_m": [20.0, 30.0], "lateral_offset_m": 0.0},
        "left_front": {
            "s_offset_m": [0.0, 20.0],
            "adjacent": "left",
        },
        "right_front": {
            "s_offset_m": [0.0, 20.0],
            "adjacent": "right",
        },
        "opposite_front": {
            "s_offset_m": [45.0, 60.0],
            "lateral_offset_m": 0.0,
            "opposite": True,
        },
    },
    "spawn_slots_by_road_group": {
        "1차선": ["front", "opposite_front"],
        "2차선(오른쪽)": ["front", "front", "opposite_front", "opposite_front", "left_front"],
        "2차선(왼쪽)": ["front", "front", "opposite_front", "opposite_front", "right_front"],
        "3차선(중간)": [
            "front",
            "front",
            "front",
            "front",
            "opposite_front",
            "opposite_front",
            "opposite_front",
            "opposite_front",
            "left_front",
            "right_front",
        ],
        "3차선(오른쪽)": ["front", "front", "opposite_front", "opposite_front", "left_front"],
        "3차선(왼쪽)": ["front", "front", "opposite_front", "opposite_front", "right_front"],
        "일방통행": ["front"],
        "default": ["front"],
    },
    "road_groups": {
        "일방통행": {
            "has_opposite_lane": False,
            "opposite_links": {},
        }
    },
    "spawn_management": {
        "target_npc_count_min": 3,
        "target_npc_count_max": 5,
        "target_npc_count_by_road_group": {
            "일방통행": [1, 2],
            "1차선": [3, 4],
            "2차선(오른쪽)": [3, 4],
            "2차선(왼쪽)": [3, 4],
            "3차선(중간)": [3, 4],
            "3차선(오른쪽)": [3, 4],
            "3차선(왼쪽)": [3, 4],
        },
        "despawn_distance_behind_m": 210.0,
        "despawn_distance_ahead_m": 390.0,
        "despawn_keep_radius_m": 80.0,
        "despawn_keep_route_distance_m": 90.0,
        "route_projection_back_window_m": 40.0,
        "route_projection_front_window_m": 80.0,
        "min_distance_from_ego_m": 18.0,
        "min_xy_distance_from_ego_m": 18.0,
        "min_distance_between_npcs_m": 18.0,
        "initial_spawn_settle_sec": 1.0,
        "spawn_grace_sec": 3.0,
        "visibility_grace_sec": 3.0,
        "outside_confirm_frames": 10,
        "camera_delete_front_m": 130.0,
        "camera_delete_rear_m": 40.0,
        "camera_delete_side_m": 60.0,
        "replacement_spawn_offset_m": [80.0, 170.0],
        "replacement_spawn_step_m": 10.0,
        "replacement_min_remaining_route_m": 80.0,
        "replacement_min_forward_m": 25.0,
        "replacement_max_forward_m": 180.0,
        "replacement_max_abs_left_m": 90.0,
        "npc_status_log_interval_sec": 5.0,
        "spawn_deferred_log_interval_sec": 5.0,
        "wait_until_ego_close": True,
        "activate_distance_m": [20.0, 30.0],
        "opposite_activate_distance_m": [40.0, 55.0],
        "waiting_stop_refresh_interval_sec": 0.5,
        "active_speed_hold_interval_sec": 0.5,
        "speed_guard_enabled": True,
        "speed_guard_max_kmh": 16.0,
        "speed_guard_correction_interval_sec": 1.0,
        "protect_waiting_npcs": True,
        "waiting_despawn_behind_m": 30.0,
        "stuck_npc_cleanup_enabled": True,
        "stuck_npc_grace_sec": 6.0,
        "stuck_npc_speed_mps": 0.5,
        "stuck_npc_duration_sec": 3.0,
        "npc_state_miss_remove_count": 20,
        "npc_state_query_bulk_first": True,
        "npc_state_individual_fallback": True,
        "npc_state_individual_fallback_interval_sec": 1.0,
        "npc_state_miss_log_interval_sec": 2.0,
        "npc_state_recovery_log_min_misses": 2,
        "speed_ramp_enabled": True,
        "visible_activation_start_speed_kmh": 8.0,
        "speed_ramp_step_kmh": 2.0,
        "speed_ramp_interval_sec": 1.0,
        "same_direction_route_extension_m": 320.0,
        "same_direction_route_extension_max_links": 18,
        "opposite_route_min_length_m": 320.0,
        "opposite_route_max_links": 18,
        "adjacent_route_min_length_m": 260.0,
        "adjacent_route_max_links": 18,
        "adjacent_route_allow_unavoidable_overlap": True,
    },
    "speed": {
        "fixed_kmh": 12.0,
        "same_direction_kmh": [12.0, 12.0],
        "opposite_direction_kmh": [12.0, 12.0],
        "stopped_probability": 0.0,
    },
    "ros_object_state": {
        "enabled": True,
        "topic": "/Object_topic",
        "max_age_sec": 1.0,
    },
    "camera_visibility": {
        "enabled": True,
        "margin_deg": 10.0,
        "margin_m": 15.0,
        "spawn_margin_deg": 5.0,
        "spawn_margin_m": 10.0,
        "reject_spawn_if_visible": True,
        "cameras": {
            "cam_front": {
                "translation": [1.92, 0.0, 1.21],
                "pitch_deg": 3.0,
                "yaw_deg": 0.0,
                "fov_deg": 70.0,
                "max_range_m": 120.0,
            },
            "cam_front_right": {
                "translation": [1.92, -0.56, 1.21],
                "pitch_deg": 3.0,
                "yaw_deg": -45.0,
                "fov_deg": 70.0,
                "max_range_m": 120.0,
            },
            "cam_front_left": {
                "translation": [1.92, 0.56, 1.21],
                "pitch_deg": 3.0,
                "yaw_deg": 45.0,
                "fov_deg": 70.0,
                "max_range_m": 120.0,
            },
        },
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
    def __init__(self, route_points, route_link_spans=None):
        self.route_points = list(route_points or [])
        self.route_length_m = polyline_length(self.route_points) if len(self.route_points) >= 2 else 0.0
        self.route_link_spans = list(route_link_spans or [])

    def pose_at(self, route_s, lateral_offset_m=0.0):
        if len(self.route_points) < 2:
            raise ValueError("Route must have at least 2 points")

        route_s = max(0.0, min(float(route_s), self.route_length_m))
        x, y, z, yaw_deg = interpolate_on_polyline(self.route_points, route_s)
        if abs(float(lateral_offset_m)) > 1e-6:
            yaw_rad = math.radians(yaw_deg)
            x += -math.sin(yaw_rad) * float(lateral_offset_m)
            y += math.cos(yaw_rad) * float(lateral_offset_m)
        return x, y, z, yaw_deg

    def project_s(self, x, y):
        return project_distance_on_polyline(self.route_points, x, y)

    def project_s_near(self, x, y, prev_s, back_window_m=40.0, front_window_m=80.0):
        if len(self.route_points) < 2:
            raise ValueError("Route must have at least 2 points")
        if prev_s is None:
            return self.project_s(x, y)

        min_s = max(0.0, float(prev_s) - float(back_window_m))
        max_s = min(self.route_length_m, float(prev_s) + float(front_window_m))
        best_s = float(prev_s)
        best_dist = float("inf")
        cumulative = 0.0

        for p0, p1 in zip(self.route_points[:-1], self.route_points[1:]):
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-12:
                continue

            seg_len = math.sqrt(seg_len_sq)
            seg_start_s = cumulative
            seg_end_s = cumulative + seg_len
            cumulative = seg_end_s
            if seg_end_s < min_s or seg_start_s > max_s:
                continue

            t = ((float(x) - p0[0]) * dx + (float(y) - p0[1]) * dy) / seg_len_sq
            t = max(0.0, min(1.0, t))
            candidate_s = seg_start_s + t * seg_len
            if candidate_s < min_s or candidate_s > max_s:
                continue

            px = p0[0] + t * dx
            py = p0[1] + t * dy
            d = dist_xy(float(x), float(y), px, py)
            if d < best_dist:
                best_dist = d
                best_s = candidate_s

        if best_dist == float("inf"):
            return self.project_s(x, y)
        return best_s

    def link_at_s(self, route_s):
        route_s = float(route_s)
        for item in self.route_link_spans:
            if float(item["start_s"]) <= route_s <= float(item["end_s"]):
                return item["link_id"]
        if self.route_link_spans:
            return self.route_link_spans[-1]["link_id"]
        return None

    def link_index_at_s(self, route_s):
        route_s = float(route_s)
        for idx, item in enumerate(self.route_link_spans):
            if float(item["start_s"]) <= route_s <= float(item["end_s"]):
                return idx
        if self.route_link_spans:
            return len(self.route_link_spans) - 1
        return None


class NpcVehicleModelSelector:
    def __init__(self, cfg, rng):
        self.cfg = cfg or {}
        self.rng = rng
        self.models = []
        self.initialized = False

    def initialize(self, sim_bridge):
        if self.initialized:
            return
        self.initialized = True

        whitelist = [str(item) for item in self.cfg.get("whitelist", []) if item]
        if whitelist:
            self.models = self._filter_models(whitelist, require_category=False)
        else:
            self.models = self._filter_models(
                sim_bridge.get_available_surround_vehicle_models(),
                require_category=True,
            )

        if self.models:
            print(f"[NPC] vehicle model candidates={len(self.models)}")
        else:
            print("[NPC] no valid vehicle model candidates; NPC spawn disabled")

    def _filter_models(self, models, require_category=True):
        allowed = [str(item).lower() for item in self.cfg.get("allowed_categories", [])]
        exclude_prefixes = [
            str(item).lower()
            for item in self.cfg.get("exclude_prefixes", [])
            if item
        ]

        out = []
        for model in models or []:
            name = str(model)
            lower = name.lower()
            if any(lower.startswith(prefix) for prefix in exclude_prefixes):
                continue
            if require_category and allowed and not any(category in lower for category in allowed):
                continue
            if name not in out:
                out.append(name)
        return out

    def choose(self):
        if not self.models:
            return None
        return self.rng.choice(self.models)


class NpcSpawnSlotSampler:
    def __init__(self, cfg, rng):
        self.cfg = cfg or {}
        self.rng = rng

    def allowed_slots(self, road_group):
        by_group = self.cfg.get("spawn_slots_by_road_group", {})
        slots = by_group.get(road_group)
        if slots is None:
            slots = by_group.get("default", ["front"])
        return [slot for slot in slots if slot in self.cfg.get("spawn_slots", {})]

    def sample(self, road_group):
        slots = self.allowed_slots(road_group)
        if not slots:
            return None

        slot_name = self.rng.choice(slots)
        slot_cfg = self.cfg["spawn_slots"][slot_name]
        s_range = slot_cfg.get("s_offset_m", [20.0, 35.0])
        if not isinstance(s_range, (list, tuple)) or len(s_range) != 2:
            s_range = [float(s_range), float(s_range)]

        lo = float(min(s_range[0], s_range[1]))
        hi = float(max(s_range[0], s_range[1]))
        s_offset = self.rng.uniform(lo, hi)
        return {
            "slot": slot_name,
            "s_offset_m": s_offset,
            "lateral_offset_m": float(slot_cfg.get("lateral_offset_m", 0.0)),
            "opposite": bool(slot_cfg.get("opposite", False)),
            "adjacent": slot_cfg.get("adjacent"),
            "allow_visible_spawn": bool(slot_cfg.get("allow_visible_spawn", False)),
            "allow_close_spawn": bool(slot_cfg.get("allow_close_spawn", False)),
        }

    def sample_front_edge(self, road_group):
        def offset_range(slot_cfg):
            s_range = slot_cfg.get("s_offset_m", [0.0, 0.0])
            if not isinstance(s_range, (list, tuple)) or len(s_range) != 2:
                value = float(s_range)
                return value, value
            return float(s_range[0]), float(s_range[1])

        slots = self.allowed_slots(road_group)
        positive_slots = []
        for candidate in slots:
            slot_cfg = self.cfg.get("spawn_slots", {}).get(candidate, {})
            if max(offset_range(slot_cfg)) > 0.0:
                positive_slots.append(candidate)
        if not positive_slots:
            return self.sample(road_group)

        slot_name = self.rng.choice(positive_slots)

        slot_cfg = self.cfg["spawn_slots"][slot_name]
        s_offset = max(offset_range(slot_cfg))
        return {
            "slot": slot_name,
            "s_offset_m": s_offset,
            "lateral_offset_m": float(slot_cfg.get("lateral_offset_m", 0.0)),
            "opposite": bool(slot_cfg.get("opposite", False)),
            "adjacent": slot_cfg.get("adjacent"),
            "allow_visible_spawn": bool(slot_cfg.get("allow_visible_spawn", False)),
            "allow_close_spawn": bool(slot_cfg.get("allow_close_spawn", False)),
        }


class NpcSpawnValidator:
    def __init__(self, cfg):
        self.cfg = cfg or {}

    def validate(self, candidate, ego_s, route_length_m, live_npcs):
        management = self.cfg.get("spawn_management", {})
        margin = float(self.cfg.get("spawn_margin_from_route_end_m", 8.0))
        min_ego = float(management.get("min_distance_from_ego_m", 15.0))
        min_npc = float(management.get("min_distance_between_npcs_m", 10.0))

        spawn_s = float(candidate["route_s"])
        if spawn_s < margin or spawn_s > route_length_m - margin:
            return False

        if (
            not bool(candidate.get("allow_close_spawn", False))
            and abs(spawn_s - ego_s) < min_ego
        ):
            return False

        x = float(candidate["x"])
        y = float(candidate["y"])
        for npc in live_npcs:
            if abs(float(npc.route_s) - spawn_s) < min_npc:
                return False
            if dist_xy(x, y, npc.x, npc.y) < min_npc:
                return False

        return True


class ManagedNpcVehicle:
    def __init__(
        self,
        label,
        vehicle,
        route_s,
        x,
        y,
        model_name,
        slot,
        speed_kmh,
        stopped,
        opposite=False,
        route_links=None,
        waiting=False,
        activate_distance_m=None,
    ):
        self.label = label
        self.vehicle = vehicle
        self.route_s = float(route_s)
        self.x = float(x)
        self.y = float(y)
        self.model_name = model_name
        self.slot = slot
        self.speed_kmh = float(speed_kmh)
        self.stopped = bool(stopped)
        self.opposite = bool(opposite)
        self.route_links = list(route_links or [])
        self.waiting = bool(waiting)
        self.activate_distance_m = (
            None if activate_distance_m is None else float(activate_distance_m)
        )
        self.activated = not self.waiting
        self.spawn_time = time.time()
        self.state_miss_count = 0
        self.last_camera_visible_time = 0.0
        self.outside_camera_frames = 0
        self.ramping = False
        self.current_speed_kmh = 0.0 if self.waiting or self.stopped else float(speed_kmh)
        self.last_speed_update_time = 0.0
        self.last_speed_hold_time = 0.0
        self.last_speed_guard_time = 0.0
        self.last_wait_stop_time = 0.0
        self.last_motion_x = float(x)
        self.last_motion_y = float(y)
        self.last_motion_time = self.spawn_time
        self.estimated_speed_mps = 0.0
        self.low_motion_since = None
        self.last_progress_s = float(route_s)
        self.low_progress_since = None
        self.has_first_state = False
        self.last_state_time = 0.0
        self.last_state_source = "spawn"
        self.state_miss_first_time = 0.0
        self.last_state_miss_log_time = 0.0
        self.last_individual_state_query_time = 0.0


class RosObjectStateCache:
    def __init__(self, cfg):
        self.cfg = cfg or {}
        self.enabled = bool(self.cfg.get("enabled", True))
        self.topic = str(self.cfg.get("topic", "/Object_topic"))
        self.max_age_sec = float(self.cfg.get("max_age_sec", 1.0))
        self.lock = threading.Lock()
        self.states_by_label = {}
        self.started = False
        self.subscriber = None

    def ensure_started(self):
        if not self.enabled or self.started:
            return
        self.started = True
        if not self.ros_master_available():
            self.enabled = False
            print(
                "[NPC] ROS object state disabled: "
                f"ROS master unavailable ({self.ros_master_uri()})"
            )
            return
        try:
            import rospy
            from morai_msgs.msg import ObjectStatusList

            if not rospy.core.is_initialized():
                rospy.init_node(
                    "aim_scenario_runner_npc_state",
                    anonymous=True,
                    disable_signals=True,
                )
            self.subscriber = rospy.Subscriber(
                self.topic,
                ObjectStatusList,
                self._object_topic_cb,
                queue_size=1,
            )
            print(f"[NPC] ROS object state subscribed: {self.topic}")
        except Exception as exc:
            self.enabled = False
            print(f"[NPC] ROS object state disabled: {exc}")

    def ros_master_uri(self):
        return os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311")

    def ros_master_available(self):
        try:
            parsed = urlparse(self.ros_master_uri())
            host = parsed.hostname or "127.0.0.1"
            port = parsed.port or 11311
            with socket.create_connection((host, port), timeout=0.25):
                return True
        except Exception:
            return False

    def _object_topic_cb(self, msg):
        now = time.time()
        states = {}
        for obj in getattr(msg, "npc_list", []) or []:
            label = str(getattr(obj, "name", "") or "").strip()
            if not label:
                continue
            state = self._state_from_object(obj, now)
            states[label] = state
            short_label = label.split("/")[-1]
            if short_label:
                states[short_label] = state
        with self.lock:
            self.states_by_label = states

    def _state_from_object(self, obj, stamp):
        vx = float(getattr(getattr(obj, "velocity", None), "x", 0.0))
        vy = float(getattr(getattr(obj, "velocity", None), "y", 0.0))
        vz = float(getattr(getattr(obj, "velocity", None), "z", 0.0))
        speed_mps = math.sqrt(vx * vx + vy * vy + vz * vz) / 3.6
        return {
            "x": float(obj.position.x),
            "y": float(obj.position.y),
            "yaw_deg": float(obj.heading),
            "speed_mps": speed_mps,
            "stamp": float(stamp),
            "source": "ros",
        }

    def get(self, label):
        if not self.enabled:
            return None
        self.ensure_started()
        now = time.time()
        with self.lock:
            state = self.states_by_label.get(str(label))
            if state is None:
                return None
            state = dict(state)
        if self.max_age_sec > 0.0 and now - float(state["stamp"]) > self.max_age_sec:
            return None
        return state


class NpcVehicleManager:
    def __init__(self, sim_bridge, cfg, rng=None, map_loader=None):
        self.sim_bridge = sim_bridge
        self.map_loader = map_loader
        self.cfg = deep_merge_dict(DEFAULT_NPC_CONFIG, cfg or {})
        self.rng = rng or random.Random()
        self.model_selector = NpcVehicleModelSelector(self.cfg.get("vehicle_model"), self.rng)
        self.slot_sampler = NpcSpawnSlotSampler(self.cfg, self.rng)
        self.validator = NpcSpawnValidator(self.cfg)
        self.route_helper = None
        self.route_links = []
        self.road_group = "default"
        self.live_npcs = []
        self.next_id = 1
        self.last_update_time = 0.0
        self.enabled = bool(self.cfg.get("enabled", False))
        self.opposite_link_cache = {}
        self.opposite_route_cache = {}
        self.adjacent_route_cache = {}
        self.same_direction_route_extension_cache = {}
        self.failed_route_signatures = set()
        self.logged_same_direction_extensions = set()
        self.logged_duplicate_route_trims = set()
        self.road_graph = None
        self.target_npc_count = 0
        self.pending_front_wait_replacements = 0
        self.last_status_log_time = 0.0
        self.last_spawn_deferred_log_time = 0.0
        self.last_spawn_deferred_signature = None
        self.ros_object_state_cache = RosObjectStateCache(
            self.cfg.get("ros_object_state", {})
        )
        self.ros_object_state_cache.ensure_started()

    def set_route(self, route_points, route_links, road_group, route_link_spans=None):
        self.route_helper = RoutePositionHelper(route_points, route_link_spans=route_link_spans)
        self.route_links = list(route_links or [])
        self.road_group = road_group or "default"
        self.last_update_time = 0.0
        self.target_npc_count = self.select_target_npc_count()
        self.pending_front_wait_replacements = 0
        self.last_status_log_time = 0.0
        self.last_spawn_deferred_log_time = 0.0
        self.last_spawn_deferred_signature = None
        self.opposite_route_cache = {}
        self.adjacent_route_cache = {}
        self.same_direction_route_extension_cache = {}
        self.failed_route_signatures = set()
        self.logged_same_direction_extensions = set()
        self.logged_duplicate_route_trims = set()
        print(
            f"[NPC] route configured group={self.road_group}, "
            f"length={self.route_helper.route_length_m:.1f}m, "
            f"target_count={self.target_npc_count}"
        )

    def select_target_npc_count(self):
        management = self.cfg.get("spawn_management", {})
        if "target_npc_count" in management:
            return max(0, int(management.get("target_npc_count", 5)))

        count_by_group = management.get("target_npc_count_by_road_group", {}) or {}
        count_range = count_by_group.get(self.road_group)
        if isinstance(count_range, (list, tuple)) and len(count_range) == 2:
            lo = int(count_range[0])
            hi = int(count_range[1])
        else:
            lo = int(management.get("target_npc_count_min", 3))
            hi = int(management.get("target_npc_count_max", 5))
        if lo > hi:
            lo, hi = hi, lo
        return max(0, self.rng.randint(lo, hi))

    def sample_management_range(self, key, default_value):
        management = self.cfg.get("spawn_management", {})
        value = management.get(key, default_value)
        if isinstance(value, (list, tuple)) and len(value) == 2:
            lo = float(min(value[0], value[1]))
            hi = float(max(value[0], value[1]))
            return self.rng.uniform(lo, hi)
        return float(value)

    def sample_activate_distance(self, opposite=False):
        if opposite:
            fallback = self.cfg.get("spawn_management", {}).get(
                "activate_distance_m",
                [20.0, 30.0],
            )
            return self.sample_management_range(
                "opposite_activate_distance_m",
                fallback,
            )
        return self.sample_management_range(
            "activate_distance_m",
            [20.0, 30.0],
        )

    def reset_live_refs(self):
        self.live_npcs = []
        self.last_update_time = 0.0
        self.pending_front_wait_replacements = 0
        self.last_status_log_time = 0.0

    def destroy_all(self, reason="cleanup"):
        for npc in list(self.live_npcs):
            self.destroy_npc(npc, reason=reason)
        self.live_npcs = []

    def destroy_npc(self, npc, reason="unknown"):
        try:
            if npc.vehicle is not None:
                npc.vehicle.destroy()
        except Exception as exc:
            print(f"[NPC] destroy failed label={npc.label} reason={reason}: {exc}")
        print(f"[NPC] despawn label={npc.label} reason={reason}")

    def update(self, ego_s, ego_state=None):
        if not self.enabled or self.route_helper is None:
            return

        now = time.time()
        interval = float(self.cfg.get("update_interval_sec", 0.5))
        if now - self.last_update_time < interval:
            return
        self.last_update_time = now

        self.model_selector.initialize(self.sim_bridge)
        if not self.model_selector.models:
            return

        state_missing_despawns = self.refresh_npc_positions()
        self.pending_front_wait_replacements += state_missing_despawns
        despawned_count = self.despawn_old_npcs(ego_s, ego_state=ego_state)
        self.pending_front_wait_replacements += despawned_count
        self.activate_waiting_npcs(ego_s, ego_state=ego_state)
        self.update_speed_ramps()
        self.enforce_waiting_vehicle_stops()
        self.enforce_active_vehicle_speeds()
        self.enforce_active_vehicle_speed_guard()
        self.log_npc_status(ego_s, ego_state=ego_state)

        target_count = int(self.target_npc_count)
        missing = max(0, target_count - len(self.live_npcs))
        for _ in range(missing):
            force_front_wait = self.pending_front_wait_replacements > 0
            spawned = self.try_spawn_one(
                ego_s,
                force_front_wait=force_front_wait,
                ego_state=ego_state,
            )
            if spawned and force_front_wait:
                self.pending_front_wait_replacements = max(
                    0,
                    self.pending_front_wait_replacements - 1,
                )

    def spawn_initial_npcs(self, ego_s=0.0, ego_state=None):
        if not self.enabled or self.route_helper is None:
            return 0

        self.model_selector.initialize(self.sim_bridge)
        if not self.model_selector.models:
            return 0

        target_count = int(self.target_npc_count)
        print(f"[NPC] initial spawn begin target={target_count}")
        spawned_count = 0
        if target_count > 0 and not self.live_npcs:
            if self.try_spawn_one(
                ego_s,
                force_front_wait=True,
                ego_state=ego_state,
                reject_visible_spawn=False,
                release_visible_spawn_immediately=False,
            ):
                spawned_count += 1
        max_rounds = max(1, target_count * int(self.cfg.get("max_spawn_attempts", 20)))
        rounds = 0
        while len(self.live_npcs) < target_count and rounds < max_rounds:
            rounds += 1
            if self.try_spawn_one(
                ego_s,
                force_front_wait=False,
                ego_state=ego_state,
                reject_visible_spawn=False,
                release_visible_spawn_immediately=False,
            ):
                spawned_count += 1

        settle_sec = float(
            self.cfg.get("spawn_management", {}).get("initial_spawn_settle_sec", 1.0)
        )
        if settle_sec > 0.0:
            time.sleep(settle_sec)
        self.refresh_npc_positions()
        print(
            f"[NPC] initial spawn complete spawned={spawned_count} "
            f"live={len(self.live_npcs)}/{target_count}"
        )
        return spawned_count

    def grpc_actor_state_to_npc_state(self, state):
        if state is None:
            return None
        return {
            "x": float(state.transform.location.x),
            "y": float(state.transform.location.y),
            "yaw_deg": float(getattr(state.transform.rotation, "z", 0.0)),
            "speed_mps": None,
            "stamp": time.time(),
            "source": "grpc",
        }

    def should_query_individual_state(self, npc, now):
        management = self.cfg.get("spawn_management", {})
        interval = float(
            management.get("npc_state_individual_fallback_interval_sec", 1.0)
        )
        if interval <= 0.0:
            return True
        if npc.last_individual_state_query_time <= 0.0:
            return True
        return now - npc.last_individual_state_query_time >= interval

    def get_current_npc_state(self, npc, states_by_label, allow_actor_query=True):
        state = self.ros_object_state_cache.get(npc.label)
        if state is not None:
            return state

        actor_state = None
        if states_by_label is not None:
            actor_state = states_by_label.get(npc.label)
        if actor_state is None and allow_actor_query:
            try:
                npc.last_individual_state_query_time = time.time()
                actor_state = npc.vehicle.get_actor_state()
            except Exception:
                actor_state = None
        return self.grpc_actor_state_to_npc_state(actor_state)

    def log_state_miss_if_needed(self, npc, now, max_misses):
        management = self.cfg.get("spawn_management", {})
        interval = float(management.get("npc_state_miss_log_interval_sec", 2.0))
        if interval <= 0.0:
            return
        first_miss = npc.state_miss_count == 1
        due = now - npc.last_state_miss_log_time >= interval
        near_remove = max_misses > 0 and npc.state_miss_count >= max(1, max_misses - 2)
        if not (first_miss or due or near_remove):
            return
        npc.last_state_miss_log_time = now
        miss_for = 0.0
        if npc.state_miss_first_time > 0.0:
            miss_for = now - npc.state_miss_first_time
        last_age = -1.0
        if npc.last_state_time > 0.0:
            last_age = now - npc.last_state_time
        print(
            f"[NPC] state miss label={npc.label} "
            f"misses={npc.state_miss_count}/{max_misses} "
            f"miss_for={miss_for:.1f}s last_age={last_age:.1f}s "
            f"keep_last=1 route_s={npc.route_s:.1f}"
        )

    def log_state_recovered_if_needed(self, npc, recovered_misses, now, state):
        management = self.cfg.get("spawn_management", {})
        min_misses = int(management.get("npc_state_recovery_log_min_misses", 2))
        if recovered_misses < min_misses:
            return
        miss_for = 0.0
        if npc.state_miss_first_time > 0.0:
            miss_for = now - npc.state_miss_first_time
        print(
            f"[NPC] state recovered label={npc.label} "
            f"misses={recovered_misses} miss_for={miss_for:.1f}s "
            f"source={state.get('source', 'unknown')}"
        )

    def refresh_npc_positions(self):
        self.ros_object_state_cache.ensure_started()
        management = self.cfg.get("spawn_management", {})
        states_by_label = None
        use_bulk_first = bool(management.get("npc_state_query_bulk_first", True))
        individual_fallback = bool(management.get("npc_state_individual_fallback", False))
        if use_bulk_first and hasattr(self.sim_bridge, "get_all_vehicle_actor_states"):
            states_by_label = self.sim_bridge.get_all_vehicle_actor_states()

        keep = []
        removed_count = 0
        for npc in self.live_npcs:
            now = time.time()
            fallback_due = individual_fallback and self.should_query_individual_state(npc, now)
            allow_actor_query = not use_bulk_first or fallback_due
            state = self.get_current_npc_state(
                npc,
                states_by_label,
                allow_actor_query=allow_actor_query,
            )
            if state is None and states_by_label is None and hasattr(
                self.sim_bridge,
                "get_all_vehicle_actor_states",
            ):
                states_by_label = self.sim_bridge.get_all_vehicle_actor_states()
                fallback_due = individual_fallback and self.should_query_individual_state(npc, now)
                allow_actor_query = fallback_due
                state = self.get_current_npc_state(
                    npc,
                    states_by_label,
                    allow_actor_query=allow_actor_query,
                )

            if state is None:
                if use_bulk_first and individual_fallback and not fallback_due:
                    npc.last_state_source = "miss:throttled"
                    keep.append(npc)
                    continue
                npc.state_miss_count += 1
                now = time.time()
                if npc.state_miss_count == 1:
                    npc.state_miss_first_time = now
                npc.last_state_source = "miss:last"
                npc.low_motion_since = None
                npc.low_progress_since = None
                max_misses = int(management.get("npc_state_miss_remove_count", 2))
                self.log_state_miss_if_needed(npc, now, max_misses)
                if max_misses > 0 and npc.state_miss_count >= max_misses:
                    print(
                        f"[NPC] state-missing despawn decision label={npc.label} "
                        f"misses={npc.state_miss_count}"
                    )
                    self.destroy_npc(npc, reason="state_missing")
                    removed_count += 1
                    continue
                keep.append(npc)
                continue

            now = time.time()
            recovered_misses = npc.state_miss_count
            if recovered_misses > 0:
                self.log_state_recovered_if_needed(npc, recovered_misses, now, state)
            next_x = float(state["x"])
            next_y = float(state["y"])
            dt = max(1e-3, now - float(npc.last_motion_time))
            moved = dist_xy(next_x, next_y, npc.last_motion_x, npc.last_motion_y)
            state_speed_mps = state.get("speed_mps")
            if state_speed_mps is None:
                npc.estimated_speed_mps = moved / dt
            else:
                npc.estimated_speed_mps = float(state_speed_mps)
            npc.last_motion_x = next_x
            npc.last_motion_y = next_y
            npc.last_motion_time = now
            npc.x = next_x
            npc.y = next_y
            npc.state_miss_count = 0
            npc.state_miss_first_time = 0.0
            npc.last_state_miss_log_time = 0.0
            npc.last_state_time = now
            npc.last_state_source = str(state.get("source", "unknown"))
            try:
                management = self.cfg.get("spawn_management", {})
                stuck_speed = float(management.get("stuck_npc_speed_mps", 0.5))
                stuck_progress = float(management.get("stuck_npc_progress_m", 0.5))
                if npc.opposite:
                    npc.route_s = self.route_helper.project_s(npc.x, npc.y)
                else:
                    npc.route_s = self.route_helper.project_s_near(
                        npc.x,
                        npc.y,
                        npc.route_s,
                        back_window_m=float(
                            management.get("route_projection_back_window_m", 40.0)
                        ),
                        front_window_m=float(
                            management.get("route_projection_front_window_m", 80.0)
                        ),
                    )
                if not npc.has_first_state:
                    npc.has_first_state = True
                    npc.last_progress_s = float(npc.route_s)
                    npc.last_motion_x = next_x
                    npc.last_motion_y = next_y
                    npc.low_motion_since = now if not npc.waiting and not npc.stopped else None
                    npc.low_progress_since = now if not npc.waiting and not npc.stopped else None
                    keep.append(npc)
                    continue

                if npc.waiting or npc.stopped or npc.estimated_speed_mps >= stuck_speed:
                    npc.low_motion_since = None
                elif npc.low_motion_since is None:
                    npc.low_motion_since = now

                if npc.waiting or npc.stopped:
                    npc.low_progress_since = None
                    npc.last_progress_s = float(npc.route_s)
                elif abs(float(npc.route_s) - float(npc.last_progress_s)) >= stuck_progress:
                    npc.low_progress_since = None
                    npc.last_progress_s = float(npc.route_s)
                elif npc.low_progress_since is None:
                    npc.low_progress_since = now
            except Exception:
                pass
            keep.append(npc)
        self.live_npcs = keep
        return removed_count

    def ego_xy_from_state(self, ego_state):
        if ego_state is None:
            return None
        try:
            return float(ego_state["x"]), float(ego_state["y"])
        except Exception:
            return None

    def ego_pose_from_state(self, ego_state):
        if ego_state is None:
            return None
        try:
            yaw = ego_state.get("yaw_deg", ego_state.get("yaw"))
            return float(ego_state["x"]), float(ego_state["y"]), float(yaw)
        except Exception:
            return None

    def world_to_ego_xy(self, x, y, ego_pose):
        ego_x, ego_y, ego_yaw_deg = ego_pose
        dx = float(x) - ego_x
        dy = float(y) - ego_y
        yaw = math.radians(ego_yaw_deg)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        forward = dx * cos_y + dy * sin_y
        left = -dx * sin_y + dy * cos_y
        return forward, left

    def is_point_in_camera_visibility(
        self,
        ego_pose,
        x,
        y,
        *,
        margin_deg=None,
        margin_m=None,
    ):
        visibility_cfg = self.cfg.get("camera_visibility", {})
        if not visibility_cfg.get("enabled", False):
            return False
        if ego_pose is None:
            return False

        margin_deg = float(
            visibility_cfg.get("margin_deg", 10.0)
            if margin_deg is None
            else margin_deg
        )
        margin_m = float(
            visibility_cfg.get("margin_m", 15.0)
            if margin_m is None
            else margin_m
        )
        cameras = visibility_cfg.get("cameras", {}) or {}
        ego_forward, ego_left = self.world_to_ego_xy(x, y, ego_pose)

        for camera in cameras.values():
            translation = camera.get("translation", [0.0, 0.0, 0.0])
            try:
                cam_forward = float(translation[0])
                cam_left = float(translation[1])
            except Exception:
                cam_forward = 0.0
                cam_left = 0.0

            rel_forward = ego_forward - cam_forward
            rel_left = ego_left - cam_left
            distance = math.hypot(rel_forward, rel_left)
            max_range = float(camera.get("max_range_m", 120.0)) + margin_m
            if distance <= 1e-6 or distance > max_range:
                continue

            angle = math.degrees(math.atan2(rel_left, rel_forward))
            camera_yaw = float(camera.get("yaw_deg", 0.0))
            half_fov = 0.5 * float(camera.get("fov_deg", 70.0)) + margin_deg
            if abs(self.normalize_angle_deg(angle - camera_yaw)) <= half_fov:
                return True
        return False

    def update_camera_visibility_state(self, npc, ego_pose, now):
        visible = self.is_point_in_camera_visibility(ego_pose, npc.x, npc.y)
        if visible:
            npc.last_camera_visible_time = now
            npc.outside_camera_frames = 0
        else:
            npc.outside_camera_frames += 1
        return visible

    def is_outside_camera_delete_limits(self, npc, ego_pose, management):
        forward, left = self.world_to_ego_xy(npc.x, npc.y, ego_pose)
        front_m = float(management.get("camera_delete_front_m", 130.0))
        rear_m = float(management.get("camera_delete_rear_m", 40.0))
        side_m = float(management.get("camera_delete_side_m", 60.0))
        outside = forward > front_m or forward < -rear_m or abs(left) > side_m
        return outside, forward, left

    def log_npc_status(self, ego_s, ego_state=None):
        management = self.cfg.get("spawn_management", {})
        interval = float(management.get("npc_status_log_interval_sec", 5.0))
        if interval <= 0.0:
            return
        now = time.time()
        if now - self.last_status_log_time < interval:
            return
        self.last_status_log_time = now

        ego_pose = self.ego_pose_from_state(ego_state)
        items = []
        for npc in self.live_npcs:
            rel_s = npc.route_s - float(ego_s)
            if ego_pose is not None:
                forward, left = self.world_to_ego_xy(npc.x, npc.y, ego_pose)
                visible = self.is_point_in_camera_visibility(ego_pose, npc.x, npc.y)
                pos_msg = f"f={forward:.1f} l={left:.1f} vis={int(visible)}"
            else:
                pos_msg = "f=? l=? vis=?"
            items.append(
                f"{npc.label}:{npc.slot}:s={rel_s:.1f}:{pos_msg}:"
                f"wait={int(npc.waiting)}:out={npc.outside_camera_frames}:"
                f"src={npc.last_state_source}:"
                f"v={npc.estimated_speed_mps:.1f}mps/"
                f"{npc.estimated_speed_mps * 3.6:.1f}kmh:"
                f"stuck={0.0 if npc.low_motion_since is None else now - npc.low_motion_since:.1f}:"
                f"prog_stuck={0.0 if npc.low_progress_since is None else now - npc.low_progress_since:.1f}"
            )
        detail = " | ".join(items) if items else "(none)"
        print(
            f"[NPC] status live={len(self.live_npcs)}/{self.target_npc_count} "
            f"pending={self.pending_front_wait_replacements} {detail}"
        )

    def replacement_spawn_offsets(self, management):
        hidden_range = management.get("replacement_spawn_offset_m", [80.0, 170.0])
        step = max(1.0, float(management.get("replacement_spawn_step_m", 10.0)))
        if not isinstance(hidden_range, (list, tuple)) or len(hidden_range) != 2:
            value = float(hidden_range)
            return [value]

        lo = float(min(hidden_range[0], hidden_range[1]))
        hi = float(max(hidden_range[0], hidden_range[1]))
        offsets = []
        current = lo
        while current <= hi + 1e-6:
            offsets.append(current)
            current += step
        if hi not in offsets:
            offsets.append(hi)
        self.rng.shuffle(offsets)
        return offsets

    def despawn_old_npcs(self, ego_s, ego_state=None):
        management = self.cfg.get("spawn_management", {})
        despawn_behind = float(management.get("despawn_distance_behind_m", 210.0))
        despawn_ahead = float(
            management.get(
                "despawn_distance_ahead_m",
                390.0,
            )
        )
        keep_radius = float(management.get("despawn_keep_radius_m", 80.0))
        keep_route_distance = float(
            management.get("despawn_keep_route_distance_m", 90.0)
        )
        ego_xy = self.ego_xy_from_state(ego_state)
        ego_pose = self.ego_pose_from_state(ego_state)
        visibility_cfg = self.cfg.get("camera_visibility", {})
        camera_protection_enabled = bool(visibility_cfg.get("enabled", False)) and ego_pose is not None
        spawn_grace_sec = float(management.get("spawn_grace_sec", 3.0))
        visibility_grace_sec = float(management.get("visibility_grace_sec", 3.0))
        outside_confirm_frames = int(management.get("outside_confirm_frames", 10))
        protect_waiting_npcs = bool(management.get("protect_waiting_npcs", True))
        waiting_despawn_behind = float(management.get("waiting_despawn_behind_m", 30.0))
        stuck_cleanup_enabled = bool(management.get("stuck_npc_cleanup_enabled", True))
        stuck_grace_sec = float(management.get("stuck_npc_grace_sec", 6.0))
        stuck_duration_sec = float(management.get("stuck_npc_duration_sec", 3.0))
        state_miss_remove_count = int(management.get("npc_state_miss_remove_count", 20))
        now = time.time()

        keep = []
        despawned_count = 0
        for npc in self.live_npcs:
            rel_s = npc.route_s - ego_s
            xy_distance = None
            age = now - npc.spawn_time
            if spawn_grace_sec > 0.0 and age < spawn_grace_sec:
                keep.append(npc)
                continue
            if npc.state_miss_count > 0 and (
                state_miss_remove_count <= 0
                or npc.state_miss_count < state_miss_remove_count
            ):
                keep.append(npc)
                continue
            stuck_protected = False
            if stuck_cleanup_enabled:
                if keep_route_distance > 0.0 and abs(rel_s) <= keep_route_distance:
                    stuck_protected = True
                if ego_xy is not None and keep_radius > 0.0:
                    xy_distance = dist_xy(npc.x, npc.y, ego_xy[0], ego_xy[1])
                    if xy_distance <= keep_radius:
                        stuck_protected = True
                if camera_protection_enabled:
                    visible_for_stuck = self.is_point_in_camera_visibility(
                        ego_pose,
                        npc.x,
                        npc.y,
                    )
                    if visible_for_stuck:
                        stuck_protected = True
            if (
                stuck_cleanup_enabled
                and not npc.waiting
                and not npc.stopped
                and age >= stuck_grace_sec
                and not stuck_protected
                and (
                    (
                        npc.low_motion_since is not None
                        and now - npc.low_motion_since >= stuck_duration_sec
                    )
                    or (
                        npc.low_progress_since is not None
                        and now - npc.low_progress_since >= stuck_duration_sec
                    )
                )
            ):
                motion_stuck_for = 0.0 if npc.low_motion_since is None else now - npc.low_motion_since
                progress_stuck_for = 0.0 if npc.low_progress_since is None else now - npc.low_progress_since
                print(
                    f"[NPC] stuck despawn decision label={npc.label} "
                    f"speed={npc.estimated_speed_mps:.2f}mps "
                    f"motion_stuck_for={motion_stuck_for:.1f}s "
                    f"progress_stuck_for={progress_stuck_for:.1f}s "
                    f"route_s={npc.route_s:.1f} ego_s={ego_s:.1f}"
                )
                self.destroy_npc(npc, reason="stuck")
                despawned_count += 1
                continue
            if protect_waiting_npcs and npc.waiting:
                if rel_s >= -waiting_despawn_behind:
                    keep.append(npc)
                    continue
                if camera_protection_enabled:
                    visible = self.update_camera_visibility_state(npc, ego_pose, now)
                    recently_visible = (
                        npc.last_camera_visible_time > 0.0
                        and now - npc.last_camera_visible_time <= visibility_grace_sec
                    )
                    if visible or recently_visible:
                        keep.append(npc)
                        continue
                print(
                    f"[NPC] waiting despawn decision label={npc.label} "
                    f"rel_s={rel_s:.1f}m route_s={npc.route_s:.1f} ego_s={ego_s:.1f}"
                )
                self.destroy_npc(npc, reason="waiting_behind")
                despawned_count += 1
                continue
            if camera_protection_enabled:
                visible = self.update_camera_visibility_state(npc, ego_pose, now)
                recently_visible = (
                    npc.last_camera_visible_time > 0.0
                    and now - npc.last_camera_visible_time <= visibility_grace_sec
                )
                if visible or recently_visible or npc.outside_camera_frames < outside_confirm_frames:
                    keep.append(npc)
                    continue
                if keep_route_distance > 0.0 and abs(rel_s) <= keep_route_distance:
                    keep.append(npc)
                    continue
                if ego_xy is not None and keep_radius > 0.0:
                    xy_distance = dist_xy(npc.x, npc.y, ego_xy[0], ego_xy[1])
                    if xy_distance <= keep_radius:
                        keep.append(npc)
                        continue
                outside_limits, forward_m, left_m = self.is_outside_camera_delete_limits(
                    npc,
                    ego_pose,
                    management,
                )
                if outside_limits:
                    print(
                        f"[NPC] camera despawn decision label={npc.label} "
                        f"forward={forward_m:.1f}m left={left_m:.1f}m "
                        f"outside_frames={npc.outside_camera_frames}"
                    )
                    self.destroy_npc(npc, reason="camera")
                    despawned_count += 1
                    continue
                keep.append(npc)
                continue
            if ego_xy is not None and keep_radius > 0.0:
                xy_distance = dist_xy(npc.x, npc.y, ego_xy[0], ego_xy[1])
                if xy_distance <= keep_radius:
                    keep.append(npc)
                    continue
            if keep_route_distance > 0.0 and abs(rel_s) <= keep_route_distance:
                keep.append(npc)
                continue
            if rel_s < -despawn_behind or rel_s > despawn_ahead:
                xy_msg = "unknown" if xy_distance is None else f"{xy_distance:.1f}m"
                print(
                    f"[NPC] despawn decision label={npc.label} "
                    f"rel_s={rel_s:.1f}m xy={xy_msg} "
                    f"route_s={npc.route_s:.1f} ego_s={ego_s:.1f}"
                )
                self.destroy_npc(npc, reason="distance")
                despawned_count += 1
            else:
                keep.append(npc)
        self.live_npcs = keep
        return despawned_count

    def activate_waiting_npcs(self, ego_s, ego_state=None):
        ego_xy = self.ego_xy_from_state(ego_state)
        failed = []

        for npc in self.live_npcs:
            if not npc.waiting:
                continue
            if npc.activate_distance_m is None:
                npc.activate_distance_m = self.sample_activate_distance(
                    opposite=bool(npc.opposite)
                )
            npc_activate_distance = float(npc.activate_distance_m)
            xy_distance = None
            if ego_xy is not None:
                xy_distance = dist_xy(npc.x, npc.y, ego_xy[0], ego_xy[1])
            if xy_distance is None or xy_distance > npc_activate_distance:
                continue

            npc.waiting = False
            npc.activated = True
            npc.stopped = False
            npc.ramping = bool(
                self.cfg.get("spawn_management", {}).get("speed_ramp_enabled", True)
            )
            npc.current_speed_kmh = 0.0 if npc.ramping else npc.speed_kmh
            npc.last_speed_update_time = 0.0
            print(
                f"[NPC] activate label={npc.label} "
                f"xy_distance={xy_distance:.1f}m "
                f"threshold={npc_activate_distance:.1f}m "
                f"opposite={npc.opposite} "
                f"start_speed={npc.current_speed_kmh:.1f} "
                f"speed={npc.speed_kmh:.1f}"
            )
            if not self.configure_spawned_vehicle(npc):
                failed.append(npc)

        if failed:
            for npc in failed:
                self.destroy_npc(npc, reason="configure_failed")
            failed_ids = {id(npc) for npc in failed}
            self.live_npcs = [npc for npc in self.live_npcs if id(npc) not in failed_ids]

    def update_speed_ramps(self):
        management = self.cfg.get("spawn_management", {})
        if not management.get("speed_ramp_enabled", True):
            return

        step = float(management.get("speed_ramp_step_kmh", 4.0))
        interval = float(management.get("speed_ramp_interval_sec", 0.5))
        now = time.time()

        for npc in self.live_npcs:
            if not npc.ramping or npc.waiting or npc.stopped:
                continue
            if interval > 0.0 and now - npc.last_speed_update_time < interval:
                continue

            npc.last_speed_update_time = now
            next_speed = min(npc.speed_kmh, npc.current_speed_kmh + step)
            npc.current_speed_kmh = next_speed
            self.sim_bridge.set_vehicle_speed_limit(npc.vehicle, next_speed, enabled=True)
            npc.last_speed_hold_time = now
            if next_speed >= npc.speed_kmh:
                npc.ramping = False

    def enforce_waiting_vehicle_stops(self):
        management = self.cfg.get("spawn_management", {})
        interval = float(management.get("waiting_stop_refresh_interval_sec", 0.5))
        if interval <= 0.0:
            return

        now = time.time()
        for npc in self.live_npcs:
            if not npc.waiting and not npc.stopped:
                continue
            if now - npc.last_wait_stop_time < interval:
                continue
            self.sim_bridge.set_vehicle_speed_limit(
                npc.vehicle,
                0.0,
                enabled=True,
                quiet=True,
            )
            self.sim_bridge.stop_vehicle(npc.vehicle, quiet=True)
            npc.current_speed_kmh = 0.0
            npc.last_wait_stop_time = now

    def enforce_active_vehicle_speeds(self):
        management = self.cfg.get("spawn_management", {})
        interval = float(management.get("active_speed_hold_interval_sec", 1.0))
        if interval <= 0.0:
            return

        now = time.time()
        for npc in self.live_npcs:
            if npc.waiting or npc.stopped or npc.ramping:
                continue
            if now - npc.last_speed_hold_time < interval:
                continue
            self.sim_bridge.set_vehicle_speed_limit(
                npc.vehicle,
                npc.speed_kmh,
                enabled=True,
                quiet=True,
            )
            npc.current_speed_kmh = npc.speed_kmh
            npc.last_speed_hold_time = now

    def enforce_active_vehicle_speed_guard(self):
        management = self.cfg.get("spawn_management", {})
        if not bool(management.get("speed_guard_enabled", True)):
            return

        max_kmh = float(management.get("speed_guard_max_kmh", 16.0))
        interval = float(management.get("speed_guard_correction_interval_sec", 1.0))
        now = time.time()
        for npc in self.live_npcs:
            if npc.waiting or npc.stopped:
                continue
            if not npc.has_first_state:
                continue
            measured_kmh = float(npc.estimated_speed_mps) * 3.6
            if measured_kmh <= max_kmh:
                continue
            if interval > 0.0 and now - npc.last_speed_guard_time < interval:
                continue

            target_kmh = min(float(npc.speed_kmh), max_kmh)
            self.sim_bridge.set_vehicle_speed_limit(
                npc.vehicle,
                target_kmh,
                enabled=True,
                quiet=True,
            )
            self.sim_bridge.set_vehicle_velocity(
                npc.vehicle,
                target_kmh / 3.6,
                quiet=True,
            )
            npc.current_speed_kmh = min(npc.current_speed_kmh, target_kmh)
            npc.last_speed_guard_time = now
            npc.last_speed_hold_time = now
            print(
                f"[NPC] speed guard label={npc.label} "
                f"measured={measured_kmh:.1f}kmh "
                f"target={target_kmh:.1f}kmh"
            )

    def try_spawn_one(
        self,
        ego_s,
        force_front_wait=False,
        ego_state=None,
        reject_visible_spawn=None,
        release_visible_spawn_immediately=True,
    ):
        attempts = int(self.cfg.get("max_spawn_attempts", 20))
        ego_pose = self.ego_pose_from_state(ego_state)
        visibility_cfg = self.cfg.get("camera_visibility", {})
        if reject_visible_spawn is None:
            reject_visible_spawn = bool(visibility_cfg.get("reject_spawn_if_visible", True))
        reject_visible_spawn = bool(
            visibility_cfg.get("enabled", False)
            and reject_visible_spawn
            and ego_pose is not None
        )
        management = self.cfg.get("spawn_management", {})
        visible_rejects = 0
        invalid_rejects = 0
        validator_rejects = 0
        pose_rejects = 0
        remaining_rejects = 0
        position_rejects = 0
        route_rejects = 0
        hidden_offsets = (
            self.replacement_spawn_offsets(management)
            if force_front_wait and reject_visible_spawn
            else []
        )
        for _ in range(attempts):
            if force_front_wait:
                sample = self.slot_sampler.sample_front_edge(self.road_group)
            else:
                sample = self.slot_sampler.sample(self.road_group)
            if sample is None:
                return False

            if hidden_offsets:
                sample = dict(sample)
                sample["slot"] = f"{sample['slot']}_hidden"
                sample["s_offset_m"] = hidden_offsets.pop(0)

            spawn_s = float(ego_s) + float(sample["s_offset_m"])
            if force_front_wait:
                margin = float(self.cfg.get("spawn_margin_from_route_end_m", 8.0))
                spawn_s = min(spawn_s, self.route_helper.route_length_m - margin)
            if spawn_s < 0.0 or spawn_s > self.route_helper.route_length_m:
                invalid_rejects += 1
                continue
            if force_front_wait and reject_visible_spawn:
                min_remaining = float(
                    management.get("replacement_min_remaining_route_m", 80.0)
                )
                if self.route_helper.route_length_m - spawn_s < min_remaining:
                    remaining_rejects += 1
                    continue

            opposite = bool(sample.get("opposite", False))
            adjacent = sample.get("adjacent")
            opposite_link = None
            adjacent_link = None
            route_link = self.route_helper.link_at_s(spawn_s)
            npc_route_links = self.build_route_suffix_at_s(spawn_s)

            if opposite:
                pose = self.sample_opposite_pose(spawn_s)
                if pose is None:
                    pose_rejects += 1
                    continue
                x, y, z, yaw, opposite_link = pose
                npc_route_links = self.build_opposite_route_from_link(opposite_link)
            elif adjacent:
                pose = self.sample_adjacent_pose(spawn_s, str(adjacent))
                if pose is None:
                    pose_rejects += 1
                    continue
                x, y, z, yaw, adjacent_link = pose
                npc_route_links = self.build_adjacent_route_from_link(adjacent_link)
            else:
                x, y, z, yaw = self.route_helper.pose_at(
                    spawn_s,
                    sample.get("lateral_offset_m", 0.0),
                )

            route_signature = tuple(npc_route_links or [])
            if route_signature in self.failed_route_signatures:
                route_rejects += 1
                continue

            candidate = {
                "route_s": spawn_s,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "slot": sample["slot"],
                "allow_close_spawn": bool(sample.get("allow_close_spawn", False)),
            }
            visible_at_spawn = False
            allow_close_spawn = bool(sample.get("allow_close_spawn", False))
            if ego_pose is not None:
                visible_at_spawn = self.is_point_in_camera_visibility(
                    ego_pose,
                    x,
                    y,
                    margin_deg=0.0,
                    margin_m=0.0,
                )
                min_xy_from_ego = float(
                    management.get(
                        "min_xy_distance_from_ego_m",
                        management.get("min_distance_from_ego_m", 18.0),
                    )
                )
                ego_xy_distance = dist_xy(x, y, ego_pose[0], ego_pose[1])
                if (
                    not allow_close_spawn
                    and min_xy_from_ego > 0.0
                    and ego_xy_distance < min_xy_from_ego
                ):
                    position_rejects += 1
                    continue
            if force_front_wait and ego_pose is not None:
                forward, left = self.world_to_ego_xy(x, y, ego_pose)
                min_forward = float(management.get("replacement_min_forward_m", 25.0))
                max_forward = float(management.get("replacement_max_forward_m", 180.0))
                max_abs_left = float(management.get("replacement_max_abs_left_m", 90.0))
                if (
                    (not allow_close_spawn and forward < min_forward)
                    or forward > max_forward
                    or abs(left) > max_abs_left
                ):
                    position_rejects += 1
                    continue
            allow_visible_spawn = bool(sample.get("allow_visible_spawn", False))
            if reject_visible_spawn and not allow_visible_spawn:
                visible_with_spawn_margin = self.is_point_in_camera_visibility(
                    ego_pose,
                    x,
                    y,
                    margin_deg=float(visibility_cfg.get("spawn_margin_deg", 15.0)),
                    margin_m=float(visibility_cfg.get("spawn_margin_m", 25.0)),
                )
                if visible_with_spawn_margin:
                    visible_rejects += 1
                    continue
            if not self.validator.validate(
                candidate,
                ego_s,
                self.route_helper.route_length_m,
                self.live_npcs,
            ):
                validator_rejects += 1
                continue

            model_name = self.model_selector.choose()
            if not model_name:
                return False

            speed_kmh, stopped = self.sample_speed(opposite=opposite)
            wait_until_close = bool(
                self.cfg.get("spawn_management", {}).get("wait_until_ego_close", False)
            )
            waiting = bool((force_front_wait or wait_until_close) and spawn_s >= ego_s and not stopped)
            if (
                waiting
                and visible_at_spawn
                and not reject_visible_spawn
                and release_visible_spawn_immediately
            ):
                waiting = False
            activate_distance_m = (
                self.sample_activate_distance(opposite=opposite)
                if waiting
                else None
            )
            initial_velocity_kmh = 0.0 if waiting or stopped else speed_kmh
            initial_velocity_mps = initial_velocity_kmh / 3.6
            label = f"AIM_NPC_{self.next_id:04d}"
            self.next_id += 1
            transform = self.sim_bridge.make_transform(x, y, z, yaw)

            try:
                vehicle = self.sim_bridge.spawn_vehicle(
                    transform=transform,
                    model_name=model_name,
                    label=label,
                    velocity=initial_velocity_mps,
                    multi_ego=False,
                )
            except Exception as exc:
                print(f"[NPC] spawn failed label={label}: {exc}")
                return False

            if vehicle is None:
                return False

            npc = ManagedNpcVehicle(
                label=label,
                vehicle=vehicle,
                route_s=spawn_s,
                x=x,
                y=y,
                model_name=model_name,
                slot=sample["slot"],
                speed_kmh=speed_kmh,
                stopped=stopped,
                opposite=opposite,
                route_links=npc_route_links,
                waiting=waiting,
                activate_distance_m=activate_distance_m,
            )
            if not self.configure_spawned_vehicle(npc):
                if route_signature:
                    self.failed_route_signatures.add(route_signature)
                self.destroy_npc(npc, reason="configure_failed")
                route_rejects += 1
                continue
            self.live_npcs.append(npc)
            spawn_pos_msg = ""
            if ego_pose is not None:
                forward, left = self.world_to_ego_xy(x, y, ego_pose)
                spawn_pos_msg = f" f={forward:.1f} left={left:.1f} visible={int(visible_at_spawn)}"
            print(
                f"[NPC] spawn label={label} model={model_name} "
                f"slot={sample['slot']} s={spawn_s:.1f} "
                f"opposite={opposite} opposite_link={opposite_link} "
                f"adjacent={adjacent or '-'} adjacent_link={adjacent_link} "
                f"route_links={len(npc_route_links)} "
                f"lateral={sample.get('lateral_offset_m', 0.0):.1f} "
                f"speed={speed_kmh:.1f} stopped={stopped} waiting={waiting} "
                f"activate_dist={activate_distance_m if activate_distance_m is not None else -1.0:.1f}"
                f"{spawn_pos_msg}"
            )
            return True

        if reject_visible_spawn or force_front_wait:
            self.log_spawn_deferred(
                visible_rejects=visible_rejects,
                invalid_rejects=invalid_rejects,
                validator_rejects=validator_rejects,
                pose_rejects=pose_rejects,
                position_rejects=position_rejects,
                route_rejects=route_rejects,
                remaining_rejects=remaining_rejects,
                attempts=attempts,
            )
        return False

    def log_spawn_deferred(
        self,
        visible_rejects,
        invalid_rejects,
        validator_rejects,
        pose_rejects,
        position_rejects,
        route_rejects,
        remaining_rejects,
        attempts,
    ):
        management = self.cfg.get("spawn_management", {})
        interval = float(management.get("spawn_deferred_log_interval_sec", 5.0))
        signature = (
            int(visible_rejects),
            int(invalid_rejects),
            int(validator_rejects),
            int(pose_rejects),
            int(position_rejects),
            int(route_rejects),
            int(remaining_rejects),
            int(attempts),
        )
        now = time.time()
        if (
            interval > 0.0
            and signature == self.last_spawn_deferred_signature
            and now - self.last_spawn_deferred_log_time < interval
        ):
            return

        self.last_spawn_deferred_signature = signature
        self.last_spawn_deferred_log_time = now
        print(
            f"[NPC] spawn deferred: visible_rejects={visible_rejects} "
            f"invalid_rejects={invalid_rejects} "
            f"validator_rejects={validator_rejects} "
            f"pose_rejects={pose_rejects} "
            f"position_rejects={position_rejects} "
            f"route_rejects={route_rejects} "
            f"remaining_rejects={remaining_rejects} attempts={attempts}"
        )

    def adjacent_link_for_route_link(self, route_link, side):
        if self.map_loader is None:
            return None
        if not route_link or route_link not in self.map_loader.link_set:
            return None

        link = self.map_loader.link_set[route_link]
        side = str(side).lower()
        if side == "left":
            candidates = [
                link.get("left_lane_change_dst_link_idx"),
                link.get("lane_ch_link_left"),
            ]
        elif side == "right":
            candidates = [
                link.get("right_lane_change_dst_link_idx"),
                link.get("lane_ch_link_right"),
            ]
        else:
            return None

        for candidate in candidates:
            if isinstance(candidate, dict):
                candidate = (
                    candidate.get("idx")
                    or candidate.get("id")
                    or candidate.get("link_id")
                    or candidate.get("value")
                )
            if candidate and candidate in self.map_loader.link_set:
                return candidate
        return None

    def sample_adjacent_pose(self, spawn_s, side):
        if self.map_loader is None:
            return None

        route_link = self.route_helper.link_at_s(spawn_s)
        adjacent_link = self.adjacent_link_for_route_link(route_link, side)
        if not adjacent_link:
            return None

        center_x, center_y, _, _ = self.route_helper.pose_at(spawn_s, 0.0)
        points = self.map_loader.get_link_points(adjacent_link)
        try:
            link_s = project_distance_on_polyline(points, center_x, center_y)
            x, y, z, yaw = interpolate_on_polyline(points, link_s)
            return x, y, z, yaw, adjacent_link
        except Exception:
            return None

    def sample_opposite_pose(self, spawn_s):
        if self.map_loader is None:
            return None

        road_cfg = self.cfg.get("road_groups", {}).get(self.road_group, {})
        if not road_cfg.get("has_opposite_lane", True):
            return None

        route_link = self.route_helper.link_at_s(spawn_s)
        opposite_map = road_cfg.get("opposite_links", {}) or {}
        opposite_links = opposite_map.get(route_link, [])
        if isinstance(opposite_links, str):
            opposite_links = [opposite_links]
        opposite_links = [link_id for link_id in opposite_links if link_id in self.map_loader.link_set]
        if not opposite_links and self.cfg.get("auto_find_opposite_links", True):
            opposite_links = self.find_opposite_links(route_link)
        if not opposite_links:
            return None

        center_x, center_y, _, _ = self.route_helper.pose_at(spawn_s, 0.0)
        link_id = self.rng.choice(opposite_links)
        points = self.map_loader.get_link_points(link_id)
        try:
            link_s = project_distance_on_polyline(points, center_x, center_y)
            x, y, z, yaw = interpolate_on_polyline(points, link_s)
            return x, y, z, yaw, link_id
        except Exception:
            return None

    def find_opposite_links(self, route_link):
        if not route_link or route_link not in self.map_loader.link_set:
            return []
        if route_link in self.opposite_link_cache:
            return list(self.opposite_link_cache[route_link])

        target_points = self.map_loader.get_link_points(route_link)
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
            overlap = max(0.0, min(target_range[1], other_range[1]) - max(target_range[0], other_range[0]))
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
        if result:
            print(f"[NPC] opposite links auto {route_link}: {result}")
        return list(result)

    def ensure_road_graph(self):
        if self.map_loader is None:
            return None

        if self.road_graph is None:
            from utils.road_link_graph import RoadLinkGraph

            self.road_graph = RoadLinkGraph(self.map_loader)
        return self.road_graph

    def build_forward_route_from_link(
        self,
        start_link,
        min_length_m=120.0,
        max_links=8,
        avoid_links=None,
        avoid_strict=False,
    ):
        if self.map_loader is None:
            return []
        if not start_link or start_link not in self.map_loader.link_set:
            return []

        road_graph = self.ensure_road_graph()
        if road_graph is None:
            return []

        route = [start_link]
        seen = {start_link}
        avoid = set(avoid_links or [])
        length_m = road_graph.link_length(start_link)
        current = start_link

        while length_m < min_length_m and len(route) < max_links:
            next_links = [
                link_id
                for link_id in road_graph.adj.get(current, [])
                if link_id not in seen and "-" not in link_id
            ]
            if not next_links:
                break
            preferred_links = [link_id for link_id in next_links if link_id not in avoid]
            if preferred_links:
                next_links = preferred_links
            elif avoid_strict:
                break
            current = next_links[0]
            route.append(current)
            seen.add(current)
            length_m += road_graph.link_length(current)

        return route

    def build_opposite_route_from_link(self, opposite_link):
        management = self.cfg.get("spawn_management", {})
        min_length_m = float(management.get("opposite_route_min_length_m", 320.0))
        max_links = int(management.get("opposite_route_max_links", 18))
        cache_key = (opposite_link, min_length_m, max_links)
        cached = self.opposite_route_cache.get(cache_key)
        if cached is not None:
            return list(cached)

        route = self.build_forward_route_from_link(
            opposite_link,
            min_length_m=min_length_m,
            max_links=max_links,
        )
        if not route:
            return []

        road_graph = self.ensure_road_graph()
        length_m = road_graph.path_length(route) if road_graph is not None else 0.0
        if length_m < min_length_m:
            print(
                f"[NPC] opposite route short "
                f"opposite_start={opposite_link} "
                f"length={length_m:.1f}/{min_length_m:.1f}m links={len(route)}"
            )
        else:
            print(
                f"[NPC] opposite route extended "
                f"opposite_start={opposite_link} "
                f"length={length_m:.1f}m links={len(route)}"
            )
        self.opposite_route_cache[cache_key] = list(route)
        return route

    def build_adjacent_route_from_link(self, adjacent_link):
        management = self.cfg.get("spawn_management", {})
        min_length_m = float(management.get("adjacent_route_min_length_m", 260.0))
        max_links = int(management.get("adjacent_route_max_links", 18))
        allow_unavoidable_overlap = bool(
            management.get("adjacent_route_allow_unavoidable_overlap", True)
        )
        cache_key = (
            adjacent_link,
            min_length_m,
            max_links,
            allow_unavoidable_overlap,
            tuple(self.route_links),
        )
        cached = self.adjacent_route_cache.get(cache_key)
        if cached is not None:
            return list(cached)

        route = self.build_forward_route_from_link(
            adjacent_link,
            min_length_m=min_length_m,
            max_links=max_links,
            avoid_links=set(self.route_links),
            avoid_strict=not allow_unavoidable_overlap,
        )
        if not route:
            return []

        road_graph = self.ensure_road_graph()
        length_m = road_graph.path_length(route) if road_graph is not None else 0.0
        overlapping = [link_id for link_id in route if link_id in self.route_links]
        if overlapping:
            print(
                f"[NPC] adjacent route separate with unavoidable overlap "
                f"adjacent_start={adjacent_link} "
                f"length={length_m:.1f}m links={len(route)} overlaps={overlapping}"
            )
        elif length_m < min_length_m:
            print(
                f"[NPC] adjacent route short "
                f"adjacent_start={adjacent_link} "
                f"length={length_m:.1f}/{min_length_m:.1f}m links={len(route)}"
            )
        else:
            print(
                f"[NPC] adjacent route separate "
                f"adjacent_start={adjacent_link} "
                f"length={length_m:.1f}m links={len(route)}"
            )
        self.adjacent_route_cache[cache_key] = list(route)
        return route

    def extend_same_direction_route(self, route_links):
        route = list(route_links or [])
        if not route or self.map_loader is None:
            return route
        management = self.cfg.get("spawn_management", {})
        min_extra_m = float(management.get("same_direction_route_extension_m", 320.0))
        max_extra_links = int(management.get("same_direction_route_extension_max_links", 18))
        cache_key = (tuple(route), min_extra_m, max_extra_links)
        cached = self.same_direction_route_extension_cache.get(cache_key)
        if cached is not None:
            return list(cached)

        if min_extra_m <= 0.0 or max_extra_links <= 0:
            self.same_direction_route_extension_cache[cache_key] = list(route)
            return route

        road_graph = self.ensure_road_graph()
        if road_graph is None:
            self.same_direction_route_extension_cache[cache_key] = list(route)
            return route

        base_end = route[-1]
        current = route[-1]
        original_links = set(route)
        extension_seen = set()
        extra_m = 0.0
        added = 0

        while extra_m < min_extra_m and added < max_extra_links:
            next_links = [
                link_id
                for link_id in road_graph.adj.get(current, [])
                if (
                    link_id not in original_links
                    and link_id not in extension_seen
                    and "-" not in link_id
                )
            ]
            if not next_links:
                break

            current = next_links[0]
            route.append(current)
            extension_seen.add(current)
            extra_m += road_graph.link_length(current)
            added += 1

        if added > 0 and cache_key not in self.logged_same_direction_extensions:
            self.logged_same_direction_extensions.add(cache_key)
            if extra_m < min_extra_m:
                print(
                    f"[NPC] same-direction route short "
                    f"base_end={base_end} added={added} "
                    f"extra={extra_m:.1f}/{min_extra_m:.1f}m"
                )
            else:
                print(
                    f"[NPC] same-direction route extended "
                    f"base_end={base_end} added={added} extra={extra_m:.1f}m"
                )
        self.same_direction_route_extension_cache[cache_key] = list(route)
        return route

    def build_route_suffix_from_link(self, start_link):
        if start_link in self.route_links:
            suffix = list(self.route_links[self.route_links.index(start_link) :])
            return self.extend_same_direction_route(suffix)
        fallback = self.build_forward_route_from_link(start_link)
        return fallback or self.extend_same_direction_route(self.route_links)

    def trim_route_before_duplicate(self, route_links):
        route = list(route_links or [])
        seen = set()
        out = []
        duplicate_link = None
        for link_id in route:
            if link_id in seen:
                duplicate_link = link_id
                break
            seen.add(link_id)
            out.append(link_id)

        if duplicate_link is not None and out:
            key = (tuple(route), duplicate_link)
            if key not in self.logged_duplicate_route_trims:
                self.logged_duplicate_route_trims.add(key)
                print(
                    f"[NPC] route duplicate trimmed "
                    f"duplicate={duplicate_link} before={len(route)} after={len(out)}"
                )
            return out
        return route

    def build_route_suffix_at_s(self, route_s):
        idx = self.route_helper.link_index_at_s(route_s)
        if idx is not None and 0 <= idx < len(self.route_links):
            suffix = list(self.route_links[idx:])
            return self.trim_route_before_duplicate(self.extend_same_direction_route(suffix))

        route_link = self.route_helper.link_at_s(route_s)
        route = self.build_route_suffix_from_link(route_link)
        return self.trim_route_before_duplicate(route)

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
        values = [(point[0] - origin[0]) * ux + (point[1] - origin[1]) * uy for point in points]
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

    def sample_speed(self, opposite=False):
        speed_cfg = self.cfg.get("speed", {})
        if "fixed_kmh" in speed_cfg:
            return float(speed_cfg["fixed_kmh"]), False

        key = "opposite_direction_kmh" if opposite else "same_direction_kmh"
        speed_range = speed_cfg.get(key, [20.0, 50.0])
        if not isinstance(speed_range, (list, tuple)) or len(speed_range) != 2:
            speed_range = [float(speed_range), float(speed_range)]

        stopped_probability = float(speed_cfg.get("stopped_probability", 0.1))
        stopped = self.rng.random() < stopped_probability
        if stopped:
            return 0.0, True

        lo = float(min(speed_range[0], speed_range[1]))
        hi = float(max(speed_range[0], speed_range[1]))
        return self.rng.uniform(lo, hi), False

    def configure_spawned_vehicle(self, npc):
        try:
            if npc.route_links:
                route_ok = self.sim_bridge.set_vehicle_route(
                    npc.vehicle,
                    npc.route_links,
                    decision_range=30.0,
                    label=npc.label,
                )
                if not route_ok:
                    print(
                        f"[NPC] route config failed label={npc.label} "
                        f"slot={npc.slot} opposite={npc.opposite} "
                        f"links={npc.route_links}"
                    )
                    return False

            if npc.stopped or npc.waiting:
                self.sim_bridge.set_vehicle_speed_limit(
                    npc.vehicle,
                    0.0,
                    enabled=True,
                    quiet=True,
                )
                self.sim_bridge.stop_vehicle(npc.vehicle)
                npc.last_wait_stop_time = time.time()
                return True

            self.sim_bridge.resume_vehicle_ai(npc.vehicle)
            target_speed_kmh = npc.current_speed_kmh if npc.ramping else npc.speed_kmh
            self.sim_bridge.set_vehicle_speed_limit(
                npc.vehicle,
                target_speed_kmh,
                enabled=True,
            )
            npc.last_speed_hold_time = time.time()
            return True
        except Exception as exc:
            print(f"[NPC] configure warning label={npc.label}: {exc}")
            return False
