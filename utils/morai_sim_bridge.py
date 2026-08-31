import os
import sys
import time


class MoraiSimBridge:
    def __init__(self, global_cfg: dict):
        self.global_cfg = global_cfg

        grpc_cfg = global_cfg["grpc"]
        path_cfg = global_cfg["paths"]

        self.host = grpc_cfg.get("host", "127.0.0.1")
        self.port = int(grpc_cfg.get("port", 7789))
        self.client_key = grpc_cfg.get("client_key", "aim_scenario_runner")
        self.grpc_src = path_cfg["grpc_src"]

        self._add_grpc_paths()
        self._disable_third_party_mgeo_fetch()

        from api.morai_sim_client import MoraiSimClient

        self.client = MoraiSimClient(self.client_key)
        self.world = None
        self._skip_next_place_ego_transform = False

    def _add_grpc_paths(self):
        api_path = os.path.join(self.grpc_src, "api")
        proto_path = os.path.join(self.grpc_src, "proto")

        for p in [self.grpc_src, api_path, proto_path]:
            if p not in sys.path:
                sys.path.append(p)

    def _disable_third_party_mgeo_fetch(self):
        """
        third_party SimulationWorld는 생성 시 MORAI 서버의 GetMGeo를 호출한다.
        우리는 local mgeo_root의 JSON을 직접 쓰므로 서버 MGeo 다운로드를 생략한다.
        """
        try:
            from api.map import Map
        except Exception as e:
            print(f"[MORAI] skip disabling third_party MGeo fetch: {e}")
            return

        if getattr(Map, "_aim_scenario_runner_mgeo_fetch_disabled", False):
            return

        def _skip_get_mgeo_data(self, map_name):
            self.mgeo_data = {}
            print(f"[MORAI] skip GetMGeo for {map_name}; using local MGeo JSON")

        Map.get_mgeo_data = _skip_get_mgeo_data
        Map._aim_scenario_runner_mgeo_fetch_disabled = True

    def connect(self):
        self.client.connect(self.host, self.port)

        if not self.client.is_connected():
            raise RuntimeError("Failed to connect MORAI gRPC server")

        try:
            import grpc

            channel = self.client._sim_adapter._channel
            grpc.channel_ready_future(channel).result(timeout=3.0)
        except Exception as exc:
            self.client.disconnect()
            raise RuntimeError(
                f"Failed to reach MORAI gRPC server at {self.host}:{self.port}. "
                "Check MORAI is running, gRPC is enabled, and runtime.yaml grpc.host/port match this PC."
            ) from exc

        print(f"[gRPC] connected to {self.host}:{self.port}")

    def make_transform(self, x, y, z, yaw_deg):
        from proto.morai.common.type_pb2 import Transform

        tf = Transform()
        tf.location.x = float(x)
        tf.location.y = float(y)
        tf.location.z = float(z)

        tf.rotation.x = 0.0
        tf.rotation.y = 0.0
        tf.rotation.z = float(yaw_deg)
        return tf

    def start_world(self, ego_transform):
        from proto.morai.simulation.start_param_pb2 import MapAndVehicle
        from proto.morai.actor.actor_set_pb2 import EgoCruiseControl
        from proto.morai.actor.actor_enum_pb2 import EGO_CRUISE_TYPE_LINK
        from proto.morai.simulation.sync_mode_pb2 import SyncMode
        from proto.morai.simulation.simulation_enum_pb2 import SYNC_MODE_TYPE_UNSPECIFIED

        morai_cfg = self.global_cfg["morai"]

        map_and_vehicle = MapAndVehicle()
        map_and_vehicle.map_name = morai_cfg["map_name"]
        map_and_vehicle.ego_vehicle_model = morai_cfg["ego_vehicle_model"]

        cruise = EgoCruiseControl()
        cruise.cruise_on = False
        cruise.cruise_type = EGO_CRUISE_TYPE_LINK
        cruise.link_speed_ratio = 0
        cruise.constant_velocity = 0

        sync_mode = SyncMode()
        sync_mode.type = SYNC_MODE_TYPE_UNSPECIFIED

        self.client.start_simulation(
            map_and_vehicle=map_and_vehicle,
            ego_transform=ego_transform,
            ego_cruise_setting=cruise,
            sync_mode=sync_mode,
        )

        self.world = self.client.get_simulation_world()
        if self.world is None:
            raise RuntimeError("Failed to start MORAI simulation world")

        self._skip_next_place_ego_transform = True
        print("[MORAI] world started")

    def reset_actors(self):
        if self.world is not None:
            self.world.destroy_all_actors()
            print("[MORAI] actors destroyed")

    def get_ego(self):
        if self.world is None:
            raise RuntimeError("world is None. Call start_world() first.")
        return self.world.get_ego()

    def set_ego_transform(self, transform):
        ego = self.get_ego()
        ok = ego.set_transform(transform)
        print(f"[Ego] set_transform: {ok}")
        return ok

    def set_ego_velocity(self, velocity=0.0):
        ego = self.get_ego()
        try:
            ok = ego.set_velocity(float(velocity))
        except Exception as e:
            print(f"[Ego] set_velocity {float(velocity):.1f} failed: {e}")
            return False
        print(f"[Ego] set_velocity {float(velocity):.1f}: {ok}")
        return ok

    def stop_ego_motion(self, settle_sec=0.0):
        """
        이전 제어/물리 속도를 끊어 다음 시나리오 시작점으로 이동하기 전 흔들림을 줄인다.
        """
        ok_any = False

        try:
            ok_any = bool(self.stop_ego_cruise()) or ok_any
        except Exception as e:
            print(f"[Ego] stop_cruise warning: {e}")

        try:
            ok_any = bool(self.stop_ego_control()) or ok_any
        except Exception as e:
            print(f"[Ego] stop_control warning: {e}")

        try:
            ok_any = bool(self.set_ego_velocity(0.0)) or ok_any
        except Exception as e:
            print(f"[Ego] set_velocity warning: {e}")

        if settle_sec and float(settle_sec) > 0.0:
            time.sleep(float(settle_sec))

        return ok_any

    def hold_ego_stopped(self, duration_sec=0.5, interval_sec=0.05):
        duration_sec = max(0.0, float(duration_sec))
        interval_sec = max(0.01, float(interval_sec))
        end_time = time.time() + duration_sec
        ok_any = False

        while True:
            try:
                ok_any = bool(self.stop_ego_control()) or ok_any
            except Exception as e:
                print(f"[Ego] hold stop_control warning: {e}")
            try:
                ok_any = bool(self.set_ego_velocity(0.0)) or ok_any
            except Exception as e:
                print(f"[Ego] hold set_velocity warning: {e}")

            if time.time() >= end_time:
                break
            time.sleep(interval_sec)

        print(f"[Ego] hold stopped duration={duration_sec:.2f}s ok={ok_any}")
        return ok_any

    def place_ego_stopped(self, transform, settle_sec=0.2):
        """
        Ego를 지정 transform에 둔 뒤 속도/제어 잔류값을 0으로 만든다.
        """
        if self._skip_next_place_ego_transform:
            self._skip_next_place_ego_transform = False
            transform_ok = True
            print("[Ego] set_transform skipped: already placed by start_world")
        else:
            transform_ok = self.set_ego_transform(transform)

        velocity_ok = self.set_ego_velocity(0.0)

        if settle_sec and float(settle_sec) > 0.0:
            time.sleep(float(settle_sec))

        return bool(transform_ok or velocity_ok)

    def set_ego_route(
        self,
        route_links,
        decision_range=30.0,
        route_waypoint_indices=None,
        waypoint_indices=None,
    ):
        """
        Ego vehicle route 설정.

        grpc_inha_univ의 Vehicle.set_vehicle_route()는 protobuf 환경에 따라
        param.links.append(...)에서 실패할 수 있어서 여기서 직접 add()로 구성한다.
        """
        from proto.morai.actor.actor_set_pb2 import VehicleRoute
        from proto.morai.common.enum_pb2 import STATUS_CODE_SUCCESS

        ego = self.get_ego()

        if route_waypoint_indices is None:
            route_waypoint_indices = waypoint_indices

        if route_waypoint_indices is None:
            route_waypoint_indices = {}

        param = VehicleRoute()
        param.actor_info.CopyFrom(ego.get_object_info())
        param.decision_range = float(decision_range)

        for i, link_id in enumerate(route_links):
            link_info = param.links.add()
            link_info.id.value = str(link_id)

            if isinstance(route_waypoint_indices, dict):
                link_info.waypoint_idx = int(route_waypoint_indices.get(link_id, 0))
            elif isinstance(route_waypoint_indices, (list, tuple)) and i < len(route_waypoint_indices):
                link_info.waypoint_idx = int(route_waypoint_indices[i])
            else:
                link_info.waypoint_idx = 0

        result = ego._sim_adapter.set_vehicle_route(param)
        ok = result is not None and result.status == STATUS_CODE_SUCCESS

        print(f"[Ego] set_vehicle_route: {ok}, links={list(route_links)}")
        return ok

    def set_vehicle_route(
        self,
        vehicle,
        route_links,
        decision_range=30.0,
        route_waypoint_indices=None,
        waypoint_indices=None,
        label="Vehicle",
    ):
        """
        일반 vehicle route 설정.

        third_party Vehicle.set_vehicle_route()가 protobuf append() 문제로 실패할 수 있어
        Ego와 동일하게 add()로 직접 구성한다.
        """
        from proto.morai.actor.actor_set_pb2 import VehicleRoute
        from proto.morai.common.enum_pb2 import STATUS_CODE_SUCCESS

        if vehicle is None:
            return False

        if route_waypoint_indices is None:
            route_waypoint_indices = waypoint_indices

        if route_waypoint_indices is None:
            route_waypoint_indices = {}

        param = VehicleRoute()
        param.actor_info.CopyFrom(vehicle.get_object_info())
        param.decision_range = float(decision_range)

        for i, link_id in enumerate(route_links):
            link_info = param.links.add()
            link_info.id.value = str(link_id)

            if isinstance(route_waypoint_indices, dict):
                link_info.waypoint_idx = int(route_waypoint_indices.get(link_id, 0))
            elif isinstance(route_waypoint_indices, (list, tuple)) and i < len(route_waypoint_indices):
                link_info.waypoint_idx = int(route_waypoint_indices[i])
            else:
                link_info.waypoint_idx = 0

        result = vehicle._sim_adapter.set_vehicle_route(param)
        ok = result is not None and result.status == STATUS_CODE_SUCCESS

        print(f"[{label}] set_vehicle_route: {ok}, links={list(route_links)}")
        return ok

    def set_ego_cruise(self, enable=True, link_speed_ratio=40, constant_velocity=20, cruise_type="link"):
        from proto.morai.actor.actor_enum_pb2 import EGO_CRUISE_TYPE_CONSTANT, EGO_CRUISE_TYPE_LINK

        cruise_type_value = EGO_CRUISE_TYPE_CONSTANT if str(cruise_type).lower() == "constant" else EGO_CRUISE_TYPE_LINK
        ego = self.get_ego()
        ok = ego.set_cruise_mode(
            enable=enable,
            cruise_type=cruise_type_value,
            link_speed_ratio=link_speed_ratio,
            constant_velocity=constant_velocity,
        )
        print(f"[Ego] set_cruise_mode: {ok}")
        return ok

    def stop_ego_cruise(self):
        return self.set_ego_cruise(enable=False, link_speed_ratio=0, constant_velocity=0)

    def set_ego_control_mode_cruise(self):
        """
        Ego control mode를 MORAI built-in cruise mode로 설정.
        protobuf 환경에 따라 VehicleControlMode.VEHICLE_CONTROL_CRUISE_MODE 접근이
        실패할 수 있어서 모듈 최상위 상수를 직접 사용한다.
        """
        from proto.morai.actor.actor_set_pb2 import VehicleControlModeParam
        from proto.morai.actor.actor_enum_pb2 import VEHICLE_CONTROL_CRUISE_MODE
        from proto.morai.common.enum_pb2 import STATUS_CODE_SUCCESS

        ego = self.get_ego()

        param = VehicleControlModeParam()
        param.actor_info.CopyFrom(ego.get_object_info())
        param.mode = VEHICLE_CONTROL_CRUISE_MODE

        result = ego._sim_adapter.set_vehicle_control_mode(param)
        ok = result is not None and result.status == STATUS_CODE_SUCCESS

        print(f"[Ego] set_control_mode CRUISE: {ok}")
        return ok

    def set_ego_control_mode_auto(self):
        """
        Ego control mode를 외부 알고리즘 제어 모드로 설정.
        """
        from proto.morai.actor.actor_set_pb2 import VehicleControlModeParam
        from proto.morai.actor.actor_enum_pb2 import VEHICLE_CONTROL_AUTO_MODE
        from proto.morai.common.enum_pb2 import STATUS_CODE_SUCCESS

        ego = self.get_ego()

        param = VehicleControlModeParam()
        param.actor_info.CopyFrom(ego.get_object_info())
        param.mode = VEHICLE_CONTROL_AUTO_MODE

        result = ego._sim_adapter.set_vehicle_control_mode(param)
        ok = result is not None and result.status == STATUS_CODE_SUCCESS

        print(f"[Ego] set_control_mode AUTO: {ok}")
        return ok

    def control_ego(self, steer, target_speed, brake=0.0, throttle=0.0):
        from proto.morai.actor.actor_enum_pb2 import LONG_CMD_TYPE_SPEED

        ego = self.get_ego()
        try:
            ok = ego.control(
                long_cmd_type=LONG_CMD_TYPE_SPEED,
                throttle=float(throttle),
                brake=float(brake),
                steer=float(steer),
                velocity=float(target_speed),
                acceleration=0.0,
                frame=0,
            )
        except Exception as exc:
            print(f"[Ego] control warning: {exc}")
            return False
        return ok

    def stop_ego_control(self):
        return self.control_ego(steer=0.0, target_speed=0.0, brake=1.0, throttle=0.0)

    def set_ego_gear_drive(self):
        from proto.morai.actor.actor_enum_pb2 import GEAR_MODE_D

        ego = self.get_ego()
        ok = ego.set_vehicle_gear(GEAR_MODE_D)
        print(f"[Ego] set_gear D: {ok}")
        return ok

    def spawn_vehicle(self, transform, model_name, label, velocity=0.0, multi_ego=True):
        vehicle = self.world.spawn_vehicle(
            transform=transform,
            model_name=model_name,
            label=label,
            velocity=velocity,
            multi_ego=multi_ego,
        )
        print(f"[Spawn] vehicle {label}: {vehicle is not None}")
        return vehicle

    def spawn_pedestrian(
        self,
        transform,
        model_name,
        label,
        velocity=0.0,
        active_dist=30.0,
        move_dist=30.0,
        start_action=False,
    ):
        try:
            pedestrian = self.world.spawn_pedestrian(
                transform=transform,
                model_name=model_name,
                label=label,
                velocity=float(velocity),
                active_dist=float(active_dist),
                move_dist=float(move_dist),
                start_action=bool(start_action),
            )
        except Exception as exc:
            print(f"[Spawn] pedestrian {label}: False ({exc})")
            return None
        print(f"[Spawn] pedestrian {label}: {pedestrian is not None}")
        return pedestrian

    def get_available_surround_vehicle_models(self):
        try:
            from proto.morai.simulator.category_obstacles_pb2 import CategoryObstacles

            param = CategoryObstacles()
            param.vehicle = True
            objects = self.client._sim_adapter._simulator_stub.GetAvailableObject(param)
        except Exception as e:
            print(f"[MORAI] get_available_surround_vehicle_models failed: {e}")
            return []

        if objects is None:
            return []

        return list(objects.surround_vehicle)

    def get_available_pedestrian_models(self):
        try:
            from proto.morai.simulator.category_obstacles_pb2 import CategoryObstacles

            param = CategoryObstacles()
            param.pedestrian = True
            objects = self.client._sim_adapter._simulator_stub.GetAvailableObject(param)
        except Exception as e:
            print(f"[MORAI] get_available_pedestrian_models failed: {e}")
            return []

        if objects is None:
            return []

        return list(objects.pedestrian)

    def control_pedestrian(self, pedestrian, direction_x, direction_y, speed_mps, quiet=False):
        if pedestrian is None:
            return False
        try:
            from proto.morai.common.type_pb2 import Vector3

            direction = Vector3()
            direction.x = float(direction_x)
            direction.y = float(direction_y)
            direction.z = 0.0
            ok = pedestrian.control(direction, float(speed_mps))
        except Exception as exc:
            if not quiet:
                print(f"[Ped] control failed: {exc}")
            return False
        if not quiet:
            print(f"[Ped] control speed={float(speed_mps):.2f}mps: {ok}")
        return ok

    def get_all_vehicle_actor_states(self):
        try:
            from proto.morai.actor.actor_get_pb2 import GetAllActorsFilter

            param = GetAllActorsFilter()
            param.client_key = self.client_key
            param.vehicle = True
            param.pedestrian = False
            param.obstacle = False
            response = self.client._sim_adapter.get_all_actors_state(param)
        except Exception as e:
            now = time.time()
            last_log = getattr(self, "_last_get_all_vehicle_actor_states_fail_log", 0.0)
            if now - last_log >= 2.0:
                self._last_get_all_vehicle_actor_states_fail_log = now
                print(f"[MORAI] get_all_vehicle_actor_states failed: {e}")
            return {}

        states = {}
        if response is None:
            return states

        for state in getattr(response, "states", []) or []:
            actor_id = getattr(getattr(state, "actor_info", None), "id", None)
            label = getattr(actor_id, "value", "")
            if label:
                states[str(label)] = state
        return states

    def set_vehicle_speed(self, vehicle, speed_kmh, quiet=False):
        from proto.morai.actor.actor_enum_pb2 import LONG_CMD_TYPE_SPEED

        if vehicle is None:
            return False

        speed_kmh = float(speed_kmh)
        speed_mps = speed_kmh / 3.6
        ok_velocity = vehicle.set_velocity(speed_mps)
        ok_control = vehicle.control(
            long_cmd_type=LONG_CMD_TYPE_SPEED,
            throttle=0.0,
            brake=1.0 if speed_kmh <= 0.0 else 0.0,
            steer=0.0,
            velocity=speed_mps,
            acceleration=0.0,
            frame=0,
        )
        ok = bool(ok_velocity or ok_control)
        if not quiet:
            print(
                f"[Vehicle] set_speed {speed_kmh:.1f}km/h "
                f"({speed_mps:.2f}m/s): {ok} "
                f"(velocity={ok_velocity}, control={ok_control})"
            )
        return ok

    def set_vehicle_velocity(self, vehicle, velocity, quiet=False):
        if vehicle is None:
            return False

        ok = vehicle.set_velocity(float(velocity))
        if not quiet:
            print(f"[Vehicle] set_velocity {velocity}: {ok}")
        return ok

    def set_vehicle_speed_limit(self, vehicle, speed_limit_kmh, enabled=True, quiet=False):
        if vehicle is None or not hasattr(vehicle, "set_vehicle_dynamics_speed_limit"):
            return False

        speed_limit_kmh = max(0.0, float(speed_limit_kmh))
        speed_limit_mps = speed_limit_kmh / 3.6
        try:
            ok = vehicle.set_vehicle_dynamics_speed_limit(
                bool(enabled),
                speed_limit_mps,
                reset=not bool(enabled),
            )
        except Exception as exc:
            if not quiet:
                print(f"[Vehicle] set_speed_limit warning: {exc}")
            return False
        if not quiet:
            print(
                f"[Vehicle] set_speed_limit {speed_limit_kmh:.1f}km/h "
                f"({speed_limit_mps:.3f}m/s), enabled={enabled}: {ok}"
            )
        return ok

    def set_vehicle_physics(self, vehicle, enabled):
        if vehicle is None or not hasattr(vehicle, "set_physics"):
            return False

        ok = vehicle.set_physics(bool(enabled))
        print(f"[Vehicle] set_physics {enabled}: {ok}")
        return ok

    def set_vehicle_ai(self, vehicle, enabled):
        if vehicle is None or not hasattr(vehicle, "set_ai"):
            return False

        ok = vehicle.set_ai(bool(enabled))
        print(f"[Vehicle] set_ai {enabled}: {ok}")
        return ok

    def stop_vehicle(self, vehicle, quiet=False):
        from proto.morai.actor.actor_enum_pb2 import LONG_CMD_TYPE_SPEED

        if vehicle is None:
            return False

        limit_ok = self.set_vehicle_speed_limit(vehicle, 0.0, enabled=True, quiet=True)
        pause_ok = vehicle.set_pause(True)
        velocity_ok = vehicle.set_velocity(0.0)
        try:
            control_ok = vehicle.control(
                long_cmd_type=LONG_CMD_TYPE_SPEED,
                throttle=0.0,
                brake=1.0,
                steer=0.0,
                velocity=0.0,
                acceleration=0.0,
                frame=0,
            )
        except Exception:
            control_ok = False
        ok = bool(limit_ok or pause_ok or velocity_ok or control_ok)
        if not quiet:
            print(
                f"[Vehicle] stop: {ok} "
                f"(limit={limit_ok}, pause={pause_ok}, "
                f"velocity={velocity_ok}, control={control_ok})"
            )
        return ok

    def resume_vehicle_ai(self, vehicle):
        if vehicle is None:
            return False

        pause_ok = vehicle.set_pause(False)
        ai_ok = self.set_vehicle_ai(vehicle, True)
        print(f"[Vehicle] resume_ai: {bool(pause_ok or ai_ok)} (pause={pause_ok}, ai={ai_ok})")
        return bool(pause_ok or ai_ok)

    def stop(self):
        if self.client is None:
            return

        if getattr(self.client, "_simulation_world", None) is not None:
            self.client.finalize()
            print("[MORAI] finalized")
        else:
            self.client.disconnect()
            print("[MORAI] disconnected")


def _extract_xy_from_actor_state(state):
    """
    MORAI ActorState에서 x, y 위치 추출.
    """
    return (
        float(state.transform.location.x),
        float(state.transform.location.y),
    )


# class에 나중에 붙이기 위한 monkey patch 방식
def _morai_get_ego_xy(self):
    ego = self.get_ego()
    state = ego.get_actor_state()
    if state is None:
        raise RuntimeError("Failed to get ego actor state")
    return _extract_xy_from_actor_state(state)


MoraiSimBridge.get_ego_xy = _morai_get_ego_xy


def _morai_set_ego_destination(self, x, y, z, decision_range=50.0):
    from proto.morai.common.type_pb2 import Vector3

    ego = self.get_ego()

    pos = Vector3()
    pos.x = float(x)
    pos.y = float(y)
    pos.z = float(z)

    ok = ego.set_vehicle_destination(decision_range, pos)
    print(f"[Ego] set_vehicle_destination: {ok}, goal=({x:.2f}, {y:.2f}, {z:.2f})")
    return ok


def _morai_get_ego_state_debug(self):
    ego = self.get_ego()
    state = ego.get_actor_state()
    if state is None:
        return None

    x = state.transform.location.x
    y = state.transform.location.y

    vehicle_state = state.vehicle_state
    cur_link = vehicle_state.current_link_info.id.value
    remain_dist = vehicle_state.remaining_distance
    remain_link_count = vehicle_state.remaining_link_count
    is_pass_des_pos = vehicle_state.is_pass_des_pos

    return {
        "x": x,
        "y": y,
        "current_link": cur_link,
        "remaining_distance": remain_dist,
        "remaining_link_count": remain_link_count,
        "is_pass_des_pos": is_pass_des_pos,
    }


MoraiSimBridge.set_ego_destination = _morai_set_ego_destination
MoraiSimBridge.get_ego_state_debug = _morai_get_ego_state_debug


def _morai_get_ego_motion_state(self):
    ego = self.get_ego()
    state = ego.get_actor_state()
    if state is None:
        try:
            from utils.shutdown_state import is_shutdown_requested

            if is_shutdown_requested():
                raise KeyboardInterrupt
        except ImportError:
            pass
        raise RuntimeError("Failed to get ego actor state")

    vx = float(state.velocity.x)
    vy = float(state.velocity.y)
    vz = float(state.velocity.z)
    speed = (vx * vx + vy * vy + vz * vz) ** 0.5
    vehicle_state = state.vehicle_state

    return {
        "x": float(state.transform.location.x),
        "y": float(state.transform.location.y),
        "z": float(state.transform.location.z),
        "yaw_deg": float(state.transform.rotation.z),
        "speed": speed,
        "current_link": vehicle_state.current_link_info.id.value,
        "front_wheel_angle": float(vehicle_state.front_wheel_angle),
        "tl_id": vehicle_state.tl_id.value,
        "tl_color": int(vehicle_state.tl_color),
    }


MoraiSimBridge.get_ego_motion_state = _morai_get_ego_motion_state


def _morai_resolve_traffic_light_color(self, color):
    from proto.morai.infrastructure import infrastructure_enum_pb2 as tl_enum

    if isinstance(color, int):
        return int(color)

    name = str(color or "G_WITH_GLEFT").strip().upper()
    if not name.startswith("TL_COLOR_"):
        name = f"TL_COLOR_{name}"
    if not hasattr(tl_enum, name):
        raise ValueError(f"Unknown traffic light color: {color}")
    return int(getattr(tl_enum, name))


def _morai_set_traffic_light_state(
    self,
    tl_id,
    color="G_WITH_GLEFT",
    impulse=False,
    sibling=True,
    quiet=False,
):
    if not tl_id:
        return False

    from proto.morai.common.enum_pb2 import STATUS_CODE_SUCCESS
    from proto.morai.infrastructure.traffic_light_pb2 import TrafficLightStateParam

    param = TrafficLightStateParam()
    param.info.id.value = str(tl_id)
    param.info.color = self.resolve_traffic_light_color(color)
    param.is_impulse = bool(impulse)
    param.set_sibling = bool(sibling)

    try:
        result = self.world._sim_adapter.set_traffic_light_state(param)
    except Exception as exc:
        if not quiet:
            print(f"[TrafficLight] set failed tl_id={tl_id}: {exc}")
        return False

    ok = result is not None and result.status == STATUS_CODE_SUCCESS
    if not quiet:
        print(
            f"[TrafficLight] set tl_id={tl_id} color={color} "
            f"impulse={bool(impulse)} sibling={bool(sibling)} ok={ok}"
        )
    return ok


MoraiSimBridge.resolve_traffic_light_color = _morai_resolve_traffic_light_color
MoraiSimBridge.set_traffic_light_state = _morai_set_traffic_light_state


def _morai_restart_world(self, ego_transform):
    """
    MORAI world를 완전히 재시작해서 built-in cruise 내부 route 상태까지 초기화.
    """
    print("[MORAI] restart world")

    try:
        if self.world is not None:
            self.client.finalize()
    except Exception as e:
        print(f"[MORAI] finalize warning during restart: {e}")

    # client 객체를 새로 생성해서 gRPC 연결부터 다시 시작
    from api.morai_sim_client import MoraiSimClient

    self.client = MoraiSimClient(self.client_key)
    self.world = None
    self._skip_next_place_ego_transform = False

    self.connect()
    self.start_world(ego_transform)


MoraiSimBridge.restart_world = _morai_restart_world
