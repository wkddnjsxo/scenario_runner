"""
grpc_example.py

다른 터미널에서  ./run_experiment.sh 3  이 계속 돌고 있는 상태에서,
이 스크립트는 다음 동작만 수행한다.

  1) 시작 시 차량을 config.yaml 의 initial_position 으로 보낸다.
  2) AUTO_MODE 로 차량 제어 모드를 설정한다 (외부 컨트롤러가 제어 가능하도록).
  3) 메인 루프에서 reset_config.reset_interval (기본 10초) 이 경과할 때마다
       - 차량을 초기 위치로 이동
       - 속도(velocity)/조향(steer)/throttle/brake/acceleration 을 모두 0 으로 만드는
         VehicleCtrlCmd 를 짧은 간격으로 여러 번 전송하여
         외부 컨트롤러(run_experiment.sh) 가 보내는 명령을 잠시 덮어쓴다.

모든 수치/주소/모드는 config.yaml 에서 읽는다.
"""

import os
import sys
import time
import math
import yaml


# ---------------------------------------------------------------
# 경로 / 모듈 import
# ---------------------------------------------------------------
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
sys.path.append(os.path.normpath(os.path.join(current_path, 'proto')))

from proto.sim_adapter import SimAdapter
from proto.morai.actor.actor_control_pb2 import VehicleCtrlCmd
from proto.morai.actor.actor_set_pb2 import (
    SetTransformParam,
    VehicleControlModeParam,
)
from proto.morai.common.enum_pb2 import ObjectType


# ---------------------------------------------------------------
# 설정 로드
# ---------------------------------------------------------------
CONFIG_FILE = os.path.join(current_path, 'config.yaml')


def load_config(config_path):
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"설정 파일을 찾을 수 없습니다: {config_path}")
    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    print(f"✓ 설정 파일 로드 완료: {config_path}")
    return cfg


try:
    CONFIG = load_config(CONFIG_FILE)
except Exception as e:
    print(f"✗ 설정 로드 실패: {e}")
    sys.exit(1)

INITIAL_POSITION = CONFIG['initial_position']
RESET_CONFIG = CONFIG['reset_config']
ZERO_CTRL = CONFIG['zero_control']
GRPC_CONFIG = CONFIG['grpc_config']
CONTROL_MODE = CONFIG['control_mode']
SIM_CONFIG = CONFIG['simulation']


# ===============================================================
# 메인 클래스
# ===============================================================
class MORAI_gRPC:

    def __init__(self):
        self.adaptor = SimAdapter()
        self.adaptor.connect(GRPC_CONFIG['address'], GRPC_CONFIG['port'])

        self.initial_position = INITIAL_POSITION.copy()
        self.current_position = INITIAL_POSITION.copy()

        self.reset_count = 0
        self.last_reset_time = time.time()

    # -----------------------------------------------------------
    # 메인 루프
    # -----------------------------------------------------------
    def main(self):
        print("=" * 60)
        print("🚗 MORAI gRPC 리셋 컨트롤러 시작")
        print("=" * 60)
        print(f"초기 위치 : X={INITIAL_POSITION['x']:.3f}, "
              f"Y={INITIAL_POSITION['y']:.3f}, "
              f"yaw={INITIAL_POSITION['yaw']:.3f}")
        if RESET_CONFIG.get('time_based_reset', False):
            print(f"시간 리셋 : {RESET_CONFIG['reset_interval']} 초마다")
        if RESET_CONFIG.get('distance_based_reset', False):
            print(f"거리 리셋 : {RESET_CONFIG['max_distance']} m 초과 시")
        print("외부 터미널에서 ./run_experiment.sh 가 동작 중이라고 가정합니다.")
        print("=" * 60 + "\n")

        # --- 초기 설정 ---
        self.set_vehicle_position(self.initial_position)
        time.sleep(SIM_CONFIG['initial_wait_time'])

        if CONTROL_MODE.get('set_on_start', True):
            self.set_vehicle_control_mode()
            time.sleep(SIM_CONFIG['initial_wait_time'])

        # 시작 직후에도 0 명령 한 번 보내서 정지 상태로
        self.send_zero_control_cmd()

        # 리셋 타이머 시작점 재설정 (초기 setup 에 걸린 시간 제외)
        self.last_reset_time = time.time()

        # --- 모니터링 루프 ---
        max_iter = SIM_CONFIG.get('max_iterations', 0)
        log_interval = SIM_CONFIG.get('log_interval', 2.0)
        last_log_time = time.time()

        try:
            while True:
                time.sleep(SIM_CONFIG['check_interval'])

                # 주기적 로그
                now = time.time()
                elapsed = now - self.last_reset_time
                if now - last_log_time >= log_interval:
                    print(f"[모니터링] 리셋 #{self.reset_count} | "
                          f"경과: {elapsed:5.2f} s / "
                          f"{RESET_CONFIG['reset_interval']} s")
                    last_log_time = now

                # 리셋 조건 검사
                if self.check_reset_condition():
                    self.reset_vehicle()

                    if max_iter > 0 and self.reset_count >= max_iter:
                        print(f"\n✓ 최대 반복 횟수({max_iter}) 도달, 종료")
                        break

        except KeyboardInterrupt:
            print("\n\n⏹️  사용자 중단 (Ctrl+C)")
        finally:
            print("프로그램 종료")

    # -----------------------------------------------------------
    # 리셋 조건 검사
    # -----------------------------------------------------------
    def check_reset_condition(self):
        # 시간 기반
        if RESET_CONFIG.get('time_based_reset', False):
            if (time.time() - self.last_reset_time) >= RESET_CONFIG['reset_interval']:
                print(f"\n⏰ 시간 리셋 조건 충족 "
                      f"({RESET_CONFIG['reset_interval']} 초 경과)")
                return True

        # 거리 기반 (옵션)
        if RESET_CONFIG.get('distance_based_reset', False):
            dx = self.current_position['x'] - self.initial_position['x']
            dy = self.current_position['y'] - self.initial_position['y']
            if math.sqrt(dx * dx + dy * dy) > RESET_CONFIG['max_distance']:
                print("\n📏 거리 리셋 조건 충족")
                return True

        return False

    # -----------------------------------------------------------
    # 차량 리셋: 초기 위치 + 속도/조향 0
    # -----------------------------------------------------------
    def reset_vehicle(self):
        self.reset_count += 1
        print("=" * 60)
        print(f"[리셋 #{self.reset_count}] 초기 위치로 이동 + 속도/조향 0")
        print("=" * 60)

        # 1) 위치 리셋
        self.set_vehicle_position(self.initial_position)

        # 2) 짧은 안정화 대기
        time.sleep(0.1)

        # 3) 속도/조향 0 명령 (여러 번 보내서 외부 컨트롤러 명령을 덮어씀)
        self.send_zero_control_cmd()

        # 4) 타이머 갱신
        self.last_reset_time = time.time()
        print("✓ 리셋 완료\n")

    # -----------------------------------------------------------
    # 차량 위치 설정
    # -----------------------------------------------------------
    def set_vehicle_position(self, position=None):
        if position is None:
            position = self.initial_position

        req = SetTransformParam()
        req.actor_info.id.value = GRPC_CONFIG['ego_id']
        req.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        req.actor_info.client_key = GRPC_CONFIG['client_key']
        req.transform.location.x = position['x']
        req.transform.location.y = position['y']
        req.transform.location.z = position['z']
        req.transform.rotation.x = position['roll']
        req.transform.rotation.y = position['pitch']
        req.transform.rotation.z = position['yaw']

        try:
            response = self.adaptor._actor_stub.SetTransform(req)
            desc = getattr(response, 'description', '')
            print(f"  · SetTransform : {desc}")
            self.current_position = position.copy()
        except Exception as e:
            print(f"  ✗ SetTransform 실패 : {e}")

    # -----------------------------------------------------------
    # 차량 제어 모드 설정 (AUTO_MODE 등)
    # -----------------------------------------------------------
    def set_vehicle_control_mode(self):
        req = VehicleControlModeParam()
        req.actor_info.id.value = GRPC_CONFIG['ego_id']
        req.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        req.actor_info.client_key = GRPC_CONFIG['client_key']
        req.mode = CONTROL_MODE['mode_type']

        try:
            response = self.adaptor._actor_stub.SetVehicleControlMode(req)
            desc = getattr(response, 'description', '')
            print(f"  · SetVehicleControlMode (mode={CONTROL_MODE['mode_type']}) : {desc}")
        except Exception as e:
            print(f"  ✗ SetVehicleControlMode 실패 : {e}")

    # -----------------------------------------------------------
    # 속도/조향 0 명령 전송
    # -----------------------------------------------------------
    def send_zero_control_cmd(self):
        """
        외부 컨트롤러(run_experiment.sh)가 계속 명령을 보내는 환경에서도
        리셋 직후 차량이 '정지 + 조향 0' 상태가 되도록
        VehicleCtrlCmd 를 짧은 간격으로 여러 번 전송한다.
        """
        repeat = int(ZERO_CTRL.get('repeat_count', 5))
        interval = float(ZERO_CTRL.get('repeat_interval', 0.02))

        sent_ok = 0
        for i in range(repeat):
            req = VehicleCtrlCmd()
            req.actor_info.id.value = GRPC_CONFIG['ego_id']
            req.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
            req.actor_info.client_key = GRPC_CONFIG['client_key']
            req.long_cmd_type = ZERO_CTRL['long_cmd_type']
            req.throttle = ZERO_CTRL['throttle']
            req.brake = ZERO_CTRL['brake']
            req.steer = ZERO_CTRL['steer']
            req.velocity = ZERO_CTRL['velocity']
            req.acceleration = ZERO_CTRL['acceleration']

            try:
                response = self.adaptor.control_vehicle(req)
                if response is not None:
                    sent_ok += 1
            except Exception as e:
                print(f"  ✗ ControlVehicle({i}) 실패 : {e}")

            if i < repeat - 1:
                time.sleep(interval)

        print(f"  · ZeroCtrl 전송 : {sent_ok}/{repeat} 성공 "
              f"(throttle=0, brake=0, steer=0, velocity=0)")


# ===============================================================
# 진입점
# ===============================================================
if __name__ == '__main__':
    MORAI_gRPC().main()
