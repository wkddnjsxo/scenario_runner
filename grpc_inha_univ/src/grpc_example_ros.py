#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MORAI gRPC + ROS 토픽 하이브리드 제어 시스템
- gRPC: 차량 위치 제어 (SetTransform, SetVehicleControlMode)
- ROS: 센서 데이터 수집 (/Ego_topic으로 속도/위치)
"""

import os, sys, time
import yaml
import subprocess
import signal
import threading
import rospy

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
sys.path.append(os.path.normpath(os.path.join(current_path, 'proto')))
from proto.sim_adapter import *

# 설정 파일 경로
CONFIG_FILE = os.path.join(current_path, 'config.yaml')

# config.yaml에서 설정 로드
def load_config(config_path):
    """YAML 설정 파일 로드"""
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"설정 파일을 찾을 수 없습니다: {config_path}")
    
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    print(f"✓ 설정 파일 로드 완료: {config_path}\n")
    return config

# 설정 로드
try:
    CONFIG = load_config(CONFIG_FILE)
except Exception as e:
    print(f"✗ 설정 로드 실패: {e}")
    sys.exit(1)

# 설정에서 필요한 값 추출
INITIAL_POSITION = CONFIG['initial_position']
RESET_CONFIG = CONFIG['reset_config']
GRPC_CONFIG = CONFIG['grpc_config']
CONTROL_MODE = CONFIG['control_mode']
SIM_CONFIG = CONFIG['simulation']

MORAI_SIM_ADDRESS = GRPC_CONFIG['address']
MORAI_SIM_PORT = GRPC_CONFIG['port']


class MORAI_gRPC_ROS:
    """gRPC (제어) + ROS 토픽 (센서)"""

    def __init__(self):        
        self.adaptor = SimAdapter()
        self.adaptor.connect(MORAI_SIM_ADDRESS, MORAI_SIM_PORT)
        
        # Signal handler 등록 (Ctrl+C 처리)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # ⚠️ 중요: MORAI 시뮬레이터 설정 확인
        print("\n" + "="*70)
        print("⚠️  [필수 설정] MORAI 시뮬레이터에서 다음을 확인하세요:")
        print("="*70)
        print("  1. Vehicle Controller    → 'AV External Controller' 로 설정")
        print("  2. Ego Controller        → 'AV External Controller' 로 설정")
        print("  3. gRPC Server Address   → " + MORAI_SIM_ADDRESS)
        print("  4. gRPC Server Port      → " + str(MORAI_SIM_PORT))
        print("="*70)
        print("위 설정이 완료되면 엔터를 누르세요...\n")
        input()
        
        # ✅ 초기 위치 저장 (ROS 초기화 전에!)
        self.initial_position = INITIAL_POSITION.copy()
        self.current_position = INITIAL_POSITION.copy()
        self.current_speed = 0.0
        self.ego_info = None
        
        # ✅ ROS 초기화 (이제 한 번만)
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('morai_grpc_ros_control', anonymous=True)
                print("✓ ROS 노드 초기화 완료")
            else:
                print("ℹ️  ROS 노드 이미 초기화됨")
        except Exception as e:
            print(f"⚠️  ROS 초기화 경고: {e}")
        
        # ✅ /Ego_topic 구독
        try:
            rospy.Subscriber('/Ego_topic', rospy.AnyMsg, self.ego_callback_generic)
            print("✓ /Ego_topic 구독 시작 (velocity, position 데이터 수집)")
            time.sleep(1)  # 구독 안정화 대기
        except Exception as e:
            print(f"⚠️  /Ego_topic 구독 실패: {e}")
        
        # 실험 프로세스 관리
        self.experiment_process = None
        self.iteration_count = 0
        self.max_iterations = CONFIG['experiment']['max_iterations']
        self.running = True
        
        # 자동 초기화 타이머 (10초 뒤)
        self.auto_reset_timer = None
        self.start_time = time.time()
        
        print("✓ 초기화 완료\n")
    
    
    def signal_handler(self, signum, frame):
        """Ctrl+C 처리"""
        print("\n\n⏹️  프로그램 중단 신호 받음 (Ctrl+C)")
        self.running = False
        self.terminate_experiment()
        print("✓ 정리 완료, 종료합니다")
        sys.exit(0)
    
    
    def ego_callback_generic(self, msg):
        """ROS 토픽 콜백: /Ego_topic에서 velocity, position 수집"""
        self.ego_info = msg
        
        try:
            # velocity (m/s) 읽기
            if hasattr(msg, 'velocity'):
                vel_x = msg.velocity.x
                vel_y = msg.velocity.y
                vel_z = msg.velocity.z
                
                # 속도 크기 계산 (m/s -> km/h)
                speed_ms = (vel_x**2 + vel_y**2 + vel_z**2) ** 0.5
                self.current_speed = speed_ms * 3.6
            else:
                self.current_speed = 0.0
            
            # position (x, y, z) 읽기
            if hasattr(msg, 'position'):
                self.current_position = {
                    'x': msg.position.x,
                    'y': msg.position.y,
                    'z': msg.position.z
                }
        except Exception as e:
            pass  # 오류 무시
    
    
    def get_vehicle_speed(self):
        """ROS 토픽에서 차량 속도 수집"""
        if self.ego_info is None:
            return 0.0
        return self.current_speed
    
    
    def get_vehicle_position(self):
        """ROS 토픽에서 차량 위치 수집"""
        if self.ego_info is None:
            return self.current_position
        return self.current_position
    
    
    # ========== 실험 프로세스 관리 함수 ==========
    
    def launch_experiment(self):
        """
        run_experiment.sh를 독립적인 환경에서 실행
        마치 새로운 터미널에서 직접 실행하는 것처럼
        """
        try:
            script_path = CONFIG['experiment']['script_path']
            scenario = CONFIG['experiment']['scenario']
            
            if not os.path.exists(script_path):
                print(f"✗ 실험 스크립트를 찾을 수 없음: {script_path}")
                return False
            
            if not os.access(script_path, os.X_OK):
                print(f"⚠️  스크립트 실행 권한 없음. 권한 추가 중...")
                os.chmod(script_path, 0o755)
            
            print(f"\n🚀 실험 시작: {script_path} {scenario}")
            print(f"   PID 추적: {os.getpid()}")
            
            # 새로운 bash 로그인 셸에서 실행 (-l 플래그)
            # 이렇게 하면 ~/.bashrc와 ~/.bash_profile이 자동 로드되고,
            # 워크스페이스의 setup.bash도 자동으로 로드됨
            cmd = f"""
set -e
echo "[실험] 환경 변수 설정 중..."
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export EXPERIMENT_MODE={scenario}
echo "[실험] EXPERIMENT_MODE={scenario} 설정"
cd /root/aim_ws
echo "[실험] 작업 디렉토리: $(pwd)"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
echo "[실험] ROS 환경 로드 완료"
echo "[실험] 최종 EXPERIMENT_MODE=$EXPERIMENT_MODE"
echo "[실험] 실행: {script_path} {scenario}"
bash {script_path} {scenario}
echo "[실험] 완료"
            """
            
            # 로그 파일 생성
            log_file = f"/tmp/experiment_run_{int(time.time())}.log"
            
            # 완전히 독립적인 환경에서 실행
            with open(log_file, 'w') as log_f:
                self.experiment_process = subprocess.Popen(
                    ['bash', '-l', '-c', cmd],  # -l: 로그인 셸로 실행
                    stdout=log_f,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid
                )
            
            print(f"✓ 실험 프로세스 시작 (PID: {self.experiment_process.pid})")
            print(f"✓ 로그 파일: {log_file}")
            return True
            
        except Exception as e:
            print(f"✗ 실험 시작 실패: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    
    def terminate_experiment(self):
        """실행 중인 실험 프로세스 종료"""
        try:
            if self.experiment_process is None:
                return True
            
            print(f"\n⛔ 실험 종료 중 (PID: {self.experiment_process.pid})...")
            
            # 프로세스 그룹 전체 종료
            try:
                os.killpg(os.getpgid(self.experiment_process.pid), signal.SIGTERM)
            except:
                os.killpg(os.getpgid(self.experiment_process.pid), signal.SIGKILL)
            
            self.experiment_process.wait(timeout=5)
            print("✓ 실험 프로세스 종료 완료")
            self.experiment_process = None
            return True
            
        except Exception as e:
            print(f"✗ 실험 종료 실패: {e}")
            return False
    
    
    def is_experiment_running(self):
        """실험 프로세스가 실행 중인지 확인"""
        if self.experiment_process is None:
            return False
        return self.experiment_process.poll() is None
    
    
    # ========== 모니터링 및 조건 확인 함수 ==========
    
    def check_speed_condition(self):
        """속도가 설정값을 초과했는지 확인 (디버깅 포함)"""
        if not RESET_CONFIG['speed_based_reset']:
            return False
        
        current_speed = self.get_vehicle_speed()
        max_speed = RESET_CONFIG['max_speed']
        
        # 디버깅: 매번 속도 출력
        print(f"[속도체크] 현재: {current_speed:.2f} km/h | 최대: {max_speed} km/h | "
              f"ego_info: {'있음' if self.ego_info else '없음'}")
        
        if current_speed > max_speed:
            print(f"🚨 속도 초과! 현재 속도: {current_speed:.1f} km/h > 최대: {max_speed} km/h")
            return True
        
        return False
    
    
    def check_reset_condition(self):
        """리셋 조건 확인"""
        # 속도 기반 리셋
        if self.check_speed_condition():
            return True
        
        # 거리 기반 리셋
        if RESET_CONFIG['distance_based_reset']:
            distance = self.calculate_distance_from_initial()
            if distance > RESET_CONFIG['max_distance']:
                print(f"⚠️  거리 초과! 현재 거리: {distance:.1f}m > 최대: {RESET_CONFIG['max_distance']}m")
                return True
        
        return False
    
    
    def calculate_distance_from_initial(self):
        """초기 위치로부터의 거리 계산"""
        import math
        dx = self.current_position['x'] - self.initial_position['x']
        dy = self.current_position['y'] - self.initial_position['y']
        distance = math.sqrt(dx**2 + dy**2)
        return distance
    
    
    def reset_and_restart(self):
        """차량을 초기 위치로 리셋하고 속도/조향을 0으로 설정"""
        print("\n" + "="*60)
        print(f"[리셋 #{self.iteration_count}] 차량을 초기 위치로 리셋")
        print("="*60)
        
        # 1. 차량 제어 초기화 (속도 0, 조향 0)
        print("차량 제어 초기화 중...")
        self.SetVehicleControl(velocity=0.0, steering=0.0)
        time.sleep(1)
        
        # 2. 차량을 초기 위치로 리셋
        print("차량 위치 리셋 중...")
        self.SetVehiclePosition(self.initial_position)
        time.sleep(SIM_CONFIG['initial_wait_time'])
        
        # 3. 제어 모드 재설정
        self.SetVehicleControlMode()
        time.sleep(SIM_CONFIG['initial_wait_time'])
        
        print("✓ 차량 리셋 완료\n")
        
        self.iteration_count += 1
    
    
    def SetVehiclePosition(self, position=None):
        """Ego 차량 위치 Setting"""
        if position is None:
            position = self.initial_position
            
        from proto.morai.common.enum_pb2 import ObjectType        
        from proto.morai.actor.actor_set_pb2 import SetTransformParam
        request = SetTransformParam()
        request.actor_info.id.value = GRPC_CONFIG['ego_id']
        request.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.actor_info.client_key = GRPC_CONFIG['client_key']
        request.transform.location.x = position['x']
        request.transform.location.y = position['y']
        request.transform.location.z = position['z']
        request.transform.rotation.x = position.get('roll', 0.0)
        request.transform.rotation.y = position.get('pitch', 0.0)
        request.transform.rotation.z = position.get('yaw', 0.0)

        try:
            response = self.adaptor._actor_stub.SetTransform(request)
            print(f'✓ 위치 설정: {response.description}')
        except Exception as e :
            print(f'✗ 위치 설정 오류: {e}')
    
    
    def SetVehicleControlMode(self):
        """Ego 차량의 ControlMode 선택 (AUTO_MODE로 설정)"""
        from proto.morai.actor.actor_set_pb2 import VehicleControlModeParam
        from proto.morai.actor.actor_enum_pb2 import VehicleControlMode
        from proto.morai.common.enum_pb2 import ObjectType
        
        mode_value = CONTROL_MODE['mode_type']
        mode_name = {0: "KEYBOARD", 1: "AUTO_MODE", 2: "CRUISE_MODE"}.get(mode_value, "UNKNOWN")
        
        request = VehicleControlModeParam()
        request.actor_info.id.value = GRPC_CONFIG['ego_id']
        request.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.actor_info.client_key = GRPC_CONFIG['client_key']
        request.mode = mode_value

        try:
            response = self.adaptor._actor_stub.SetVehicleControlMode(request)
            print(f'✓ 제어 모드 설정: {mode_name} ({mode_value}) - {response.description}')
        except Exception as e :
            print(f'✗ 제어 모드 설정 오류 ({mode_name}): {e}')
    
    
    def SetVehicleControl(self, velocity=0.0, steering=0.0):
        """차량 속도와 조향 제어 (gRPC)"""
        from proto.morai.actor.actor_set_pb2 import VehicleControlParam
        from proto.morai.common.enum_pb2 import ObjectType
        
        request = VehicleControlParam()
        request.actor_info.id.value = GRPC_CONFIG['ego_id']
        request.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.actor_info.client_key = GRPC_CONFIG['client_key']
        request.accel = velocity  # 가속도 (m/s²) - 0이면 정지
        request.steering = steering  # 조향 (도) - 0이면 직진
        
        try:
            response = self.adaptor._actor_stub.SetVehicleControl(request)
            print(f'✓ 차량 제어: 속도={velocity}, 조향={steering} - {response.description}')
        except Exception as e:
            print(f'✗ 차량 제어 오류: {e}')

    
    def main(self):
        """메인 루프"""
        print("=" * 60)
        print("🚗 MORAI gRPC + ROS 제어 시스템 시작")
        print("=" * 60)
        print(f"초기 위치: X={INITIAL_POSITION['x']}, Y={INITIAL_POSITION['y']}")
        print(f"속도 리셋 조건: {RESET_CONFIG['max_speed']} km/h")
        print("=" * 60 + "\n")
        
        # 초기 설정
        self.SetVehiclePosition(self.initial_position)
        time.sleep(SIM_CONFIG['initial_wait_time'])
        self.SetVehicleControlMode()
        time.sleep(SIM_CONFIG['initial_wait_time'])
        
        # /Ego_topic 토픽 수신 대기 (최대 10초)
        print("🔍 /Ego_topic 토픽 수신 대기 중...")
        max_wait = 10
        wait_count = 0
        while self.ego_info is None and wait_count < max_wait * 2:
            time.sleep(0.5)
            wait_count += 1
            print(f"  [{wait_count/2:.1f}s] 대기 중...", end='\r')
        
        if self.ego_info is None:
            print("⚠️  /Ego_topic 수신 실패. 하지만 계속 진행합니다.")
        else:
            print("✓ /Ego_topic 수신 성공!\n")
        
        # 차량 모니터링 시작 (run_experiment.sh 호출 제거)
        print("📌 차량 제어 모니터링 시작")
        time.sleep(1)
        
        # 메인 모니터링 루프
        try:
            last_logged_time = time.time()
            prev_position = self.initial_position.copy()
            auto_reset_triggered = False
            
            while self.iteration_count < self.max_iterations and self.running:
                time.sleep(SIM_CONFIG['check_interval'])
                
                # 10초 뒤에 자동 초기화
                elapsed_time = time.time() - self.start_time
                if elapsed_time >= 10 and not auto_reset_triggered:
                    print(f"\n⏱️  10초 경과! 차량을 초기화합니다...")
                    # 속도 0, 조향 0으로 설정
                    self.SetVehicleControl(velocity=0.0, steering=0.0)
                    time.sleep(1)
                    # 초기 위치로 리셋
                    self.SetVehiclePosition(self.initial_position)
                    time.sleep(1)
                    auto_reset_triggered = True
                    print("✓ 차량 초기화 완료\n")
                
                # 차량 상태 모니터링
                speed = self.get_vehicle_speed()
                position = self.get_vehicle_position()
                
                # 위치 변화 계산
                if position:
                    distance_moved = ((position.get('x', 0) - prev_position['x'])**2 + 
                                    (position.get('y', 0) - prev_position['y'])**2) ** 0.5
                    prev_position = position.copy() if isinstance(position, dict) else prev_position
                else:
                    distance_moved = 0.0
                
                # 상태 출력 (매 2초마다)
                current_time = time.time()
                if current_time - last_logged_time >= 2:
                    print(f"\n[모니터링] 경과시간: {elapsed_time:.1f}s | "
                          f"속도: {speed:.1f} km/h | "
                          f"위치: ({position.get('x', 0):.1f}, {position.get('y', 0):.1f}) | "
                          f"이동거리: {distance_moved:.3f}m")
                    last_logged_time = current_time
                
                # 조건 확인 및 리셋
                if self.check_reset_condition():
                    self.reset_and_restart()
            
            print(f"\n✓ 최대 반복 횟수({self.max_iterations})에 도달하여 종료")
                    
        except Exception as e:
            print(f"\n✗ 오류 발생: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.terminate_experiment()
            print("\n프로그램 종료")

        
if __name__ == '__main__':
    example = MORAI_gRPC_ROS()    
    example.main()
