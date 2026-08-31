import os, sys, time
import yaml
import subprocess
import signal
import psutil

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

class MORAI_gRPC:

    def __init__(self):        
        self.adaptor = SimAdapter()
        self.adaptor.connect(MORAI_SIM_ADDRESS, MORAI_SIM_PORT)
        
        # 초기 위치 저장
        self.initial_position = INITIAL_POSITION.copy()
        self.current_position = INITIAL_POSITION.copy()
        self.reset_count = 0
        
        # 실험 프로세스 관리
        self.experiment_process = None
        self.current_speed = 0.0
        self.iteration_count = 0
        self.max_iterations = CONFIG['experiment']['max_iterations']


    def main(self):
        """
        메인 루프: 차량 위치 설정 → 실험 실행 → 속도 모니터링 → 조건 확인 시 리셋 반복
        """
        print("=" * 60)
        print("🚗 MORAI gRPC 제어 시스템 시작")
        print("=" * 60)
        print(f"초기 위치: X={INITIAL_POSITION['x']}, Y={INITIAL_POSITION['y']}")
        print(f"속도 리셋 조건: {RESET_CONFIG['max_speed']} km/h")
        print(f"실험 스크립트: {CONFIG['experiment']['script_path']}")
        print("=" * 60 + "\n")
        
        # 초기 설정
        self.SetVehiclePosition(self.initial_position)
        time.sleep(SIM_CONFIG['initial_wait_time'])
        self.SetVehicleControlMode()
        time.sleep(SIM_CONFIG['initial_wait_time'])
        
        # 실험 시작
        if CONFIG['experiment']['enable_experiment']:
            self.launch_experiment()
            time.sleep(2)
        
        # 메인 모니터링 루프
        try:
            last_logged_time = time.time()
            prev_position = self.initial_position.copy()
            
            while self.iteration_count < self.max_iterations:
                time.sleep(SIM_CONFIG['check_interval'])
                
                # 차량 상태 모니터링
                speed = self.get_vehicle_speed()
                position = self.get_vehicle_position()
                
                # 위치 변화 계산
                distance_moved = ((position['x'] - prev_position['x'])**2 + 
                                (position['y'] - prev_position['y'])**2) ** 0.5
                prev_position = position.copy()
                
                # 상태 출력 (매 2초마다)
                current_time = time.time()
                if current_time - last_logged_time >= 2:
                    print(f"\n[모니터링] 반복#{self.iteration_count} | "
                          f"속도: {speed:.1f} km/h | "
                          f"위치: ({position['x']:.1f}, {position['y']:.1f}) | "
                          f"이동거리: {distance_moved:.3f}m")
                    print(f"[프로세스] 실험 실행중: {self.is_experiment_running()}")
                    last_logged_time = current_time
                
                # 조건 확인 및 리셋
                if self.check_reset_condition():
                    self.reset_and_restart()
                
                # 실험 프로세스 상태 확인
                if CONFIG['experiment']['enable_experiment']:
                    if not self.is_experiment_running():
                        print("⚠️  실험 프로세스가 중단됨. 재시작...")
                        self.launch_experiment()
            
            print(f"\n✓ 최대 반복 횟수({self.max_iterations})에 도달하여 종료")
                    
        except KeyboardInterrupt:
            print("\n\n⏹️  프로그램 중단 (Ctrl+C)")
        finally:
            # 정리 작업
            self.terminate_experiment()
            print("\n프로그램 종료")
    
    
    def check_reset_condition(self):
        """
        리셋 조건 확인 (YAML 설정 기반)
        조건 예시:
        - 주기적인 리셋 (config.yaml의 reset_interval)
        - 차량이 특정 거리에 도달
        - 오류 발생
        """
        # 시간 기반 리셋
        if RESET_CONFIG['time_based_reset']:
            elapsed_time = (time.time() - getattr(self, 'last_reset_time', time.time())) 
            if elapsed_time > RESET_CONFIG['reset_interval']:
                self.last_reset_time = time.time()
                return True
        
        # 거리 기반 리셋
        if RESET_CONFIG['distance_based_reset']:
            distance = self.calculate_distance_from_initial()
            if distance > RESET_CONFIG['max_distance']:
                return True
        
        # 오류 기반 리셋
        if RESET_CONFIG['error_based_reset']:
            if self.detect_error():
                return True
        
        return False
    
    
    def calculate_distance_from_initial(self):
        """
        초기 위치로부터의 거리 계산
        """
        import math
        dx = self.current_position['x'] - self.initial_position['x']
        dy = self.current_position['y'] - self.initial_position['y']
        distance = math.sqrt(dx**2 + dy**2)
        return distance
    
    
    def detect_error(self):
        """
        오류 감지 (필요시 구현)
        """
        # 예: 차량이 도로 밖으로 나갔을 때
        # 예: 센서 오류
        return False
    
    
    def reset_vehicle(self):
        """
        [더 이상 사용되지 않음] reset_and_restart() 사용
        """
        pass

    
    # ========== 센서 데이터 수집 함수 ==========
    
    def get_vehicle_speed(self):
        """
        MORAI 시뮬레이터에서 차량의 현재 속도를 수집
        Returns:
            speed: 속도 (km/h)
        """
        try:
            from proto.morai.sensor.sensor_pb2 import GetSensorDataRequest
            from proto.morai.common.object_identifier_pb2 import ObjectIdentifier
            
            request = GetSensorDataRequest()
            request.object_id.value = GRPC_CONFIG['ego_id']
            
            # 센서 데이터 요청
            response = self.adaptor._sensor_stub.GetSensorData(request)
            
            if response and hasattr(response, 'ground_truth'):
                # Ground truth에서 속도 정보 추출
                velocity_vector = response.ground_truth.data.velocity
                # 3D 속도를 스칼라로 변환 (m/s를 km/h로)
                speed_ms = (velocity_vector.x**2 + velocity_vector.y**2 + velocity_vector.z**2) ** 0.5
                speed_kmh = speed_ms * 3.6
                self.current_speed = speed_kmh
                return speed_kmh
            
            return 0.0
        except Exception as e:
            print(f"[경고] 속도 데이터 수집 실패: {e}")
            return 0.0
    
    
    def get_vehicle_position(self):
        """
        MORAI 시뮬레이터에서 차량의 현재 위치를 수집
        Returns:
            position: {'x', 'y', 'z'} 위치 정보
        """
        try:
            from proto.morai.sensor.sensor_pb2 import GetSensorDataRequest
            from proto.morai.common.object_identifier_pb2 import ObjectIdentifier
            
            request = GetSensorDataRequest()
            request.object_id.value = GRPC_CONFIG['ego_id']
            
            response = self.adaptor._sensor_stub.GetSensorData(request)
            
            if response and hasattr(response, 'ground_truth'):
                position_data = response.ground_truth.data.position
                position = {
                    'x': position_data.x,
                    'y': position_data.y,
                    'z': position_data.z
                }
                self.current_position = position.copy()
                return position
            
            return self.current_position
        except Exception as e:
            print(f"[경고] 위치 데이터 수집 실패: {e}")
            return self.current_position
    
    
    # ========== 실험 프로세스 관리 함수 ==========
    
    def launch_experiment(self):
        """
        run_experiment.sh를 subprocess로 실행 (출력 보이게 수정)
        """
        try:
            script_path = CONFIG['experiment']['script_path']
            scenario = CONFIG['experiment']['scenario']
            
            if not os.path.exists(script_path):
                print(f"✗ 실험 스크립트를 찾을 수 없음: {script_path}")
                return False
            
            print(f"\n🚀 실험 시작: {script_path} {scenario}")
            
            # subprocess로 실험 스크립트 실행 (출력 보임)
            self.experiment_process = subprocess.Popen(
                ['bash', script_path, str(scenario)],
                stdout=None,  # 터미널에 출력 보이도록
                stderr=None,

                preexec_fn=os.setsid  # 프로세스 그룹 생성
            )
            
            print(f"✓ 실험 프로세스 시작 (PID: {self.experiment_process.pid})")
            return True
            
        except Exception as e:
            print(f"✗ 실험 시작 실패: {e}")
            return False
    
    
    def terminate_experiment(self):
        """
        실행 중인 실험 프로세스 종료
        """
        try:
            if self.experiment_process is None:
                print("[정보] 실행 중인 실험이 없음")
                return True
            
            print(f"\n⛔ 실험 종료 중 (PID: {self.experiment_process.pid})...")
            
            # 프로세스 그룹 전체 종료
            try:
                os.killpg(os.getpgid(self.experiment_process.pid), signal.SIGTERM)
            except:
                # SIGTERM 실패 시 SIGKILL 사용
                os.killpg(os.getpgid(self.experiment_process.pid), signal.SIGKILL)
            
            # 프로세스 종료 대기
            self.experiment_process.wait(timeout=5)
            print("✓ 실험 프로세스 종료 완료")
            self.experiment_process = None
            return True
            
        except Exception as e:
            print(f"✗ 실험 종료 실패: {e}")
            return False
    
    
    def is_experiment_running(self):
        """
        실험 프로세스가 실행 중인지 확인
        """
        if self.experiment_process is None:
            return False
        
        return self.experiment_process.poll() is None
    
    
    # ========== 모니터링 및 조건 확인 함수 ==========
    
    def check_speed_condition(self):
        """
        속도가 설정값을 초과했는지 확인
        Returns:
            bool: 초과 시 True
        """
        if not RESET_CONFIG['speed_based_reset']:
            return False
        
        current_speed = self.get_vehicle_speed()
        max_speed = RESET_CONFIG['max_speed']
        
        if current_speed > max_speed:
            print(f"⚠️  속도 초과! 현재 속도: {current_speed:.1f} km/h > 최대: {max_speed} km/h")
            return True
        
        return False
    
    
    def check_reset_condition(self):
        """
        리셋 조건 확인 (YAML 설정 기반)
        """
        # 속도 기반 리셋
        if self.check_speed_condition():
            return True
        
        # 거리 기반 리셋
        if RESET_CONFIG['distance_based_reset']:
            distance = self.calculate_distance_from_initial()
            if distance > RESET_CONFIG['max_distance']:
                print(f"⚠️  거리 초과! 현재 거리: {distance:.1f}m > 최대: {RESET_CONFIG['max_distance']}m")
                return True
        
        # 오류 기반 리셋
        if RESET_CONFIG['error_based_reset']:
            if self.detect_error():
                return True
        
        return False
    
    
    def reset_and_restart(self):
        """
        차량을 초기 위치로 리셋하고 실험을 다시 시작
        """
        print("\n" + "="*60)
        print(f"[리셋 #{self.iteration_count}] 차량을 초기 위치로 리셋하고 재시작")
        print("="*60)
        
        # 1. 현재 실험 종료
        self.terminate_experiment()
        time.sleep(1)
        
        # 2. 차량을 초기 위치로 리셋
        print("차량 위치 리셋 중...")
        self.SetVehiclePosition(self.initial_position)
        time.sleep(SIM_CONFIG['initial_wait_time'])
        
        # 3. 제어 모드 재설정
        self.SetVehicleControlMode()
        time.sleep(SIM_CONFIG['initial_wait_time'])
        
        print("✓ 차량 리셋 완료\n")
        
        # 4. 새로운 실험 시작
        if CONFIG['experiment']['enable_experiment']:
            self.launch_experiment()
        
        self.iteration_count += 1
    
    
    # 차량을 특정 위치에 배치하는 예제
    def SetVehiclePosition(self, position=None):
        """
        Ego 차량 위치 Setting
        Args:
            position: 위치 딕셔너리 {'x', 'y', 'z', 'roll', 'pitch', 'yaw'}
                     None이면 초기 위치 사용
        """
        if position is None:
            position = self.initial_position
            
        from proto.morai.common.enum_pb2 import ObjectType        
        from proto.morai.actor.actor_set_pb2 import SetTransformParam
        request = SetTransformParam()
        request.actor_info.id.value = 'Ego'
        request.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.actor_info.client_key = 'Morai_Example'
        request.transform.location.x = position['x']
        request.transform.location.y = position['y']
        request.transform.location.z = position['z']
        request.transform.rotation.x = position['roll']
        request.transform.rotation.y = position['pitch']
        request.transform.rotation.z = position['yaw']

        try:
            response = self.adaptor._actor_stub.SetTransform(request)
            print(f'SetVehiclePosition Response : {response.description}')
            # 현재 위치 저장
            self.current_position = position.copy()
        except Exception as e :
            print(f'SetVehiclePosition Error : {e}')
        


    # 차량 제어 모드 변경 예제
    def SetVehicleControlMode(self):
        from proto.morai.actor.actor_set_pb2 import VehicleControlModeParam
        from proto.morai.actor.actor_enum_pb2 import VehicleControlMode
        from proto.morai.common.enum_pb2 import ObjectType
        """
        Ego 차량의 ControlMode 선택 (YAML 설정 기반)
        VehicleControlMode.VEHICLE_CONTROL_KEYBOARD = 0
        VehicleControlMode.VEHICLE_CONTROL_AUTO_MODE = 1
        VehicleControlMode.VEHICLE_CONTROL_CRUISE_MODE = 2
        """
        
        request = VehicleControlModeParam()
        request.actor_info.id.value = GRPC_CONFIG['ego_id']
        request.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.actor_info.client_key = GRPC_CONFIG['client_key']
        request.mode = CONTROL_MODE['mode_type']

        try:
            response = self.adaptor._actor_stub.SetVehicleControlMode(request)
            print(f'SetVehicleControlMode Response : {response.description}')
        except Exception as e :
            print(f'SetVehicleControlMode Error : {e}')

        
if __name__ == '__main__':

    example = MORAI_gRPC()    
    example.main()
            