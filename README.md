# Scenario Runner

`scenario_runner`는 MORAI 주행 시나리오 실행과 3카메라·LiDAR 데이터 수집을
자동화하는 로컬 실행기입니다. `grpc_inha_univ`와 K-City MGeo 지도 복사본은 이
폴더 안에 포함되어 있지만, 현재 상태는 완전한 독립 배포판이 아닙니다.

## 호환성 및 이식성 상태

> **현재 저장소만 다른 PC에 clone해서 바로 실행할 수는 없습니다.**
>
> 사용자 이름에 대한 하드코딩은 제거되어 `$HOME` 기준으로 경로를 계산합니다.
> 그러나 ROS 작업공간, 데이터 수집기, MORAI Simulator와 센서 네트워크 설정은
> 저장소 외부에 있으므로 이 저장소 하나만으로는 실행할 수 없습니다.

### 저장소에 포함된 항목

| 항목 | 저장소 내부 경로 | 비고 |
|---|---|---|
| 시나리오 실행 코드 | 저장소 루트, `zones/`, `utils/` | 포함 |
| MORAI gRPC Python 코드 | `grpc_inha_univ/` | 원본 작업공간에서 복사한 별도 사본 |
| K-City MGeo JSON | `R_KR_PG_KATRI/` | 원본 작업공간에서 복사한 별도 사본 |
| 시나리오 설정 | `config/` | 현재 장비와 데이터 수집 목적에 맞춘 설정 |

### 저장소에 포함되지 않은 필수 항목

| 항목 | 현재 기본 위치/조건 | 없을 때 발생하는 문제 |
|---|---|---|
| ROS Noetic 환경 | Ubuntu 20.04 + ROS Noetic | `rospy`, ROS 도구를 불러올 수 없음 |
| 빌드된 catkin 작업공간 | `$HOME/aim_ws/devel/setup.bash` | `morai_msgs` 등 ROS 메시지를 불러올 수 없음 |
| 데이터 수집기 프로젝트 | `$HOME/projects/morai-3d-detection` | `morai_3d_live.py`를 찾지 못해 `run.sh` 종료 |
| MORAI Simulator | gRPC `192.168.80.1:7789` | 시나리오가 MORAI에 연결되지 않음 |
| rosbridge 연결 | 1개 이상 필요; 현재 장비는 TCP 9090/9091 두 개 사용 | MORAI ROS 토픽이 들어오지 않음 |
| MORAI 센서 설정 | 아래의 정확한 ROS 토픽 발행 | 동기화 프레임이 저장되지 않음 |

데이터 수집기는 현재 다음 프로젝트의 여러 로컬 모듈에도 의존합니다.

```text
$HOME/projects/morai-3d-detection/
├── morai_3d_live.py
├── morai_sync.py
├── morai_dataset.py
├── camera_configs.py
└── verify_lidar_camera_overlay.py
```

따라서 이 저장소의 `requirements.txt`만 설치하는 것으로는 전체 데이터 수집 환경이
구성되지 않습니다. 수집기 실행에는 ROS Python 패키지 외에도 OpenCV, NumPy,
SciPy, PyTorch 등이 필요합니다.

### 현재 코드의 기본 경로와 변경 방법

| 용도 | 기본값 | 변경 방법 |
|---|---|---|
| ROS/catkin 작업공간 | `$HOME/aim_ws` | `AIM_WS_ROOT` |
| 수집기 프로젝트 | `$HOME/projects/morai-3d-detection` | `MORAI_3D_PROJECT_ROOT` |
| 수집기 단일 파일 | `<수집기 프로젝트>/morai_3d_live.py` | `MORAI_3D_COLLECTOR_SCRIPT` |
| 데이터 저장 위치 | `$HOME/dataset` | `DATASET_ROOT` |
| MORAI gRPC 주소 | `192.168.80.1:7789` | `config/local_override.yaml` |

아래의 기본 폴더 구조와 다른 위치에 설치했다면 다음처럼 환경을 지정합니다.

```bash
export AIM_WS_ROOT="$HOME/aim_ws"
export MORAI_3D_PROJECT_ROOT="$HOME/projects/morai-3d-detection"
export DATASET_ROOT="$HOME/dataset"
./run.sh
```

아래 공통 폴더 구조를 사용한다면 환경변수를 별도로 지정할 필요가 없습니다.

```text
$HOME/
├── aim_ws/
│   └── devel/setup.bash
├── scenario_runner/
└── projects/
    └── morai-3d-detection/
        └── morai_3d_live.py
```

`AIM_WS_ROOT` 아래에는 빌드가 완료된 `devel/setup.bash`와 `morai_msgs` 패키지가
있어야 합니다. `run.sh`는 rosbridge를 실행하지 않으므로 rosbridge와 MORAI 센서
연결은 실행 전에 별도로 준비해야 합니다.

### 필요한 ROS 토픽

수집기는 아래 토픽 이름을 정확히 구독합니다.

```text
/cam_front
/cam_front_left
/cam_front_right
/lidar3D
/Ego_topic
/Object_topic
```

현재 장비에서는 카메라 3개와 LiDAR 1개를 rosbridge 9090에 연결하고, 나머지
MORAI 토픽을 rosbridge 9091에 연결합니다. 두 rosbridge는 동일한 ROS master
`http://127.0.0.1:11311`을 사용해야 합니다. 포트 분배 자체는 필수가 아니지만,
위 토픽들이 하나의 ROS master에서 보여야 합니다.

실행 전 확인:

```bash
rostopic list | grep -E '^/(cam_front|cam_front_left|cam_front_right|lidar3D|Ego_topic|Object_topic)$'
```

### 공개 저장소로 배포할 때의 주의사항

- `R_KR_PG_KATRI` 지도와 `grpc_inha_univ`에는 현재 별도 라이선스 파일이 없습니다.
  공개 저장소에 올리기 전에 재배포 권한을 확인해야 합니다.
- `.gitignore`는 `__pycache__/`, `runtime/`, `ego_pose_checks/`, 로컬 override와
  실행 산출물을 제외합니다.
- 완전한 단일 저장소 clone-and-run 구성을 만들려면 데이터 수집기 코드 포함,
  ROS 설치 및 `morai_msgs` 빌드 자동화, 전체 의존성 명시가 추가로 필요합니다.

## License

이 저장소에서 직접 작성한 Scenario Runner 소스 코드는 [MIT License](LICENSE)에
따라 사용, 수정 및 재배포할 수 있습니다.

`R_KR_PG_KATRI/`의 지도 데이터와 `grpc_inha_univ/`의 MORAI 관련 자료는 이 MIT
License의 적용 대상이 아닙니다. 해당 자료의 사용 및 재배포 가능 여부는 원
저작권자 또는 공급자의 조건을 별도로 확인해야 합니다.

## Run

```bash
cd "$HOME/scenario_runner"
./run.sh
```

`run.sh` starts the MORAI 3-camera/LiDAR dataset collector first, waits until
its `/dataset_control` subscription is ready, and then starts the scenario.
Collected scenes are written under `$HOME/dataset` by default. Override
the locations with `DATASET_ROOT`, `MORAI_3D_PROJECT_ROOT`, or
`MORAI_3D_COLLECTOR_SCRIPT` when needed. On `Ctrl+C`, the collector removes the
scene that was still in progress while preserving completed scenes.

Equivalent explicit form:

```bash
./run.sh urban random_route_drive
```

MORAI must be running with gRPC server enabled at `192.168.80.1:7789`
from this WSL/container environment.

## Config

Default config lives in:

- `config/runtime.yaml`
- `config/urban_scenarios.yaml`
- `config/urban_route_links.yaml`

Machine-specific overrides can be placed in `config/local_override.yaml`; start
from `config/local_override.yaml.example` if needed.

By default, `random_route_drive` uses direct gRPC pure pursuit control.

The route BEV visualizer can export the selected route to
`runtime/current_route.json`, display it in an OpenCV window named `AIM Route`,
and publish:

```text
/aim_scenario_runner/bev_route/image
```

It is disabled in the current `config/urban_scenarios.yaml`. Enable it with:

```yaml
route_bev_visualizer_enabled: true
```

The visualizer is not standalone; it uses LBC/BEV files from `AIM_WS_ROOT`.

## NPC Vehicles

`random_route_drive` can maintain rolling NPC traffic around Ego. The feature is
configured under `scenarios.random_route_drive.npc` in
`config/urban_scenarios.yaml`.

```yaml
npc:
  enabled: true
  spawn_management:
    target_npc_count_min: 3
    target_npc_count_max: 5
```

NPC spawn positions are sampled from the selected route using `s_offset_m` and
`lateral_offset_m`; valid slots are restricted by the saved road group such as
`1차선`, `2차선(왼쪽)`, or `일방통행`. Vehicle models are selected from MORAI
surround vehicles after filtering to sedan/suv/mpv/wagon names and excluding
`Default_` or `Defalut_` prefixes. If MORAI model names do not include those
category words, fill `npc.vehicle_model.whitelist` with exact model names.
Opposite-lane NPCs use `opposite: true` slots and, by default, automatically find
nearby reverse-direction links from MGeo geometry. If a specific link pair should
be forced, add it under `npc.road_groups.<group>.opposite_links`. One-way groups
keep `has_opposite_lane: false`.

For each scenario, the NPC target count is selected once from
`target_npc_count_min` to `target_npc_count_max` and maintained until the
scenario ends. When a distant NPC is despawned, the replacement is spawned at the
front slot's maximum distance, kept stopped, and then released when Ego comes
within `activate_distance_m`.
Road groups can override the count with
`target_npc_count_by_road_group`, for example one-way roads use fewer NPCs than
three-lane roads.
Initial NPCs are spawned before Ego driving starts, with a short
settle delay so MORAI and the BEV visualizer can observe the new actors.
When waiting NPCs are released, their target speed is ramped up gradually using
`speed_ramp_step_kmh` and `speed_ramp_interval_sec`.
NPC target speed is synchronized from the scenario's Ego target speed, and
same-direction NPCs receive the selected Ego route suffix from their spawn link.

Disable NPCs with:

```yaml
npc:
  enabled: false
```

## Save Ego Pose

MORAI에서 Ego를 원하는 위치에 둔 뒤 현재 위치를 저장하고 전체맵 preview를
확인하려면:

```bash
python3 tools/save_ego_pose.py --name spawn_urban_01
```

The tool writes the pose to `config/saved_ego_positions.yaml`, saves a full-map
preview under `ego_pose_checks/`, and then asks whether to add the detected link
to `config/urban_route_links.yaml`. Enter `Y` only after confirming the preview.

Candidate links are grouped under:

```yaml
candidate_link_groups:
  1차선: []
  2차선(왼쪽): []
  2차선(오른쪽): []
  3차선(중간): []
  3차선(왼쪽): []
  3차선(오른쪽): []
  일방통행: []
```
