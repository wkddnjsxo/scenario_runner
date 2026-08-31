#!/usr/bin/env bash
set -euo pipefail

RUNNER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_ROOT="${AIM_WS_ROOT:-/home/jang/aim_ws}"
DEFAULT_COLLECTOR_ROOT="$(cd "$RUNNER_ROOT/.." && pwd)/projects/morai-3d-detection"
COLLECTOR_ROOT="${MORAI_3D_PROJECT_ROOT:-$DEFAULT_COLLECTOR_ROOT}"
COLLECTOR_SCRIPT="${MORAI_3D_COLLECTOR_SCRIPT:-$COLLECTOR_ROOT/morai_3d_live.py}"
DATASET_ROOT="${DATASET_ROOT:-/home/jang/dataset}"

cd "$RUNNER_ROOT"

RUNNER_ARGS=("$@")
set --

if [ -f "$ROS_WS_ROOT/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$ROS_WS_ROOT/devel/setup.bash"
fi

if [ -f "$ROS_WS_ROOT/morai_bridge.env" ]; then
  # shellcheck disable=SC1091
  source "$ROS_WS_ROOT/morai_bridge.env"
fi

# Python code uses AIM_WS_ROOT only for ROS/BEV resources. gRPC and MGeo are
# resolved from RUNNER_ROOT and therefore use this standalone copy.
export AIM_WS_ROOT="$ROS_WS_ROOT"

export GRPC_POLL_STRATEGY="${GRPC_POLL_STRATEGY:-epoll1}"

set -- "${RUNNER_ARGS[@]}"

if [ "$#" -eq 0 ]; then
  set -- --zone urban --scenario random_route_drive
elif [ "${1#--}" = "$1" ]; then
  zone="$1"
  scenario="${2:-random_route_drive}"
  set -- --zone "$zone" --scenario "$scenario"
fi

collector_pid=""

cleanup() {
  if [ -n "$collector_pid" ] && kill -0 "$collector_pid" 2>/dev/null; then
    kill "$collector_pid" 2>/dev/null || true
    wait "$collector_pid" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if [ ! -f "$COLLECTOR_SCRIPT" ]; then
  echo "[run] ERROR: dataset collector not found: $COLLECTOR_SCRIPT" >&2
  echo "[run] Set MORAI_3D_PROJECT_ROOT or MORAI_3D_COLLECTOR_SCRIPT." >&2
  exit 1
fi

echo "[run] Starting dataset collector"
echo "[run] Dataset root: $DATASET_ROOT"
python3 "$COLLECTOR_SCRIPT" --dataset_root "$DATASET_ROOT" &
collector_pid="$!"

collector_ready=0
for _ in $(seq 1 50); do
  if ! kill -0 "$collector_pid" 2>/dev/null; then
    wait "$collector_pid"
  fi

  if rostopic list 2>/dev/null | grep -qx "/dataset_control"; then
    collector_ready=1
    break
  fi

  sleep 0.2
done

if [ "$collector_ready" -ne 1 ]; then
  echo "[run] ERROR: dataset collector did not become ready." >&2
  exit 1
fi

python3 "$RUNNER_ROOT/scenario_launcher.py" "$@"
