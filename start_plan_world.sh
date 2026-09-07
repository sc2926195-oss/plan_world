#!/usr/bin/env bash
# 每次启动前重新随机生成障碍，然后用惯用的 vrx_environment 启动世界。
# 用法: ./start_plan_world.sh [--seed 123] [-- 其它传给 vrx_environment 的参数]
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SEED_ARGS=()
if [[ "${1:-}" == "--seed" ]]; then
  SEED_ARGS=(--seed "$2"); shift 2
fi
if [[ "${1:-}" == "--" ]]; then shift; fi

python3 "$HERE/generate_obstacles.py" "${SEED_ARGS[@]}"

# 让仓库内的 model://wamv_port / model://port_berth 可被解析
export GZ_SIM_RESOURCE_PATH="$HERE${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

source /opt/ros/humble/setup.bash
source "$HOME/vrx_ws/install/setup.bash"
exec ros2 launch vrx_gz vrx_environment.launch.py \
  world:="$(realpath "$HERE/plan_world.sdf")" "$@"
