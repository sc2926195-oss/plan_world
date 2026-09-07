#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键流程：随机(或固定)生成障碍 -> 规划三条航道并出图 -> 打开 Gazebo 仿真。

用法：
  python3 run_plan_world.py                 # 全部随机，然后启动仿真(前台阻塞)
  python3 run_plan_world.py --seed 2026     # 固定种子，可复现
  python3 run_plan_world.py --no-launch     # 只生成障碍+规划+出图，不启动仿真
  python3 run_plan_world.py --keep-obstacles --no-launch
                                            # 保留当前障碍布局，只重新规划出图

启动仿真后(另开终端)：
  python3 -m planner.gz_models show --lane 0|1|2    # 显示某条航道(推荐，模型方式必显示)
  python3 -m planner.gz_models clear                # 清除
  python3 -m planner.gz_models demo                 # 依次展示三条
  (marker 实验方式见 planner/gz_markers.py)

说明：
  - 本脚本保证“世界里的障碍”和“规划用的真值/图片”来自同一个 seed，不会错位。
  - 直接跑 start_plan_world.sh 会再随机一次障碍，可能与 planner 的图不一致；
    二者二选一：要么用本脚本，要么 start_plan_world.sh --seed 与 planner 相同。
"""
from __future__ import annotations

import argparse
import os
import random
import subprocess
import sys
from pathlib import Path

WORLD_DIR = Path(__file__).resolve().parent
INSTALL_SETUP = WORLD_DIR.parent / "install" / "setup.bash"
ROS_SETUP = Path("/opt/ros/humble/setup.bash")


def run(cmd, **kw):
    print(f"\n$ {' '.join(cmd) if isinstance(cmd, list) else cmd}\n")
    return subprocess.run(cmd, **kw)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", type=int, default=None,
                    help="障碍与规划的随机种子(默认系统随机)")
    ap.add_argument("--no-launch", action="store_true",
                    help="不启动 Gazebo 仿真")
    ap.add_argument("--keep-obstacles", action="store_true",
                    help="不重新随机障碍，沿用当前 plan_world.sdf/obstacles_meta.json")
    ap.add_argument("--planner-extra", default="",
                    help="透传给 plan_corridors 的额外参数，如 '--half-width 1.6'")
    args = ap.parse_args()

    seed = args.seed
    if seed is None:
        seed = random.SystemRandom().randint(0, 2**31 - 1)

    env = os.environ.copy()
    env.setdefault("MPLCONFIGDIR", "/tmp/mpl")   # 避免 matplotlib 缓存目录警告

    # 1) 生成障碍（写进 plan_world.sdf + obstacles_meta.json）
    if not args.keep_obstacles:
        r = run([sys.executable, str(WORLD_DIR / "generate_obstacles.py"),
                 "--seed", str(seed)], cwd=WORLD_DIR, env=env)
        if r.returncode != 0:
            return r.returncode

    # 2) 规划三条航道 + 出图
    planner_cmd = [sys.executable, "-m", "planner.plan_corridors",
                   "--seed", str(seed)]
    if args.planner_extra:
        planner_cmd += args.planner_extra.split()
    r = run(planner_cmd, cwd=WORLD_DIR, env=env)
    if r.returncode != 0:
        return r.returncode

    print("\n图片已生成: " + str(WORLD_DIR / "planner" / "planner_overview.png"))
    if args.no_launch:
        print("(未启动仿真。需要显示走廊时先启动世界，再执行：")
        print("  python3 -m planner.gz_markers show --lane 0|1|2 / clear / demo)")
        return 0

    # 3) 启动仿真(前台阻塞；Ctrl-C 退出)
    world_sdf = WORLD_DIR / "plan_world.sdf"
    if not world_sdf.exists():
        print(f"找不到世界文件: {world_sdf}")
        return 1
    if not ROS_SETUP.exists():
        print(f"找不到 ROS 环境: {ROS_SETUP}，请先 source 后再手动启动")
        return 1
    env_source = f"source {ROS_SETUP} && source {INSTALL_SETUP}"
    if not INSTALL_SETUP.exists():
        env_source = f"source {ROS_SETUP}"
    launch_cmd = (f"{env_source} && ros2 launch vrx_gz vrx_environment.launch.py "
                  f"world:=\"{world_sdf}\"")
    print(f"\n启动仿真(本终端将阻塞，Ctrl-C 退出):\n  {launch_cmd}\n")
    try:
        r = subprocess.run(["bash", "-lc", launch_cmd], cwd=WORLD_DIR, env=env)
    except KeyboardInterrupt:
        print("\n已停止仿真。")
        return 0
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())
