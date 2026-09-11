#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键在线任务：随机(或指定 seed) 生成障碍+绿港 -> 启动 Gazebo -> 自动开跑
-> 记录实际轨迹 -> 出图。

用法：
  python3 run_online_mission.py                 # 随机种子，随机绿港
  python3 run_online_mission.py --seed 100      # 可复现(seed 同时决定绿港)
  python3 run_online_mission.py --order 0,1,2   # 拜访顺序(默认最短扫掠)
  python3 run_online_mission.py --no-keep-world # 跑完关掉仿真

输出：
  online_run_<seed>.png / online_run_latest.png   实际运行轨迹图
  online_run_<seed>.json                          轨迹+判色+感知数据
"""
from __future__ import annotations

import argparse
import json
import os
import random
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "control"))

ROS_SETUP = Path("/opt/ros/humble/setup.bash")
INSTALL_SETUP = HERE.parent / "install" / "setup.bash"
WORLD_SDF = HERE / "plan_world.sdf"
PORT_X = {0: 0.0, 1: 50.0, 2: 100.0}


def sh(cmd, **kw):
    return subprocess.run(["bash", "-lc", cmd], **kw)


def world_running() -> bool:
    try:
        out = subprocess.run(["gz", "service", "--list"], capture_output=True,
                             text=True, timeout=6).stdout
        return "/world/plan_world/control" in out
    except Exception:
        return False


def launch_world(log_path: Path):
    env_src = f"source {ROS_SETUP}"
    if INSTALL_SETUP.exists():
        env_src += f" && source {INSTALL_SETUP}"
    cmd = (f"{env_src} && cd {HERE} && export DISPLAY=:0 && "
           f"exec ros2 launch vrx_gz vrx_environment.launch.py "
           f"world:=\"{WORLD_SDF}\"")
    log = open(log_path, "w")
    p = subprocess.Popen(["bash", "-lc", cmd], stdout=log, stderr=subprocess.STDOUT,
                         start_new_session=True)
    print(f"[LAUNCH] Gazebo 启动中 (pid={p.pid}, log={log_path}) ...")
    return p


def wait_ready(timeout=150.0) -> bool:
    """等到 /world/plan_world/pose/info 里出现 wamv。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            r = subprocess.run(["timeout", "4", "gz", "topic", "-e", "-t",
                                "/world/plan_world/pose/info", "--json-output"],
                               capture_output=True, text=True, timeout=8)
            if '"name":"wamv"' in r.stdout.replace(" ", ""):
                return True
        except Exception:
            pass
        time.sleep(2.0)
    return False


def plot_trajectory(seed, res, out_png: Path, checkpoint_dy: float):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from planner.plan_corridors import dock_wall_rects

    traj = res.get("traj", [])
    fig, ax = plt.subplots(figsize=(7.6, 6.8), dpi=140)
    ax.add_patch(plt.Rectangle((0, 0), 100, 100, fill=False, ec="0.6", lw=1))
    # 港口
    for px in PORT_X.values():
        for poly in dock_wall_rects(px):
            ax.add_patch(plt.Polygon(poly, closed=True, color="0.45"))
        ax.plot(px, 100, marker="v", color="0.3", ms=6)
        ax.text(px, 106, f"port_{int(px/50)}", ha="center", fontsize=8, color="0.3")
    # 感知到的障碍(最终地图)
    for (x, y, r) in res.get("sensed", []):
        ax.add_patch(plt.Circle((x, y), r, fill=False, ec="crimson",
                                ls=":", lw=1.1))
    # 实际轨迹
    if traj:
        xs = [p[1] for p in traj]; ys = [p[2] for p in traj]
        ax.plot(xs, ys, color="tab:blue", lw=1.8, label="actual track")
        ax.plot(xs[0], ys[0], marker="o", color="limegreen", ms=8, label="start")
        ax.plot(xs[-1], ys[-1], marker="*", color="red", ms=14, label="stop")
        # 每 30s 打点
        t0 = traj[0][0]
        for i, (t, x, y, _yaw) in enumerate(traj):
            if i and t - t0 >= 30 and (t - t0) % 30 < 0.35:
                ax.plot(x, y, marker=".", color="tab:blue", ms=5)
                ax.text(x + 0.6, y + 0.6, f"{int(t-t0)}s", fontsize=6,
                        color="tab:blue")
    # 检查点 + 判色结果
    for ev in res.get("events", []):
        idx = ev["port"]; px = PORT_X[idx]
        c = ev.get("color", "?")
        col = {"green": "green", "red": "red"}.get(c, "gray")
        ax.plot(px, 100 - checkpoint_dy, marker="o", mfc=col, mec="k", ms=9)
        ax.text(px + 1.2, 100 - checkpoint_dy, f"port_{idx}: {c.upper()}",
                fontsize=8, color=col)
    ok = res.get("success")
    ax.set_title(f"online mission  seed={seed}  "
                 + (f"docked=port_{res.get('dock_port')} [OK]" if ok else "FAILED"))
    ax.set_xlim(-6, 106); ax.set_ylim(-8, 112)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.25)
    ax.legend(loc="lower center", fontsize=8)
    fig.tight_layout(); fig.savefig(out_png); plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", type=int, default=None, help="不传=每次随机")
    ap.add_argument("--order", default="0,1,2")
    ap.add_argument("--checkpoint-dy", type=float, default=26.0)
    ap.add_argument("--no-keep-world", action="store_true",
                    help="跑完关掉 Gazebo(默认保留)")
    ap.add_argument("--dock-only", type=int, default=None, choices=(0, 1, 2),
                    help="只测入港: 把船直接放到该港正前方, 执行专用靠泊并出图")
    ap.add_argument("--dock-start-dy", type=float, default=12.0,
                    help="--dock-only 时, 起点在港口中心南侧的距离(m)")
    args = ap.parse_args()

    seed = args.seed if args.seed is not None else random.SystemRandom().randint(0, 2**31 - 1)
    print(f"[1/5] 生成场景(障碍+绿港), seed={seed}")
    r = sh(f"cd {HERE} && python3 generate_obstacles.py --seed {seed}")
    if r.returncode != 0:
        print("生成场景失败"); return 1

    proc = None
    print("[2/5] 检查/启动 Gazebo 世界")
    if world_running():
        print("      已有 plan_world 在运行，直接使用")
    else:
        proc = launch_world(Path("/tmp/run_online_mission_gazebo.log"))
        if not wait_ready():
            print("世界启动超时，请查看 /tmp/run_online_mission_gazebo.log")
            return 1
    print("      世界就绪")

    print("[3/5] 停船并复位到起点 (50,0,90°)")
    sh(f"cd {HERE} && python3 control/reset_boat.py")

    import online_mission as om
    if args.dock_only is not None:
        px = PORT_X[args.dock_only]
        y0 = 100.0 - args.dock_start_dy
        print(f"[DOCK-TEST] 把船放到 port_{args.dock_only} 正前方 ({px:.0f},{y0:.0f}) 并只测入港")
        sh(f"cd {HERE} && python3 control/reset_boat.py -x {px} -y {y0} -yaw 90")
        om.TRAJ = []
        ok = om.final_dock(px, pre_y=y0 + 2.0, stop_y=100.15)
        res = {"success": ok, "dock_port": args.dock_only if ok else None,
               "colors": {}, "events": [], "sensed": [],
               "traj": [list(p_) for p_ in om.TRAJ]}
        out_png = HERE / f"dock_test_port{args.dock_only}.png"
        plot_trajectory(seed, res, out_png, args.checkpoint_dy)
        plot_trajectory(seed, res, HERE / "dock_test_latest.png", args.checkpoint_dy)
        (HERE / f"dock_test_port{args.dock_only}.json").write_text(
            json.dumps(res, indent=2), encoding="utf-8")
        print(f"\n入港测试: {'成功' if ok else '失败'}\n图: {out_png}")
        if proc is not None and args.no_keep_world:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        return 0 if ok else 1

    print("[4/5] 在线任务(感知避障 + 判色 + 进港)")
    order = [int(v) for v in args.order.split(",") if v.strip()]
    try:
        res = om.run_mission(order, args.checkpoint_dy)
    finally:
        om.stop_all()

    print("[5/5] 保存轨迹图 + 数据")
    out_png = HERE / f"online_run_{seed}.png"
    plot_trajectory(seed, res, out_png, args.checkpoint_dy)
    (HERE / f"online_run_{seed}.json").write_text(
        json.dumps({"seed": seed, "order": order, **res}, indent=2),
        encoding="utf-8")
    plot_trajectory(seed, res, HERE / "online_run_latest.png", args.checkpoint_dy)

    print("\n=========== 结果 ===========")
    print(f"seed={seed}  成功={'是' if res.get('success') else '否'}  "
          f"入港={('port_%s' % res['dock_port']) if res.get('dock_port') is not None else '无'}")
    for idx, c in sorted(res.get("colors", {}).items()):
        print(f"  port_{idx}: {c.upper()}")
    print(f"轨迹图: {out_png}")
    print("============================")
    if proc is not None and args.no_keep_world:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        print("已关闭 Gazebo")
    elif proc is not None:
        print("Gazebo 保持运行；要关闭: pkill -f 'gz sim'")
    return 0 if res.get("success") else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n中断")
        raise SystemExit(130)
