#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只测入港：把船放到港口正前方(可加横向偏移)，跑专用靠泊控制并出图。

用法:
  python3 control/dock_test.py --port 1                 # port_1 正前方12m
  python3 control/dock_test.py --port 2 --x-offset 0.4  # 偏 0.4m 测横向纠偏
"""
import argparse, math, subprocess, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORLD_DIR = HERE.parent
sys.path.insert(0, str(WORLD_DIR))
sys.path.insert(0, str(HERE))
import online_mission as om
import run_online_mission as R

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=1, choices=(0, 1, 2))
    ap.add_argument("--start-dy", type=float, default=12.0, help="起点在港口中心南侧(m)")
    ap.add_argument("--x-offset", type=float, default=0.0, help="起点横向偏移(m)")
    a = ap.parse_args()
    px = om.PORT_X[a.port]
    x0, y0 = px + a.x_offset, 100.0 - a.start_dy
    print(f"[DOCK-TEST] port_{a.port} 起点=({x0:.2f},{y0:.2f}) yaw=90°")
    subprocess.run([sys.executable, str(HERE / "reset_boat.py"),
                    "-x", str(x0), "-y", str(y0), "-yaw", "90"], check=False)
    time.sleep(1.0)
    om.TRAJ = []
    ok = om.final_dock(px, pre_y=y0 + 2.0, stop_y=100.15)
    pose = om.get_pose()
    if pose:
        print(f"[DOCK-TEST] 结束位姿 x={pose[0]:.2f} y={pose[1]:.2f} "
              f"yaw={math.degrees(pose[3]):.1f}° (泊位中心 x={px}, 内沿 y=99.0)")
    res = {"success": ok, "dock_port": a.port if ok else None, "colors": {},
           "events": [], "sensed": [], "traj": [list(p) for p in om.TRAJ]}
    out = WORLD_DIR / f"dock_test_port{a.port}.png"
    R.plot_trajectory(a.port, res, out, 26.0)
    R.plot_trajectory(a.port, res, WORLD_DIR / "dock_test_latest.png", 26.0)
    print(f"[DOCK-TEST] {'✅ 成功' if ok else '❌ 失败'}  图: {out}")
    return 0 if ok else 1

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        om.stop_all()
