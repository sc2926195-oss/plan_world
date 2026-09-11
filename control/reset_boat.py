#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""先停船、再复位到起点 (50,0) 艏向 +90°。

用法:
  python3 control/reset_boat.py            # 复位到 (50,0,90°)
  python3 control/reset_boat.py -x 50 -y 0 -yaw 90
"""
import argparse
import math
import subprocess
import time


def stop():
    for _ in range(5):
        for side in ("left", "right"):
            subprocess.Popen(["gz", "topic", "-t", f"/wamv/thrusters/{side}/thrust",
                              "-m", "gz.msgs.Double", "-p", "data: 0"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-x", type=float, default=50.0)
    ap.add_argument("-y", type=float, default=0.0)
    ap.add_argument("-yaw", type=float, default=90.0)
    ap.add_argument("-z", type=float, default=0.1)
    a = ap.parse_args()
    print("1) 停船(推力清零 x5)...")
    stop()
    print("2) 世界复位(清掉线/角速度)...")
    subprocess.run(["gz", "service", "-s", "/world/plan_world/control",
                    "--reqtype", "gz.msgs.WorldControl",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "4000",
                    "--req", "reset { all: true }"],
                   capture_output=True, text=True)
    time.sleep(0.8)
    yaw = math.radians(a.yaw); qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = (f'name: "wamv" position {{ x: {a.x} y: {a.y} z: {a.z} }} '
           f'orientation {{ x: 0 y: 0 z: {qz:.6f} w: {qw:.6f} }}')
    print(f"3) 复位到 ({a.x},{a.y}) yaw={a.yaw}° ...")
    subprocess.run(["gz", "service", "-s", "/world/plan_world/set_pose",
                    "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "4000", "--req", req],
                   capture_output=True, text=True)
    time.sleep(0.5)
    print("4) 再停一次(防止复位瞬间残留推力)...")
    stop()
    print("完成。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
