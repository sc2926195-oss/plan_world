# -*- coding: utf-8 -*-
"""
小船(boat_real) 前视相机 颜色(红/绿灯信标) 感知测试。

用法: 先开 plan_world (port_1 绿 / port_0,2 红), 再:
  python3 control/boat_color_test.py            # 正对 port_1 距离扫描 + 偏航扫描
判据沿用旧 WAM-V 扫描: 绿灯 3s 亮/2s 灭 → 用 ~5s 采样帧的绿色像素
亮-暗差幅(toggle) 判断能否“看到闪烁的绿灯”。码头上的绿杆不闪=常量, 不影响。
输出: 打印结果表, 并把每距离一张亮态帧存到 /tmp/boat_cam_<d>m.png。
"""
import argparse, base64, json, math, subprocess, time
import numpy as np
from pathlib import Path

WORLD = "plan_world"
MODEL = "wamv"
IMG = (f"/world/{WORLD}/model/{MODEL}/link/{MODEL}/base_link/sensor/"
       f"front_camera_sensor/image")
SET = f"/world/{WORLD}/set_pose"
BEACON = (50.0, 104.0)   # port_1 信标(绿)

def quat(yaw):
    return 0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)

def set_pose(x, y, yaw_deg):
    yaw = math.radians(yaw_deg)
    qz, qw = math.sin(yaw/2), math.cos(yaw/2)
    req = (f'name: "{MODEL}" position {{ x: {x} y: {y} z: 0.0 }} '
           f'orientation {{ x: 0 y: 0 z: {qz:.6f} w: {qw:.6f} }}')
    subprocess.run(["gz","service","-s",SET,"--reqtype","gz.msgs.Pose",
                    "--reptype","gz.msgs.Boolean","--timeout","4000","--req",req],
                   capture_output=True, timeout=8)

def grab():
    p = subprocess.Popen(["gz","topic","-e","-t",IMG,"--json-output"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                         text=True, bufsize=1)
    try:
        for line in p.stdout:
            m = json.loads(line)
            w, h = m["width"], m["height"]
            raw = base64.b64decode(m["data"])
            a = np.frombuffer(raw, dtype=np.uint8)
            try:
                return a[:w*h*3].reshape(h, w, 3).astype(np.int16)
            except Exception:
                return a[:w*h*4].reshape(h, w, 4)[:, :, :3].astype(np.int16)
    finally:
        p.terminate()

def color_count(img, kind="green"):
    R = img[...,0]; G = img[...,1]; B = img[...,2]
    if kind == "green":
        return int(((G > R*1.5) & (G > B*1.5) & (G > 90)).sum())
    return int(((R > G*1.5) & (R > B*1.3) & (R > 90)).sum())

def measure(n=10, save=None):
    """~5s 采样, 返回 全图绿 max/min/toggle 与 红 max。"""
    gs = []; rs = []
    for i in range(n):
        img = grab()
        gs.append(color_count(img, "green")); rs.append(color_count(img, "red"))
        if save is not None and i == 0:
            Image_from(img).save(save)
        if i < n-1: time.sleep(0.55)
    return max(gs), min(gs), max(gs)-min(gs), max(rs)

def Image_from(arr):
    from PIL import Image
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))

def dist(x, y):
    return math.hypot(BEACON[0]-x, BEACON[1]-y)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--distance-y", type=float, nargs="*", default=[94,84,74,64,54,44],
                    help="船所在 y(正对 port_1, x=50), 对应到信标距离约 104-y")
    ap.add_argument("--offset", type=float, nargs="*", default=[0,15,30,45],
                    help="20m 处附加偏航角(度)")
    args = ap.parse_args()

    print(f"{'情形':24s} {'距离':>6s} {'green max/min/差':>18s} {'red max':>7s}   判定", flush=True)
    rows = []
    # 1) 正对距离扫描 (x=50, 朝 port_1)
    for y in args.distance_y:
        set_pose(50.0, y, 90.0); time.sleep(0.8)
        mx, mn, tg, rm = measure(save=f"/tmp/boat_cam_{int(104-y)}m.png")
        d = dist(50.0, y)
        ok = "✅可辨" if tg >= 60 else ("◐弱" if tg >= 25 else "❌不可辨")
        print(f"{'正对 d≈%.0fm' % d:24s} {d:6.1f} {mx:5d}/{mn:5d}/{tg:5d}  {rm:7d}   {ok}", flush=True)
        rows.append(("正对", d, mx, mn, tg, rm, ok))
    # 2) d≈20m 偏航扫描
    for off in args.offset:
        set_pose(50.0, 84.0, 90.0+off); time.sleep(0.8)
        mx, mn, tg, rm = measure(save=f"/tmp/boat_cam_off{int(off)}.png")
        d = dist(50.0, 84.0)
        ok = "✅可辨" if tg >= 60 else ("◐弱" if tg >= 25 else "❌不可辨")
        print(f"{'d≈20m 偏%g°' % off:24s} {d:6.1f} {mx:5d}/{mn:5d}/{tg:5d}  {rm:7d}   {ok}", flush=True)
        rows.append((f"偏{off}°", d, mx, mn, tg, rm, ok))
    print("DONE", flush=True)

if __name__ == "__main__":
    main()
