# -*- coding: utf-8 -*-
"""
感知前端：lidar 近水平环 -> 世界系障碍圆(黑球拟合)。

只用传感器数据(/scan/points) + 船自身位姿，不读 obstacles_meta（真值只用于
离线校验/打分，可关掉）。

流程：
  1. 取 gpu_ray 点云（字段含 x/y/z/ring），只保留近水平环 ring∈7..10，
     去掉自身(近距)与水面/海底方向(z 异常)的点；
  2. 用船位姿(真值)把点变换到世界系；
  3. 空间聚类(2m 连接) -> 每簇点做最小二乘圆拟合(Kasa)；
  4. 半径合理(0.6..3.0m，黑球 r=1.6)的簇视为障碍圆输出。

用法：
  python3 control/lidar_circles.py                 # 连续输出检测结果
  python3 control/lidar_circles.py --frames 5      # 采 5 帧求平均/合并后输出
  python3 control/lidar_circles.py --validate      # 与 obstacles_meta 对比(仅校验)
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import subprocess
import time
from typing import List, Tuple

import numpy as np

WORLD = "plan_world"
MODEL = "wamv"
PC = (f"/world/{WORLD}/model/{MODEL}/link/{MODEL}/base_link/sensor/"
      f"lidar_sensor/scan/points")
POSE = f"/world/{WORLD}/pose/info"
RINGS = (7, 8, 9, 10)
SENSOR_Z = 0.60
MIN_CLUSTER_PTS = 8
LINK_DIST = 2.0          # 聚类连接距离(m)
RMIN, RMAX = 0.6, 3.0    # 可接受的障碍圆半径


def grab_json(topic: str, timeout_s: float = 5.0):
    p = subprocess.Popen(["gz", "topic", "-e", "-t", topic, "--json-output"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                         text=True, bufsize=1)
    t0 = time.time()
    try:
        for line in p.stdout:
            if time.time() - t0 > timeout_s:
                return None
            try:
                m = json.loads(line)
            except Exception:
                continue
            if isinstance(m, dict) and m:
                return m
    finally:
        p.terminate()
    return None


def boat_pose():
    m = grab_json(POSE)
    if not m:
        return None
    for p in m.get("pose", []):
        if p.get("name") == MODEL:
            pos = p.get("position", {}); ori = p.get("orientation", {})
            qz = ori.get("z", 0.0); qw = ori.get("w", 1.0)
            return (pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0),
                    2.0 * math.atan2(qz, qw))
    return None


def cloud_points():
    """返回 (世界系点列表 [(x,y,z),...])。"""
    pose = boat_pose()
    m = grab_json(PC)
    if not pose or not m:
        return []
    fields = {f.get("name"): f.get("offset", 0) for f in m.get("field", [])}
    ps = int(m.get("pointStep", 32))
    n = int(m.get("width", 0)) * int(m.get("height", 0))
    buf = np.frombuffer(base64.b64decode(m["data"]), dtype=np.uint8).reshape(n, ps)

    def col(name, dt=np.float32, size=4):
        off = fields.get(name)
        if off is None:
            return None
        return buf[:, off:off + size].copy().view(dt)[:, 0]

    x, y, z = col("x"), col("y"), col("z")
    ring = col("ring", np.uint16, 2)
    if x is None or ring is None:
        return []
    sel = (np.isin(ring, list(RINGS)) & np.isfinite(x) & np.isfinite(y)
           & np.isfinite(z))
    r_loc = np.hypot(x, y)
    sel &= (r_loc > 1.0) & (r_loc < 115.0)
    bx, by, bz, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for px, py, pz in zip(x[sel], y[sel], z[sel]):
        wz = bz + SENSOR_Z + float(pz)
        if not (-0.8 < wz < 3.5):      # 去掉海底/异常高度
            continue
        wx = bx + c * float(px) - s * float(py)
        wy = by + s * float(px) + c * float(py)
        if not (0.0 <= wx <= 100.0 and 0.0 <= wy <= 100.0):
            continue                    # 水外 = 噪声
        if math.hypot(wx - bx, wy - by) < 4.0:
            continue                    # 自身/近距杂波
        out.append((wx, wy, wz))
    return out


def cluster_points(pts: List[Tuple[float, float, float]]):
    """简单空间聚类(2m 连接)，返回簇列表。"""
    if not pts:
        return []
    cell = LINK_DIST
    grid = {}
    for i, p in enumerate(pts):
        grid.setdefault((int(p[0] // cell), int(p[1] // cell)), []).append(i)
    seen = [False] * len(pts)
    clusters = []
    for i in range(len(pts)):
        if seen[i]:
            continue
        stack = [i]; seen[i] = True; comp = []
        while stack:
            k = stack.pop(); comp.append(k)
            kx, ky = int(pts[k][0] // cell), int(pts[k][1] // cell)
            for gx in (kx - 1, kx, kx + 1):
                for gy in (ky - 1, ky, ky + 1):
                    for j in grid.get((gx, gy), ()):
                        if seen[j]:
                            continue
                        if math.hypot(pts[j][0] - pts[k][0],
                                      pts[j][1] - pts[k][1]) <= LINK_DIST:
                            seen[j] = True; stack.append(j)
        clusters.append(comp)
    return clusters


def cluster_shape_ok(pts) -> bool:
    """形状过滤：码头墙是细长条，球的弧段更"圆"。

    用 PCA：长轴/短轴比过大且绝对长度大的簇判为墙/结构，丢弃。
    """
    if len(pts) < MIN_CLUSTER_PTS:
        return False
    P = np.asarray([(p[0], p[1]) for p in pts], float)
    P = P - P.mean(axis=0)
    cov = np.cov(P.T) if len(P) > 1 else np.zeros((2, 2))
    w = np.linalg.eigvalsh(cov)
    w = np.sort(np.maximum(w, 1e-9))
    ratio = math.sqrt(w[1] / w[0])          # 长/短轴
    length = 2.0 * math.sqrt(w[1]) * 1.7    # 近似长轴尺寸
    if ratio > 3.0 and length > 3.0:
        return False                        # 细长 -> 墙/结构
    return True


def near_port(cx: float, cy: float) -> bool:
    """港口是已知结构，黑球不会在泊位内部；泊位内的拟合直接丢掉。"""
    for px in (0.0, 50.0, 100.0):
        if abs(cx - px) < 3.0 and cy > 97.5:
            return True
    return False


def fit_circle(pts) -> Tuple[float, float, float]:
    """Kasa 拟合：返回 (cx, cy, r)。"""
    P = np.asarray(pts, float)
    x, y = P[:, 0], P[:, 1]
    A = np.column_stack([x, y, np.ones_like(x)])
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    r = math.sqrt(max(sol[2] + cx * cx + cy * cy, 0.0))
    return float(cx), float(cy), float(r)


def detect_circles(verbose: bool = False):
    pts = cloud_points()
    circles = []
    for comp in cluster_points(pts):
        if len(comp) < MIN_CLUSTER_PTS:
            continue
        cpts = [pts[i] for i in comp]
        if not cluster_shape_ok(cpts):
            continue
        cx, cy, r = fit_circle(cpts)
        if RMIN <= r <= RMAX and not near_port(cx, cy):
            circles.append((cx, cy, r, len(comp)))
    if verbose:
        print(f"  点云={len(pts)} 簇拟合出障碍圆={len(circles)}")
        for c in circles:
            print(f"    ({c[0]:7.2f},{c[1]:7.2f}) r={c[2]:.2f} n={c[3]}")
    return circles


def merge_circles(all_frames, tol: float = 2.5):
    """多帧合并：中心距 < tol 视为同一障碍，取平均。"""
    merged = []
    for frame in all_frames:
        for (cx, cy, r, n) in frame:
            for k, (mx, my, mr, mcnt) in enumerate(merged):
                if math.hypot(cx - mx, cy - my) < tol:
                    w = mcnt + 1
                    merged[k] = ((mx * mcnt + cx) / w, (my * mcnt + cy) / w,
                                 (mr * mcnt + r) / w, w)
                    break
            else:
                merged.append((cx, cy, r, 1))
    return merged


def ground_truth_circles():
    """仅用于 --validate：读 obstacles_meta 做对比，规划/控制不使用。"""
    from pathlib import Path
    meta = json.loads((Path(__file__).resolve().parent.parent /
                       "obstacles_meta.json").read_text(encoding="utf-8"))
    out = []
    for ob in meta["obstacles"]:
        if ob.get("shape") == "ball":
            out.append((ob["x"], ob["y"], ob["radius_m"]))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--validate", action="store_true")
    args = ap.parse_args()
    allf = []
    for i in range(args.frames):
        cs = detect_circles(verbose=False)
        allf.append(cs)
        if cs or i == args.frames - 1:
            print(f"frame{i}: {len(cs)} 个障碍圆", flush=True)
        time.sleep(0.4)
    det = [c for c in merge_circles(allf) if c[3] >= 2]   # 至少出现2帧才算稳定
    print(f"合并后稳定障碍圆 {len(det)} 个(>=2帧):")
    for (cx, cy, r, cnt) in det:
        print(f"  ({cx:7.2f},{cy:7.2f}) r={r:.2f}  出现{cnt}帧")
    if args.validate:
        gt = ground_truth_circles()
        print(f"\n[校验] 真值黑球 {len(gt)} 个(仅比较, 规划不使用):")
        for (gx, gy, gr) in gt:
            best, bd = None, 1e9
            for (cx, cy, r, cnt) in det:
                d = math.hypot(cx - gx, cy - gy)
                if d < bd:
                    bd, best = d, (cx, cy, r)
            tag = "✅" if bd < 2.0 else "⚠"
            print(f"  {tag} 真值({gx:6.1f},{gy:6.1f}) r={gr}  最近检测="
                  + (f"({best[0]:.1f},{best[1]:.1f}) r={best[2]:.2f} 距离{bd:.2f}m"
                     if best else "无"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
