# -*- coding: utf-8 -*-
"""
在线任务（去作弊版）：只用 自身位姿 + 三个港口位置 + lidar 感知，完成
  巡航 -> 在线避障(栅格A*) -> 接近港口 -> 相机判色 -> 找到绿港 -> 进港停稳

不使用 obstacles_meta / planner_output.json 里的真值走廊或障碍位置；
障碍圆完全来自 control/lidar_circles.py（lidar 点云在线拟合）。

用法（Gazebo 世界已启动）：
  python3 control/online_mission.py                 # 绿港由世界里的信标决定
  python3 control/online_mission.py --order 1,2,0   # 拜访顺序
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
WORLD_DIR = HERE.parent
sys.path.insert(0, str(WORLD_DIR))
sys.path.insert(0, str(HERE))

from planner import geometry as G                      # noqa: E402
from planner.occ_grid import Circle, OccupancyGrid     # noqa: E402
from planner.plan_corridors import dock_wall_rects     # noqa: E402
from lidar_circles import detect_circles               # noqa: E402
from lane_test import publish, detect_thrust_topics, wrap, clamp  # noqa: E402

WORLD = "plan_world"
MODEL = "wamv"
POSE_TOPIC = f"/world/{WORLD}/pose/info"
CAM_TOPIC = (f"/world/{WORLD}/model/{MODEL}/link/{MODEL}/base_link/sensor/"
             f"front_camera_sensor/image")
PORT_X = {0: 0.0, 1: 50.0, 2: 100.0}

# 实际运行轨迹记录(供一键脚本出图)
TRAJ: List[Tuple[float, float, float, float]] = []
EVENTS: List[dict] = []


def _rec(pose):
    """记录一次实际位姿 (t, x, y, yaw)。"""
    if pose is None:
        return
    TRAJ.append((time.time(), pose[0], pose[1], pose[3]))

# 小艇控制参数（已实测标定）
BASE = 18.0
GAIN = 15.0
YAW_DAMP = 15.0
MAX_DIFF = 14.0
SLEW = 30.0
LOOKAHEAD = 12.0
STEER_SIGN = -1.0
MAX_THRUST = 30.0


# --------------------------------------------------------------------------- #
def stop_all():
    """把两侧推进器清零(退出/复位/异常都要调用，避免船带着残留推力跑)。"""
    try:
        lt, rt, _ = detect_thrust_topics()
        for _ in range(3):
            publish(lt, 0.0); publish(rt, 0.0)
            time.sleep(0.05)
    except Exception:
        pass


def _sig_handler(signum, _frame):
    print(f"\n[SAFE] 收到信号 {signum} -> 停船退出", flush=True)
    stop_all()
    raise SystemExit(130)


signal.signal(signal.SIGINT, _sig_handler)
signal.signal(signal.SIGTERM, _sig_handler)


# --------------------------------------------------------------------------- #
def get_pose(timeout: float = 6.0):
    p = subprocess.Popen(["gz", "topic", "-e", "-t", POSE_TOPIC, "--json-output"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                         text=True, bufsize=1)
    t0 = time.time()
    try:
        for line in p.stdout:
            if time.time() - t0 > timeout:
                return None
            try:
                m = json.loads(line)
            except Exception:
                continue
            for q in m.get("pose", []):
                if q.get("name") == MODEL:
                    pos = q["position"]; o = q["orientation"]
                    return (pos["x"], pos["y"], pos["z"],
                            2.0 * math.atan2(o.get("z", 0.0), o.get("w", 1.0)))
    finally:
        p.terminate()
    return None


class SensorMap:
    """lidar 在线感知障碍圆(黑球)的累积地图。"""

    def __init__(self):
        self.lock = threading.Lock()
        # circles: (x, y, r, hits)；hits/last 为并行列表(命中数/最后看到时间)
        self.circles: List[Tuple[float, float, float, int]] = []
        self.hits: List[int] = []
        self.last: List[float] = []
        self.stop = False
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.thread.start()

    def _loop(self):
        while not self.stop:
            try:
                dets = detect_circles()
            except Exception:
                dets = []
            now = time.time()
            with self.lock:
                for (cx, cy, r, _n) in dets:
                    r = max(r, 1.6)          # 拟合半径偏小 -> 按黑球半径兜底
                    for k, (mx, my, mr, hit) in enumerate(self.circles):
                        if math.hypot(cx - mx, cy - my) < 3.0:
                            w = hit + 1
                            self.circles[k] = ((mx * hit + cx) / w,
                                               (my * hit + cy) / w,
                                               max(mr, r), w)
                            self.hits[k] = int(self.circles[k][3])
                            self.last[k] = now
                            break
                    else:
                        self.circles.append((cx, cy, r, 1))
                        self.hits.append(1); self.last.append(now)
                # 过期淘汰(>25s 没再看到)
                keep = [k for k in range(len(self.circles))
                        if now - self.last[k] < 25.0]
                self.circles = [self.circles[k] for k in keep]
                self.hits = [self.hits[k] for k in keep]
                self.last = [self.last[k] for k in keep]
            time.sleep(1.2)

    def snapshot(self, min_hits: int = 3):
        with self.lock:
            keep = [(self.circles[k], self.hits[k])
                    for k in range(len(self.circles)) if self.hits[k] >= min_hits]
            keep.sort(key=lambda kv: -kv[1])
            return [(c[0], c[1], c[2]) for (c, _h) in keep[:12]]


class Visualizer:
    """把在线规划路径/感知障碍以纯视觉模型注入 Gazebo（服务器端 create，必显示）。"""

    def __init__(self, world: str = WORLD):
        self.world = world

    @staticmethod
    def _call(service, reqtype, reptype, req):
        subprocess.run(["gz", "service", "-s", service, "--reqtype", reqtype,
                        "--reptype", reptype, "--timeout", "5000", "--req", req],
                       capture_output=True, text=True, timeout=20)

    def _remove(self, name):
        self._call(f"/world/{self.world}/remove", "gz.msgs.Entity",
                   "gz.msgs.Boolean", f'name: "{name}" type: MODEL')

    def _create(self, name, sdf):
        import re as _re
        one = _re.sub(r"\s+", " ", sdf).strip().replace('"', '\\"')
        self._call(f"/world/{self.world}/create", "gz.msgs.EntityFactory",
                   "gz.msgs.Boolean", f'sdf: "{one}"')

    def show_path(self, path, rgb=(0.1, 0.9, 1.0), step=2.0):
        """路径点每 step 米一个球 + 终点大球。"""
        if not path:
            return
        pts = [path[0]]
        acc = 0.0
        for a, b in zip(path[:-1], path[1:]):
            d = math.hypot(b[0]-a[0], b[1]-a[1])
            acc += d
            if acc >= step:
                pts.append(b); acc = 0.0
        if pts[-1] != path[-1]:
            pts.append(path[-1])
        mat = (f"<ambient>{rgb[0]} {rgb[1]} {rgb[2]} 1</ambient>"
               f"<diffuse>{rgb[0]} {rgb[1]} {rgb[2]} 1</diffuse>")
        vis = []
        for i, (x, y) in enumerate(pts):
            rr = 0.45 if i == len(pts)-1 else 0.18
            vis.append(f'<visual name="p{i}"><pose>{x:.2f} {y:.2f} 1.2 0 0 0</pose>'
                       '<geometry><sphere><radius>%.2f</radius></sphere></geometry>'
                       '<material>%s</material></visual>' % (rr, mat))
        sdf = ('<?xml version="1.0"?><sdf version="1.9">'
               '<model name="online_path"><static>true</static>'
               '<self_collide>false</self_collide><link name="l">'
               + "".join(vis) + '</link></model></sdf>')
        self._remove("online_path"); time.sleep(0.1); self._create("online_path", sdf)

    def show_obstacles(self, circles, n_ring=12):
        """每个感知圆画一圈红点(r+0.3)，直观显示它以为的障碍位置/大小。"""
        vis = []
        mat = ("<ambient>1 0.15 0.15 1</ambient><diffuse>1 0.15 0.15 1</diffuse>")
        k = 0
        for (x, y, r) in circles[:8]:      # 只画最可信的8个，控制SDF长度
            for j in range(n_ring):
                a = 2*math.pi*j/n_ring
                vis.append(f'<visual name="o{k}">'
                           f'<pose>{x+(r+0.3)*math.cos(a):.2f} '
                           f'{y+(r+0.3)*math.sin(a):.2f} 1.2 0 0 0</pose>'
                           '<geometry><sphere><radius>0.2</radius></sphere></geometry>'
                           f'<material>{mat}</material></visual>')
                k += 1
        if not vis:
            self._remove("online_obstacles"); return
        sdf = ('<?xml version="1.0"?><sdf version="1.9">'
               '<model name="online_obstacles"><static>true</static>'
               '<self_collide>false</self_collide><link name="l">'
               + "".join(vis) + '</link></model></sdf>')
        self._remove("online_obstacles"); time.sleep(0.1)
        self._create("online_obstacles", sdf)


# --------------------------------------------------------------------------- #
def choose_checkpoint(px, circles, base_dy=26.0, cand=(20.0, 26.0, 32.0),
                      min_clear=5.0):
    """动态选择判色检查点：在港口正南若干候选距离里，挑一个离感知障碍最远的点。

    返回 ((x, y), 净距m, 用了哪个dy)。净距 = 到最近感知圆面的距离。
    """
    x = min(max(px, 8.0), 92.0)
    best = None
    for dy in cand:
        y = 100.0 - dy
        clr = min((math.hypot(x - cx, y - cy) - r for (cx, cy, r) in circles),
                  default=99.0)
        # 先保证净距达标，其次优先"最空"的点，再考虑离基准距离近
        ok = clr >= min_clear
        score = (1 if ok else 0, round(clr, 1), -abs(dy - base_dy))
        if best is None or score > best[0]:
            best = (score, (x, y), clr, dy)
    return best[1], round(best[2], 2), best[3]


def plan_path(start, goal, circles, infl=3.0, cell=1.0):
    """栅格 A*：感知圆 + 已知码头墙 都是障碍；不含任何真值障碍信息。"""
    grid = OccupancyGrid((0, 0, 100, 100), cell=cell, inflation=infl,
                         soft_margin=1.0, soft_cost=2.0)
    grid.add_circles([Circle(x, y, r + 0.2) for (x, y, r) in circles])
    for px in PORT_X.values():
        for poly in dock_wall_rects(px):
            xs = [p[0] for p in poly]; ys = [p[1] for p in poly]
            i0, j0 = grid.world_to_cell(min(xs) - infl, min(ys) - infl)
            i1, j1 = grid.world_to_cell(max(xs) + infl, max(ys) + infl)
            for j in range(max(0, j0), min(grid.ny - 1, j1) + 1):
                for i in range(max(0, i0), min(grid.nx - 1, i1) + 1):
                    c = grid.cell_to_world(i, j)
                    if G.dist_point_poly(c, poly) < infl:
                        grid.blocked[j, i] = True
    # 起点周围清空(船自身位置一定可走)，避免感知圆/膨胀把起点压住
    si, sj = grid.world_to_cell(*start)
    rad = int(4.0 / grid.cell)
    for j in range(max(0, sj - rad), min(grid.ny - 1, sj + rad) + 1):
        for i in range(max(0, si - rad), min(grid.nx - 1, si + rad) + 1):
            grid.blocked[j, i] = False
    return grid.astar(start, goal)


def path_blocked(path, circles, clearance=2.6) -> bool:
    """当前路径是否被(新)感知圆挡住 -> 需要重规划。"""
    if not path or len(path) < 2:
        return True
    for (cx, cy, r) in circles:
        for a, b in zip(path[:-1], path[1:]):
            if G.dist_point_seg((cx, cy), a, b) < r + clearance:
                return True
    return False


def smooth_path(path, circles, clearance=2.6, step=1.5):
    """视线简化 + 等距重采样，减少 A* 锯齿。"""
    try:
        from planner.occ_grid import shortcut_circles
        sp = shortcut_circles(path, [Circle(x, y, r) for (x, y, r) in circles],
                              clearance)
    except Exception:
        sp = path
    return G.resample_polyline(G.remove_dup(sp), step)


def avoid_escape(goal, circles, pose, seconds=2.0, base=6.0):
    """A* 无解时：低速"排斥+朝目标"避让一小段，绝不直冲障碍。

    排斥来自 12m 内的感知圆；若附近没有障碍，则朝目标小推力前进。
    """
    ltopic, rtopic, _ = detect_thrust_topics()
    x, y, _z, yaw = pose
    t0 = time.time()
    while time.time() - t0 < seconds:
        pose = get_pose()
        if pose is None:
            time.sleep(0.15); continue
        _rec(pose)
        x, y, _z, yaw = pose
        rx = ry = 0.0
        for (cx, cy, r) in circles:
            dx, dy = x - cx, y - cy
            d = math.hypot(dx, dy)
            if d < r + 6.0 and d > 1e-6:
                w = (r + 6.0 - d) / (r + 6.0)
                rx += dx / d * w * 3.0
                ry += dy / d * w * 3.0
        gx, gy = goal[0] - x, goal[1] - y
        gd = max(1e-6, math.hypot(gx, gy))
        tx, ty = gx / gd + rx, gy / gd + ry
        psi = math.atan2(ty, tx)
        err = wrap(psi - yaw)
        # 温和转向 + 阻尼，避免兜底时自己转圈
        diff = clamp(STEER_SIGN * (8.0 * err), -5.0, 5.0)
        publish(ltopic, base - diff, MAX_THRUST)
        publish(rtopic, base + diff, MAX_THRUST)
        print(f"    [ESCAPE] pos=({x:6.2f},{y:6.2f}) 排斥=({rx:+.2f},{ry:+.2f}) "
              f"ψ={math.degrees(psi):6.1f}° err={math.degrees(err):5.1f}°", flush=True)
        time.sleep(0.12)
    for t in (ltopic, rtopic):
        publish(t, 0.0, MAX_THRUST)
    time.sleep(0.2)


def nearest_idx(path, x, y, i0=0):
    best, bi = 1e18, i0
    for i in range(i0, len(path)):
        d = (path[i][0] - x) ** 2 + (path[i][1] - y) ** 2
        if d < best:
            best, bi = d, i
    return bi


def lookahead_point(path, i, x, y, la):
    while i < len(path) - 1:
        if math.hypot(path[i][0] - x, path[i][1] - y) >= la:
            break
        i += 1
    return path[i]


def follow_to(goal, sensor: SensorMap, max_time=240.0, stop_radius=3.0,
              slow=False, verbose=True, viz: Optional[Visualizer] = None,
              infl_list=(3.0, 2.4, 1.8)):
    """在线跟随：每 2s 用最新感知地图重规划，纯跟随 -> 到 goal 附近。"""
    ltopic, rtopic, _ = detect_thrust_topics()
    t0 = time.time()
    path = None; last_plan = 0.0; i_prog = 0; diff_prev = 0.0
    v_sm = 0.0; prev = None; prev_pose_t = None; prev_yaw = None
    while time.time() - t0 < max_time:
        pose = get_pose()
        if pose is None:
            time.sleep(0.2); continue
        _rec(pose)
        x, y, _z, yaw = pose
        dist_goal = math.hypot(goal[0] - x, goal[1] - y)
        now = time.time()
        if dist_goal < stop_radius:
            publish(ltopic, 0.0); publish(rtopic, 0.0)
            return True
        # 滞回重规划：路径被挡 或 每 6s；无解时逐级降低膨胀再试
        circles = sensor.snapshot(min_hits=3)
        if path is None or path_blocked(path, circles) or now - last_plan > 6.0:
            p = None; used_infl = None
            for infl in infl_list:
                p = plan_path((x, y), goal, circles, infl=infl)
                if p is not None:
                    used_infl = infl; break
            if p is None:
                print("[PLAN] A* 无解(膨胀已降到1.8m) -> 低速排斥避让", flush=True)
                avoid_escape(goal, circles, (x, y, 0, yaw))
                last_plan = now
                continue
            p = smooth_path(p, circles, clearance=max(1.6, used_infl - 0.4))
            path = p; last_plan = now
            if viz is not None:
                try:
                    viz.show_path(path)
                    viz.show_obstacles(circles)
                except Exception as e:
                    print(f"[VIZ] 显示失败: {e}", flush=True)
            if verbose:
                print(f"[PLAN] 感知障碍 {len(circles)} 个 -> 路径 {len(path)} 点, "
                      f"目标({goal[0]:.0f},{goal[1]:.0f})", flush=True)
        i_prog = nearest_idx(path, x, y, max(0, i_prog - 5))
        la = LOOKAHEAD
        if dist_goal < 25.0:
            la = max(4.0, LOOKAHEAD * dist_goal / 25.0)
        lp = lookahead_point(path, i_prog, x, y, la)
        err = wrap(math.atan2(lp[1] - y, lp[0] - x) - yaw)
        # 速度/推力
        if slow or dist_goal < 22.0:
            base = clamp(6.0 + 0.6 * dist_goal, 4.0, BASE)
        else:
            base = BASE
        base *= max(0.35, 1.0 - 0.6 * min(abs(err) / 1.0, 1.0))
        # 角速度阻尼(用位姿差分估计)，抑制"转过头"
        yaw_rate = 0.0
        if prev_yaw is not None and prev_pose_t is not None:
            dt_p = max(1e-3, now - prev_pose_t)
            yaw_rate = wrap(yaw - prev_yaw) / dt_p
        diff_cmd = STEER_SIGN * (GAIN * err - YAW_DAMP * yaw_rate)
        prev_pose_t, prev_yaw = now, yaw
        diff_cmd = clamp(diff_cmd, -MAX_DIFF, MAX_DIFF)
        diff = clamp(diff_cmd, diff_prev - SLEW * 0.08, diff_prev + SLEW * 0.08)
        diff_prev = diff
        fl = clamp(base - diff, -MAX_THRUST, MAX_THRUST)
        fr = clamp(base + diff, -MAX_THRUST, MAX_THRUST)
        publish(ltopic, fl, MAX_THRUST); publish(rtopic, fr, MAX_THRUST)
        if verbose and now - t0 > 0:
            spd = 0.0
            if prev:
                dt = now - prev[0]
                if dt > 0:
                    spd = math.hypot(x - prev[1], y - prev[2]) / dt
            prev = (now, x, y)
            print(f"  t={now-t0:5.1f} pos=({x:6.2f},{y:6.2f}) yaw={math.degrees(yaw):6.1f} "
                  f"err={math.degrees(err):6.1f} L={fl:5.1f} R={fr:5.1f} d={dist_goal:5.1f} "
                  f"sensed={len(sensor.snapshot(2))}", flush=True)
        time.sleep(0.08)
    publish(ltopic, 0.0); publish(rtopic, 0.0)
    return False


# --------------------------------------------------------------------------- #
def grab_image():
    p = subprocess.Popen(["gz", "topic", "-e", "-t", CAM_TOPIC, "--json-output"],
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


def align_to_bearing(target_yaw, tol_deg=8.0, timeout=30.0, thrust=12.0):
    """原地转向到目标艏向(差速)，用于判色前对准港口。"""
    ltopic, rtopic, _ = detect_thrust_topics()
    t0 = time.time()
    while time.time() - t0 < timeout:
        pose = get_pose()
        if pose is None:
            time.sleep(0.2); continue
        _rec(pose)
        err = wrap(target_yaw - pose[3])
        if abs(math.degrees(err)) < tol_deg:
            publish(ltopic, 0.0); publish(rtopic, 0.0)
            time.sleep(0.5)
            publish(ltopic, 0.0); publish(rtopic, 0.0)
            return True
        # 左推力大 -> yaw 增大
        if err > 0:
            publish(ltopic, thrust, MAX_THRUST); publish(rtopic, -thrust, MAX_THRUST)
        else:
            publish(ltopic, -thrust, MAX_THRUST); publish(rtopic, thrust, MAX_THRUST)
        time.sleep(0.08)
    publish(ltopic, 0.0); publish(rtopic, 0.0)
    return False


def check_port_color(n=13):
    """采 ~7s 覆盖 3s亮/2s灭；统计画面中上部，返回 green/red/unknown。"""
    gs, rs = [], []
    for i in range(n):
        img = grab_image()
        h, w = img.shape[:2]
        img = img[int(0.05*h):int(0.80*h), int(0.20*w):int(0.80*w)]
        R, G, B = img[..., 0], img[..., 1], img[..., 2]
        gs.append(int(((G > R * 1.5) & (G > B * 1.5) & (G > 90)).sum()))
        rs.append(int(((R > G * 1.5) & (R > B * 1.3) & (R > 90)).sum()))
        if i < n - 1:
            time.sleep(0.55)
    gt, rt = max(gs) - min(gs), max(rs) - min(rs)
    if max(gt, rt) < 20:
        color = "unknown"          # 没看到灯 -> 不能当红灯下结论
    else:
        color = "green" if gt > rt else "red"
    return color, dict(green_toggle=gt, red_toggle=rt,
                       green_max=max(gs), red_max=max(rs))


# --------------------------------------------------------------------------- #
def _brake_to_stop(timeout=8.0, v_eps=0.06):
    """反向小推力刹车到基本静止(用船体系前向位移判断运动方向)。"""
    lt, rt, _ = detect_thrust_topics()
    prev = None; t0 = time.time()
    while time.time() - t0 < timeout:
        pose = get_pose()
        if pose is None:
            time.sleep(0.1); continue
        _rec(pose)
        now = time.time(); v = 0.0; fx = 0.0
        if prev is not None:
            dt = max(1e-3, now - prev[0])
            v = math.hypot(pose[0] - prev[1], pose[1] - prev[2]) / dt
            fx = (math.cos(pose[3]) * (pose[0] - prev[1])
                  + math.sin(pose[3]) * (pose[1] - prev[2]))
        if v < v_eps:
            publish(lt, 0.0); publish(rt, 0.0); return True
        cmd = -7.0 if fx > 0 else 7.0      # 与前进方向相反 -> 刹车
        publish(lt, cmd, MAX_THRUST); publish(rt, cmd, MAX_THRUST)
        prev = (now, pose[0], pose[1]); time.sleep(0.08)
    publish(lt, 0.0); publish(rt, 0.0)
    return False



# ---- 泊位几何(与 port_berth/model.sdf 一致) ----
BERTH_HALF_W = 0.75          # 内净半宽 (1.5/2)
BERTH_Y0, BERTH_Y1 = 99.0, 101.0   # 内深度方向 (世界系, 开口朝南)
BOAT_LEN, BOAT_WID = 1.25, 0.87    # 略放大一点做安全判定


def _boat_corners(x, y, yaw):
    c, s_ = math.cos(yaw), math.sin(yaw)
    out = []
    for dx in (-BOAT_LEN / 2.0, BOAT_LEN / 2.0):
        for dy in (-BOAT_WID / 2.0, BOAT_WID / 2.0):
            out.append((x + c * dx - s_ * dy, y + s_ * dx + c * dy))
    return out


def _is_fully_docked(pose, px, margin=0.04, yaw_tol_deg=15.0):
    """整条船(四个角)都在泊位内 + 艏向基本正 -> 才算入港成功。"""
    if pose is None:
        return False
    x, y, _z, yaw = pose
    for (cx, cy) in _boat_corners(x, y, yaw):
        if not (px - BERTH_HALF_W + margin <= cx <= px + BERTH_HALF_W - margin):
            return False
        if not (BERTH_Y0 + margin <= cy <= BERTH_Y1 - margin):
            return False
    if abs(math.degrees(wrap(yaw - math.pi / 2.0))) > yaw_tol_deg:
        return False
    return True


def _hold_zero(seconds=1.5):
    """只把推力置零并等待(不判断运动方向，避免误刹车)。"""
    lt, rt, _ = detect_thrust_topics()
    t0 = time.time()
    while time.time() - t0 < seconds:
        publish(lt, 0.0); publish(rt, 0.0)
        time.sleep(0.1)


def _align_fine(target=math.pi / 2.0, tol_deg=2.5, timeout=25.0, thrust=6.0):
    """原地微调艏向(带角速度阻尼)，用于入港前摆正。"""
    lt, rt, _ = detect_thrust_topics()
    t0 = time.time(); prev = None
    while time.time() - t0 < timeout:
        pose = get_pose()
        if pose is None:
            time.sleep(0.1); continue
        _rec(pose)
        now = time.time(); yaw_rate = 0.0
        if prev is not None:
            dt = max(1e-3, now - prev[0])
            yaw_rate = wrap(pose[3] - prev[1]) / dt
        prev = (now, pose[3])
        err = wrap(target - pose[3])
        if abs(math.degrees(err)) < tol_deg and abs(yaw_rate) < 0.15:
            publish(lt, 0.0); publish(rt, 0.0)
            time.sleep(0.4); publish(lt, 0.0); publish(rt, 0.0)
            return True
        cmd = clamp(STEER_SIGN * (10.0 * err - 6.0 * yaw_rate), -thrust, thrust)
        publish(lt, -cmd, MAX_THRUST); publish(rt, cmd, MAX_THRUST)
        time.sleep(0.08)
    publish(lt, 0.0); publish(rt, 0.0)
    return False


def _creep_in(px, stop_y, max_time=120.0, lat_abort=0.30, wall_y=97.0,
              base=8.0, v_max=0.9):
    """最简可靠的入港：恒定小推力直线进入 + 到位急停(带反推脉冲)。

    窄泊位(1.5m)下比"低速速度闭环"更稳：推力恒定、只做艏向保持；
    位置到 stop_y 立即停车 + 反推脉冲消惯性。偏太多立即中止倒出。
    """
    lt, rt, _ = detect_thrust_topics()
    t0 = time.time(); prev = None; diff_prev = 0.0
    while time.time() - t0 < max_time:
        pose = get_pose()
        if pose is None:
            time.sleep(0.1); continue
        _rec(pose)
        x, y, _z, yaw = pose
        now = time.time(); v = 0.0; yaw_rate = 0.0
        if prev is not None:
            dt = max(1e-3, now - prev[0])
            dx, dy = x - prev[1], y - prev[2]
            v = (math.cos(yaw) * dx + math.sin(yaw) * dy) / dt
            yaw_rate = wrap(yaw - prev[3]) / dt
        prev = (now, x, y, yaw)
        e = x - px

        if y < stop_y - 18.0 or abs(e) > 1.0:
            publish(lt, 0.0); publish(rt, 0.0)
            print(f"    [DOCK] 位置异常(y={y:.1f}, e={e:+.2f}) -> lost", flush=True)
            return "lost"
        if y > wall_y and abs(e) > lat_abort:
            publish(lt, 0.0); publish(rt, 0.0)
            print(f"    [DOCK] 口部横向偏差 {e:+.2f}m > {lat_abort} -> 退出重试", flush=True)
            return "too_far_off"

        # 到位: 急停 + 反推脉冲
        # 整条船都进到泊位内 -> 立即停车(不再调整)
        if _is_fully_docked(pose, px):
            publish(lt, 0.0); publish(rt, 0.0)
            time.sleep(0.25)
            for _ in range(6):
                publish(lt, -3.5); publish(rt, -3.5); time.sleep(0.1)
            publish(lt, 0.0); publish(rt, 0.0)
            print(f"    [DOCK] 整船已入泊位(pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}) -> 停车",
                  flush=True)
            return "ok"
        if y >= stop_y:
            publish(lt, 0.0); publish(rt, 0.0)
            time.sleep(0.2)
            for _ in range(8):
                publish(lt, -4.0); publish(rt, -4.0); time.sleep(0.1)
            publish(lt, 0.0); publish(rt, 0.0)
            print(f"    [DOCK] 到位急停 pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}", flush=True)
            return "ok"

        err = wrap(math.pi / 2.0 - yaw)        # 只保持艏向(保持直线)
        diff = clamp(STEER_SIGN * (12.0 * err - 8.0 * yaw_rate),
                     diff_prev - 2.0, diff_prev + 2.0)
        diff = clamp(diff, -4.0, 4.0)
        diff_prev = diff
        if y > wall_y or v > v_max:            # 口部/速度过大时收油
            fl = clamp(base - diff, -6, 6); fr = clamp(base + diff, -6, 6)
        else:
            fl = clamp(base - diff, -8, 10); fr = clamp(base + diff, -8, 10)
        publish(lt, fl, MAX_THRUST); publish(rt, fr, MAX_THRUST)
        print(f"    [DOCK] pos=({x:6.2f},{y:6.2f}) e={e:+.2f}m yaw={math.degrees(yaw):6.1f} "
              f"v={v:4.2f} L={fl:5.1f} R={fr:5.1f}", flush=True)
        time.sleep(0.08)
    publish(lt, 0.0); publish(rt, 0.0)
    return "timeout"


def _back_out(px, target_y, max_time=25.0):
    """沿 -Y 温和倒车退出泊位(保持艏向 90°)，限时防止跑飞。"""
    lt, rt, _ = detect_thrust_topics()
    t0 = time.time(); prev = None
    while time.time() - t0 < max_time:
        pose = get_pose()
        if pose is None:
            time.sleep(0.1); continue
        _rec(pose)
        x, y, _z, yaw = pose
        if y < target_y:
            publish(lt, 0.0); publish(rt, 0.0)
            return True
        yaw_rate = 0.0
        if prev is not None:
            dt = max(1e-3, time.time() - prev[0])
            yaw_rate = wrap(yaw - prev[1]) / dt
        prev = (time.time(), yaw)
        err = wrap(math.pi / 2.0 - yaw)
        diff = clamp(STEER_SIGN * (14.0 * err - 8.0 * yaw_rate), -5.0, 5.0)
        base = -5.0                              # 温和倒车
        publish(lt, clamp(base - diff, -10, 10), MAX_THRUST)
        publish(rt, clamp(base + diff, -10, 10), MAX_THRUST)
        time.sleep(0.1)
    publish(lt, 0.0); publish(rt, 0.0)
    return False


def _verify_docked(px, stop_y):
    pose = get_pose()
    if pose is None:
        return False
    x, y, _z, yaw = pose
    dyaw = abs(math.degrees(wrap(yaw - math.pi / 2.0)))
    ok = (abs(x - px) < 0.25) and (y > stop_y - 0.4) and (dyaw < 12.0)
    print(f"[DOCK] 校验: x偏差={x-px:+.2f}m y={y:.2f} yaw偏差={dyaw:.1f}° -> "
          f"{'OK' if ok else '不通过'}", flush=True)
    return ok


def final_dock(px, pre_y=93.0, stop_y=100.15, max_attempts=3):
    """专用靠泊：停稳 -> 对正 -> 低速直进 -> 一旦进港立即停车结束。

    规则(用户要求)：已经真正进港(口内 y>=99.2 且基本居中)就停下，不再退出调整；
    只有在"还没进港"的情况下失败(偏太多/超时)才倒出重试。
    """
    print("[DOCK] 专用入港控制: 停稳->对正->低速直进->进港即停", flush=True)
    for attempt in range(1, max_attempts + 1):
        print(f"[DOCK] ---- 第 {attempt}/{max_attempts} 次 ----", flush=True)
        _hold_zero(1.5)
        _align_fine(math.pi / 2.0, tol_deg=2.5)
        r = _creep_in(px, stop_y)
        pose = get_pose()
        if pose:
            x, y, _z, yaw = pose
            inside = _is_fully_docked(pose, px)
            print(f"[DOCK] 结果={r} pos=({x:.2f},{y:.2f}) e={x-px:+.2f} "
                  f"yaw={math.degrees(yaw):.1f}° 进港={inside}", flush=True)
        else:
            inside = False
        if r == "ok" or inside:
            stop_all()
            print("[DOCK] ✅ 已入港并停车(不再调整)", flush=True)
            return True
        if r == "lost":
            print("[DOCK] 船已偏离对中区 -> 停止尝试", flush=True)
            stop_all(); return False
        print(f"[DOCK] 第{attempt}次未进港({r}) -> 倒出到 y<{pre_y:.0f} 重试", flush=True)
        _back_out(px, pre_y)
    stop_all()
    print("[DOCK] ❌ 多次尝试仍未入港(已停在泊位外)", flush=True)
    return False


def run_mission(order, checkpoint_dy: float = 26.0, use_viz: bool = True) -> dict:
    """执行在线任务，返回结果(含实际轨迹/判色记录/最终感知地图)。"""
    global TRAJ, EVENTS
    TRAJ = []; EVENTS = []
    stop_all()                       # 开跑前先确保推力为0
    sensor = SensorMap(); sensor.start()
    viz = Visualizer() if use_viz else None
    print("[MISSION] 只用位姿+港口位置+lidar 感知; 不读障碍/走廊真值")
    print(f"[MISSION] 拜访顺序: {order}  (港口中心 y=100)")

    result = {"success": False, "dock_port": None, "colors": {},
              "order": list(order), "traj": [], "events": [], "sensed": []}
    for idx in order:
        px = PORT_X[idx]
        circles_now = sensor.snapshot(min_hits=3)
        cp, clr, dy_used = choose_checkpoint(px, circles_now, checkpoint_dy)
        print(f"\n[MISSION] === 前往 port_{idx} 检查点 ({cp[0]:.0f},{cp[1]:.0f})"
              f"  [距港口{dy_used:.0f}m, 离障碍{clr:.1f}m] ===", flush=True)
        ok_cp = follow_to(cp, sensor, max_time=200.0, stop_radius=3.0, viz=viz)
        if not ok_cp:
            cp2, clr2, dy2 = choose_checkpoint(px, sensor.snapshot(3),
                                               checkpoint_dy,
                                               cand=(46.0, 38.0, 32.0, 52.0))
            if abs(cp2[1] - cp[1]) > 1.0:
                print(f"[MISSION] 换检查点重试 ({cp2[0]:.0f},{cp2[1]:.0f}) "
                      f"[距港{dy2:.0f}m, 离障碍{clr2:.1f}m]", flush=True)
                ok_cp = follow_to(cp2, sensor, max_time=200.0, stop_radius=3.0, viz=viz)
                cp = cp2
        if not ok_cp:
            print(f"[MISSION] 未到达 port_{idx} 检查点, 跳过", flush=True)
            continue
        color = "unknown"; stats = {}
        for attempt in range(2):
            pose = get_pose()
            if pose:
                bearing = math.atan2(100.0 - pose[1], px - pose[0])
                print(f"[MISSION] 判色前转向正对 port_{idx}"
                      f"(目标艏向 {math.degrees(bearing):.1f}°)", flush=True)
                align_to_bearing(bearing)
            print(f"[MISSION] 停车判色(约7s, 第{attempt+1}次)...", flush=True)
            time.sleep(1.0)
            color, stats = check_port_color()
            print(f"[COLOR] port_{idx} 判定 = {color.upper()}   {stats}", flush=True)
            if color != "unknown":
                break
            if attempt == 0:
                print("[MISSION] 未测到灯 -> 靠近6m 再试一次", flush=True)
                follow_to((px, 100.0 - checkpoint_dy + 6.0), sensor,
                          max_time=60.0, stop_radius=2.5, slow=True, viz=viz)
        result["colors"][idx] = color
        EVENTS.append({"port": idx, "color": color, "stats": stats,
                       "pose": list(get_pose() or (0, 0, 0, 0))})
        if color != "green":
            print(f"[MISSION] port_{idx} 非正确港口, 前往下一个", flush=True)
            continue
        print(f"[MISSION] port_{idx} 是绿港 -> 低速进港", flush=True)
        follow_to((px, 92.0), sensor, max_time=120.0, stop_radius=2.5,
                  slow=True, viz=viz, infl_list=(2.0, 1.6, 1.2))
        ok_dock = final_dock(px, pre_y=93.0, stop_y=100.15)
        if ok_dock:
            print(f"[REPORT] ✅ 已停入绿港 port_{idx}; 汇报灯色 = GREEN", flush=True)
            result["success"] = True; result["dock_port"] = idx
        else:
            print(f"[REPORT] ⚠ port_{idx} 判色为绿，但入港未完成", flush=True)
        break
    if not result["success"]:
        print("[MISSION] ❌ 三个港口都没找到绿港", flush=True)
    sensor.stop = True
    stop_all()
    result["sensed"] = sensor.snapshot(min_hits=2)
    result["traj"] = [list(p) for p in TRAJ]
    result["events"] = list(EVENTS)
    return result


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--order", default="0,1,2",
                    help="拜访顺序(港编号), 默认 0,1,2 = 最短扫掠")
    ap.add_argument("--checkpoint-dy", type=float, default=26.0,
                    help="判色检查点距港口中心的南向距离(m)")
    args = ap.parse_args()
    order = [int(v) for v in args.order.split(",") if v.strip()]
    res = run_mission(order, args.checkpoint_dy)
    return 0 if res["success"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        stop_all()
