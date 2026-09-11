# -*- coding: utf-8 -*-
"""
plan_world —— 入港走廊规划器 v0（离线，只生成路径+走廊，不做运动控制）。

流程（对三个港口各执行一次）：
  1. Informed RRT*  ：从起点 (50,0) 搜索到每个港口入港对接点 (px, lane_join_y)。
                      搜索时把 10 个障碍物按 (走廊半宽 + 安全裕度) 膨胀，
                      并把三座码头整体视为不可进入区域。
  2. 折线简化        ：贪心视线 shortcut，进一步减少转折点。
  3. 直线入港段      ：从对接点沿泊位轴线笔直开进港中心 (px, 100)，保证“船头正”。
  4. 三次 B 样条平滑 ：对“开放水域折线 + 直线入港段”做 s=0 三次 B 样条插值，
                      得到平滑、穿过各航路点、末段笔直朝北的中心线。
  5. 走廊生成        ：中心线 ± 走廊半宽 -> 左右边界（走廊多边形）。
  6. 校验            ：中心线/走廊离障碍与码头墙体距离 >= 走廊半宽；
                      校验不过则换随机种子重新规划（最多 attempts 次）。
  7. 输出            ：planner_output.json / .csv + planner_overview.png
                      （三条航道画在同一张俯视图里）。

用法（在 my_plan_world 目录下）：
    python3 -m planner.plan_corridors [--half-width 1.5] [--margin 1.0]
                                      [--lane-join-y 90] [--seed N] [--attempts 6]
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import geometry as G
from .bspline import fit_cubic_bspline
from .rrt_star import RRTStar, shortcut

HERE = Path(__file__).resolve().parent
WORLD_DIR = HERE.parent
META_JSON = WORLD_DIR / "obstacles_meta.json"
OUT_JSON = HERE / "planner_output.json"
OUT_CSV = HERE / "planner_output.csv"
OUT_PNG = HERE / "planner_overview.png"

# --------------------------------------------------------------------------- #
# 场景常量（与 plan_world.sdf / port_berth 模型一致）
# --------------------------------------------------------------------------- #
START = (50.0, 0.0)                       # WAM-V 出生点
WATER = (0.0, 0.0, 100.0, 100.0)          # 采样/规划边界 (xmin,ymin,xmax,ymax)
PORTS = [                                 # 三座港口中心（泊位中心），开口朝南
    {"name": "port_0", "px": 0.0},
    {"name": "port_1", "px": 50.0},
    {"name": "port_2", "px": 100.0},
]
PORT_Y = 100.0                            # 泊位中心 y
# 实战港口: 内净 1.5m(宽) x 2.0m(深)，墙厚 0.5m
MOUTH_Y = PORT_Y - 1.0                    # 泊位口内沿 y (=99.0)，开口朝南
BERTH_HALF_W = 0.75                       # 泊位内净宽半宽 (1.5/2)
WALL_HALF_W = 0.25                        # 墙厚一半 (0.5/2)
WALL_HALF_H = 1.25                        # 侧墙半长(≈内深2.0/2 + 余量)
BACK_OFF = 1.25                           # 后墙中心离泊位中心距离

DEFAULT_HALF_WIDTH = 0.6                  # 走廊半宽(总宽1.2m, 适配1.5m宽泊位)
DEFAULT_MARGIN = 2.0                      # 搜索时的额外安全裕度(中心线离障碍>=hw+margin)
DEFAULT_LANE_JOIN_Y = 90.0                # 入港直线段起点 y（对接点）
DEFAULT_MIN_RADIUS = 5.0                  # 硬性最小转弯半径约束(m)，<=0 表示关闭
WALL_MARGIN = 0.1                         # 港内贴墙额外裕度：中心线离墙 >= 半宽+WALL_MARGIN


def dock_search_rects(px: float) -> List[G.Poly]:
    """搜索时“码头整体不可进入”的外包矩形（含泊位+墙体+后墙）。"""
    # 世界系：外沿 x∈[px-1.25, px+1.25]，y∈[98.75, 101.5]
    return [G.rect_poly(px, 100.125, 1.25, 1.5)]


def dock_wall_rects(px: float) -> List[G.Poly]:
    """校验用码头墙体矩形（世界系），泊位内部不算障碍。

    新几何(实战): 内净 1.5(宽)x2.0(深)，墙厚0.5
      侧墙: x∈[px±0.75, px±1.25], y∈[98.75,101.25]
      后墙: y∈[101.0,101.5],      x∈[px±1.25]
    """
    side = [
        G.rect_poly(px + BERTH_HALF_W + WALL_HALF_W, PORT_Y,
                    WALL_HALF_W, WALL_HALF_H),
        G.rect_poly(px - BERTH_HALF_W - WALL_HALF_W, PORT_Y,
                    WALL_HALF_W, WALL_HALF_H),
    ]
    back = G.rect_poly(px, 101.25,
                       BERTH_HALF_W + WALL_HALF_W, WALL_HALF_W)
    return side + [back]


# --------------------------------------------------------------------------- #
# 走廊校验
# --------------------------------------------------------------------------- #
def validate_centerline(centerline: Sequence[G.Point],
                        obstacle_polys: Sequence[G.Poly],
                        wall_polys: Sequence[G.Poly],
                        hw: float,
                        wall_clearance: Optional[float] = None,
                        step: float = 0.5) -> Tuple[bool, float, float]:
    """校验中心线：离障碍 >= hw，离码头墙体 >= wall_clearance(默认 hw)。

    返回 (ok, min_obstacle_clearance, min_wall_clearance)。
    """
    if wall_clearance is None:
        wall_clearance = hw
    sampled = G.resample_polyline(centerline, step)
    min_obs = (min(G.dist_point_poly(p, poly)
                   for p in sampled for poly in obstacle_polys)
               if obstacle_polys else float("inf"))
    min_wall = (min(G.dist_point_poly(p, poly)
                    for p in sampled for poly in wall_polys)
                if wall_polys else float("inf"))
    ok = (min_obs >= hw - 1e-6) and (min_wall >= wall_clearance - 1e-6)
    return ok, min_obs, min_wall


def make_corridor(centerline: Sequence[G.Point], hw: float, step: float = 0.5):
    """由中心线生成走廊左右边界（画图/输出用）。"""
    dense = G.resample_polyline(centerline, step)
    left = G.offset_polyline(dense, hw, left=True)
    right = G.offset_polyline(dense, hw, left=False)
    return dense, left, right


# --------------------------------------------------------------------------- #
# 单港口规划
# --------------------------------------------------------------------------- #
def _fillet_turn(A, E, R, step=0.25):
    """入港方向 A->E，用半径 R 的圆弧转成朝北(90°)。

    返回 (T1, arc_pts)：
      - 已几乎朝北  -> (None, None)
      - 直段太短放不下 -> (None, [])
      - 正常        -> (T1, arc点列)  (arc 从 T1 出发、终点切向朝北)
    """
    hx, hy = E[0] - A[0], E[1] - A[1]
    L = math.hypot(hx, hy)
    if L < 1e-9:
        return None, []
    h = (hx / L, hy / L)
    cross = h[0]                 # h × north 的 z 分量(>0 左转, <0 右转)
    Delta = math.atan2(abs(cross), h[1])
    if Delta < 1e-3:             # 已几乎朝北，不需要圆弧
        return None, None
    lt = R * math.tan(Delta / 2.0)
    if lt > L - 1e-6:            # 入港直段太短
        return None, []
    T1 = (E[0] - h[0] * lt, E[1] - h[1] * lt)
    sigma = 1.0 if cross > 0 else -1.0     # +1 左转(逆时针), -1 右转(顺时针)
    left = (-h[1], h[0])                   # 航向左侧单位法向
    C = (T1[0] + sigma * R * left[0],
         T1[1] + sigma * R * left[1])      # 圆心在转弯内侧
    T2 = (E[0], E[1] + lt)
    if abs(G.dist_point_point(T2, C) - R) > 1e-6 * max(1.0, R):
        return None, []
    phi1 = math.atan2(T1[1] - C[1], T1[0] - C[0])
    phi2 = math.atan2(T2[1] - C[1], T2[0] - C[0])
    # 沿 sigma 方向取短弧角差
    base = math.atan2(math.sin(phi2 - phi1), math.cos(phi2 - phi1))
    if base * sigma < 0:
        dphi = base + sigma * 2.0 * math.pi
    else:
        dphi = base
    n_pts = max(2, int(abs(dphi) * R / step) + 1)
    pts = [(C[0] + R * math.cos(phi), C[1] + R * math.sin(phi))
           for phi in (phi1 + dphi * (i / n_pts)
                       for i in range(n_pts + 1))]
    return T1, pts


def _docking_fillet_skeleton(open_path, px, lane_join_y, R, step=0.25):
    """预制入港段：open_path 末段 + R 圆弧转北 + 沿轴直道到港中心。

    返回整条骨架点列(尚未做碰撞/曲率校验)；放不下圆弧返回 None。
    """
    E = (px, lane_join_y)
    A = open_path[-2]
    T1, arc = _fillet_turn(A, E, R, step)
    skel = [tuple(q) for q in open_path[:-1]]
    if T1 is None:
        if arc is None:                     # 已朝北：直接从 E 沿轴直道
            skel.append(E)
        else:                               # arc == [] 表示直段太短放不下
            return None
    else:
        skel.append(T1)
        skel.extend(arc)
    # 沿轴线到港中心
    y0 = skel[-1][1]
    yy = y0 + step
    while yy < PORT_Y - 1e-9:
        skel.append((px, yy))
        yy = min(yy + step, PORT_Y)
    if skel[-1][1] < PORT_Y - 1e-9:
        skel.append((px, PORT_Y))
    return G.remove_dup(skel)


DOCK_DIR_NAMES = ["center", "west", "east"]

def _open_candidate(px, obstacle_polys, hw, margin, lane_join_y,
                    min_radius, seed):
    """RRT(关闭 informed 收敛, 保多样)+shortcut+开阔平滑。返回候选或 None。"""
    search_obs = list(obstacle_polys)
    for q in PORTS:
        search_obs.extend(dock_search_rects(q["px"]))
    goal = (px, lane_join_y)
    pl = RRTStar(search_obs, clearance=hw + margin, bounds=WATER, seed=seed)
    res = pl.plan(START, goal, max_iterations=1200, improve_iters=60,
                  informed=False)
    if not res["found"]:
        return None
    path = list(res["path"])
    path[-1] = goal
    op = shortcut(path, search_obs, hw + margin)
    dense_open = G.resample_polyline(op, 2.0)
    base_s = 0.002 * len(dense_open)
    kappa_max = (1.0 / min_radius) if min_radius > 0 else 0.3
    for k in (0.0, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0):
        try:
            c = fit_cubic_bspline(dense_open, n_samples=2001,
                                  s=(base_s * k if k else 0.0))
        except Exception:
            continue
        dc = G.resample_polyline(c, 0.25)
        okc, _, _ = validate_centerline(dc, obstacle_polys, [], hw)
        if not okc:
            continue
        kk = G.curvature_from_pts(dc)
        if max(abs(x) for x in kk) > kappa_max + 1e-9:
            continue
        hh = G.sample_heading(dc)
        psi = math.degrees(math.atan2(sum(math.sin(a) for a in hh[-6:]),
                                      sum(math.cos(a) for a in hh[-6:])))
        return {"cl": dc, "op": op, "psi_deg": psi}
    return None


def _path_mean_dist(a, b, k=40):
    sa = G.resample_polyline(a, max(G.dist_point_point(a[0], a[-1]) / k, 0.1))
    sb = G.resample_polyline(b, max(G.dist_point_point(b[0], b[-1]) / k, 0.1))
    n = min(len(sa), len(sb))
    step = max(1, n // k)
    return (sum(G.dist_point_point(sa[i], sb[i]) for i in range(0, n, step))
            / max(1, len(range(0, n, step))))


def _collect_candidates(px, obstacle_polys, hw, margin, lane_join_y,
                        min_radius, n_cand=3, max_trials=25, seed0=2026):
    kept = []
    tried = 0
    for i in range(max_trials):
        if len(kept) >= n_cand:
            break
        tried += 1
        c = _open_candidate(px, obstacle_polys, hw, margin, lane_join_y,
                            min_radius, seed=seed0 + i + 1)
        if c is None:
            continue
        dup = any(abs(k["psi_deg"] - c["psi_deg"]) < 12 and
                  _path_mean_dist(k["cl"], c["cl"]) < 8.0 for k in kept)
        if not dup:
            c["seed"] = seed0 + i + 1
            kept.append(c)
    return kept, tried


def _dock_candidate_3(px, cl, obstacle_polys, hw, min_radius):
    """用三方向(正中/偏西/偏东两控制点)对接，返回最短可用或 None。"""
    walls = dock_wall_rects(px)
    kappa_max = (1.0 / min_radius) if min_radius > 0 else 0.3
    dirs = [("center", ((px, 92.0), (px, 96.0))),
            ("west", ((px - 4.0, 92.0), (px, 96.0))),
            ("east", ((px + 4.0, 92.0), (px, 96.0)))]
    best = None
    for name, (P1, P2) in dirs:
        ctrl = G.remove_dup([tuple(v) for v in cl] + [P1, P2,
                                                      (px, PORT_Y)])
        for k in (0.0, 1.0, 2.0, 4.0, 8.0):
            try:
                c = fit_cubic_bspline(ctrl, n_samples=2001,
                                      s=(0.002 * (len(ctrl) // 10) * k
                                         if k else 0.0))
            except Exception:
                continue
            dc = G.resample_polyline(c, 0.25)
            okc, mo, mw = validate_centerline(dc, obstacle_polys, walls, hw)
            if not okc:
                continue
            kk = G.curvature_from_pts(dc)
            mx = max(abs(x) for x in kk)
            if mx > kappa_max + 1e-9:
                continue
            L = sum(G.dist_point_point(a, b) for a, b in zip(dc[:-1], dc[1:]))
            item = {"dir": name, "P1": list(P1), "P2": list(P2),
                    "dense": dc, "mx": mx, "L": L, "mo": mo, "mw": mw}
            if best is None or L < best["L"]:
                best = item
    return best


def _mouth_metrics(pts, px):
    """泊位口(y=MOUTH_Y)处的航向角与横向偏移，用于“斜入港”实验报告。"""
    idx = min(range(len(pts)), key=lambda i: abs(pts[i][1] - MOUTH_Y))
    h = G.sample_heading(pts)
    return round(math.degrees(h[idx]), 1), round(pts[idx][0] - px, 3)


def _arc_heading(E0, psi_deg, theta_deg, R, step=0.25):
    """从起点(E0, 航向psi)用半径R圆弧转到航向theta。返回(arc_pts, end_pt)。

    转角≈0 时返回 ([], E0)。
    """
    psi = math.radians(psi_deg)
    th = math.radians(theta_deg)
    d = (th - psi + math.pi) % (2.0 * math.pi) - math.pi   # (-pi, pi]
    if abs(d) < 1e-3:
        return [], E0
    left = (-math.sin(psi), math.cos(psi))
    C = ((E0[0] + R * left[0], E0[1] + R * left[1]) if d > 0
         else (E0[0] - R * left[0], E0[1] - R * left[1]))
    vx, vy = E0[0] - C[0], E0[1] - C[1]
    n = max(2, int(abs(d) * R / step) + 1)
    pts = []
    for i in range(n + 1):
        ang = d * (i / n)
        ca, sa = math.cos(ang), math.sin(ang)
        pts.append((C[0] + ca * vx - sa * vy, C[1] + sa * vx + ca * vy))
    return pts, pts[-1]


def _inport_curve(px, delta, theta_deg, hw, kappa_max, step=0.1):
    """港内终端曲线：口部(px+delta, 96.5, heading theta) -> 港中心(px,100,90°)。

    用 x(y) 三次多项式；要求全程 |x-px| <= 2.5-hw 且曲率 <= kappa_max。
    返回点列或 None。
    """
    y0, y1 = MOUTH_Y, PORT_Y
    L = y1 - y0
    h = math.radians(theta_deg)
    if math.sin(h) <= 1e-6:
        return None
    slope = math.cos(h) / math.sin(h)          # dx/dy @口部
    x0 = px + delta
    A = 6.0 * (x0 + slope * L / 2.0 - px) / (L ** 3)
    B = -(A * L * L + slope) / L
    max_off = (BERTH_HALF_W - hw)
    pts = []
    kmax = 0.0
    t = 0.0
    while t <= L + 1e-9:
        x = A / 3.0 * t ** 3 + B / 2.0 * t * t + slope * t + x0
        if abs(x - px) > max_off + 1e-9:
            return None
        d1 = A * t * t + B * t + slope
        d2 = 2.0 * A * t + B
        k = abs(d2) / (1.0 + d1 * d1) ** 1.5
        kmax = max(kmax, k)
        if kmax > kappa_max + 1e-9:
            return None
        pts.append((x, y0 + t))
        t += step
    return pts


def _docking_library_skeleton(px, open_tail, psi_deg, theta_deg, R,
                              hw, kappa_max, step=0.25):
    """把一个库条目(口部朝向theta)接到开阔段末端(航向psi)。

    返回 (skeleton点列, delta_m) 或 None；delta=口部横向偏移(自动算出)。
    """
    E0 = open_tail[-1]
    arc, E2 = _arc_heading(E0, psi_deg, theta_deg, R, step)
    h = math.radians(theta_deg)
    uy = math.sin(h)
    if uy <= 1e-6 or E2[1] >= MOUTH_Y:
        return None
    ly = (MOUTH_Y - E2[1]) / uy                # 直线引航长度到口部
    ux = math.cos(h)
    Mx = E2[0] + ly * ux
    delta = Mx - px
    if abs(delta) > BERTH_HALF_W - hw:
        return None
    curve = _inport_curve(px, delta, theta_deg, hw, kappa_max)
    if curve is None:
        return None
    skel = [tuple(q) for q in open_tail]
    if arc:
        skel.extend(tuple(q) for q in arc[1:])
    # 直线引航(arc 后到口部)
    nlead = max(1, int(ly / step))
    for i in range(1, nlead + 1):
        t = ly * (i / nlead)
        skel.append((E2[0] + t * ux, E2[1] + t * uy))
    # 港内终端曲线(去重口部起点)
    skel.extend(tuple(q) for q in curve[1:])
    return G.remove_dup(skel), round(delta, 3)


def plan_one_port(px: float, obstacle_polys: Sequence[G.Poly],
                  hw: float, margin: float, lane_join_y: float,
                  min_radius: float, attempts: int, seed: int,
                  no_shortcut: bool = False,
                  docking_free: bool = False) -> Optional[Dict]:
    """单港口规划。

    min_radius   : 硬性最小转弯半径(m)。
    docking_free : True=方案甲(推荐)：开阔水域 RRT(可直连/shortcut) +
                   预制入港段(R_min 圆弧转北 + 沿轴直道进港中心)。
                   不再让 RRT 随机钻进窄港(慢)。
                   默认 False = 旧“直入港 + B 样条硬拉”模式(保留对照)。
    """
    search_clear = hw + margin
    kappa_max = (1.0 / min_radius) if (min_radius and min_radius > 0) \
        else float("inf")

    wall_polys = dock_wall_rects(px)
    last_err = ""
    best_relaxed = None      # 诊断用：碰撞通过但曲率超限的最优候选(仅 align 模式)
    for attempt in range(attempts):
        rng_seed = seed + attempt if seed is not None else None

        # 两种模式的搜索目标都在开阔水域(对接点)，可直连/shortcut
        goal = (px, lane_join_y)
        search_obstacles = list(obstacle_polys)
        for q in PORTS:
            search_obstacles.extend(dock_search_rects(q["px"]))
        planner = RRTStar(search_obstacles, clearance=search_clear,
                          bounds=WATER, seed=rng_seed)
        res = planner.plan(START, goal, max_iterations=2000,
                           improve_iters=250)
        if not res["found"]:
            last_err = "RRT* 未找到路径"
            continue

        path = list(res["path"])
        path[-1] = goal
        raw_path = list(path)
        open_path = raw_path if no_shortcut else shortcut(
            path, search_obstacles, search_clear)

        if docking_free:
            # ---------- 主版本:多样候选 + 三方向(两控制点)入港对接 ----------
            seed0 = seed if seed is not None else 2026
            cands, tried = _collect_candidates(
                px, obstacle_polys, hw, margin, lane_join_y, min_radius,
                n_cand=3, max_trials=25, seed0=seed0)
            if not cands:
                last_err = f"多样候选收集失败(试{tried}次无开阔可用路径)"
                return {"port": f"port_{int(px)}", "px": px,
                        "goal_xy": [px, PORT_Y], "success": False,
                        "attempts": attempts, "last_error": last_err,
                        "candidates": []}
            best = None
            chosen = -1
            for ci, c in enumerate(cands):
                it = _dock_candidate_3(px, c["cl"], obstacle_polys, hw,
                                       min_radius)
                if it is not None and (best is None or it["L"] < best["L"]):
                    best = it
                    chosen = ci
            if best is None:
                # 失败也返回候选，便于图上观察
                cand_list = [{"cl": [list(v) for v in c["cl"]],
                              "op": [list(v) for v in c["op"]],
                              "psi_deg": round(c["psi_deg"], 1),
                              "seed": c["seed"]} for c in cands]
                first = cands[0]
                res_d = {"port": f"port_{int(px)}", "px": px,
                         "goal_xy": [px, PORT_Y], "success": False,
                         "attempts": attempts,
                         "last_error": "三方向均无法对接(请查看候选)",
                         "candidates": cand_list,
                         "docking_mode": "free"}
                if first.get("cl"):
                    res_d["centerline"] = [list(v) for v in first["cl"]]
                return res_d

            cc = cands[chosen]
            dc = best["dense"]
            _, left, right = make_corridor(dc, hw)
            res_out = _build_result(
                px, goal, cc["op"], cc["op"],
                {"mode": f"diverse_seed{cc['seed']}_{best['dir']}", "s": 0.0,
                 "full": cc["op"], "dense": dc, "left": left, "right": right,
                 "max_curv": best["mx"], "length": best["L"],
                 "min_obs": best["mo"], "min_wall": best["mw"]},
                attempt + 1, min_radius)
            res_out["docking_mode"] = "free"
            res_out["dock_dir"] = best["dir"]
            res_out["dock_ctrl_pts"] = [best["P1"], best["P2"]]
            res_out["chosen_candidate_idx"] = chosen
            res_out["candidates"] = [
                {"cl": [list(v) for v in c["cl"]],
                 "op": [list(v) for v in c["op"]],
                 "psi_deg": round(c["psi_deg"], 1),
                 "seed": c["seed"]} for c in cands]
            mh, ml = _mouth_metrics(dc, px)
            res_out["mouth_heading_deg"] = mh
            res_out["mouth_lateral_offset_m"] = ml
            return res_out

        # ---------- 旧“直入港 + B 样条”模式(保留对照) ----------
        tail = []
        y = lane_join_y
        while y < PORT_Y - 1e-6:
            tail.append((px, y))
            y = min(y + 0.5, PORT_Y)
        tail.append((px, PORT_Y))
        tail = list(tail)
        dense_open = G.resample_polyline(open_path, 2.0)
        densified_full = list(dense_open) + tail
        shortcut_full = list(open_path) + tail
        base_s = 0.002 * len(densified_full)
        full_candidates = []
        for k in (0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0):
            full_candidates.append((f"smooth_k{k:g}", densified_full,
                                    base_s * k))
        full_candidates.append(("densified_s0", densified_full, 0.0))
        full_candidates.append(("shortcut_s0", shortcut_full, 0.0))

        valid = []
        relaxed = []
        for mode, full, s in full_candidates:
            try:
                centerline = fit_cubic_bspline(full, n_samples=1601, s=s)
            except ValueError as e:
                last_err = f"B 样条拟合失败({mode}): {e}"
                continue
            centerline = G.straighten_tail_to_axis(
                centerline, px, 92.0, ease=2.0)
            ok, min_obs, min_wall = validate_centerline(
                centerline, obstacle_polys, wall_polys, hw)
            if not ok:
                last_err = (f"校验失败({mode}) min_obs={min_obs:.2f} "
                            f"min_wall={min_wall:.2f}")
                continue
            dense, left, right = make_corridor(centerline, hw)
            curv = G.curvature_from_pts(dense)
            max_curv = max(abs(c) for c in curv)
            length = sum(G.dist_point_point(a, b)
                         for a, b in zip(dense[:-1], dense[1:]))
            item = {"mode": mode, "s": s, "full": full,
                    "centerline": centerline, "dense": dense,
                    "left": left, "right": right,
                    "max_curv": max_curv, "length": length,
                    "min_obs": min_obs, "min_wall": min_wall}
            if max_curv <= kappa_max + 1e-9:
                valid.append(item)
            else:
                relaxed.append(item)

        if valid:
            best = valid[0]
            res_out = _build_result(px, goal, open_path, raw_path, best,
                                    attempt + 1, min_radius)
            _attach_extras(res_out, px, best, docking_free)
            return res_out

        if relaxed:
            rb = min(relaxed, key=lambda v: v["max_curv"])
            if (best_relaxed is None or
                    rb["max_curv"] < best_relaxed["item"]["max_curv"]):
                best_relaxed = {"item": rb, "open_path": open_path,
                                "raw_path": raw_path}
            last_err = (f"曲率硬约束不满足: 本尝试最小转弯半径 R≈"
                        f"{1/rb['max_curv'] if rb['max_curv'] else float('inf'):.2f} m "
                        f"(要求 >= {min_radius} m)")
        else:
            last_err = "无候选通过碰撞校验"

    if best_relaxed is not None:
        rb = best_relaxed["item"]
        last_err += (f"；{attempts} 次尝试中最好仅达 R≈"
                     f"{1/rb['max_curv'] if rb['max_curv'] else float('inf'):.2f} m")
        res_out = _build_result(px, goal, best_relaxed["open_path"],
                                best_relaxed["raw_path"], rb,
                                attempts, min_radius)
        res_out["success"] = False
        res_out["diagnostic"] = True
        res_out["last_error"] = last_err
        _attach_extras(res_out, px, rb, docking_free)
        return res_out
    return {"port": f"port_{int(px)}", "px": px,
            "goal_xy": [px, PORT_Y], "success": False,
            "attempts": attempts, "last_error": last_err}


def _attach_extras(res_out, px, item, docking_free):
    res_out["docking_mode"] = "free" if docking_free else "align"
    res_out["min_wall_clearance_m"] = round(item["min_wall"], 3)
    if docking_free:
        mh, ml = _mouth_metrics(item["dense"], px)
        res_out["mouth_heading_deg"] = mh
        res_out["mouth_lateral_offset_m"] = ml
        if "dock_theta_deg" in item:
            res_out["dock_theta_deg"] = item["dock_theta_deg"]
            res_out["dock_delta_m"] = item["dock_delta_m"]


def _build_result(px: float, goal, open_path, raw_path, best: Dict,
                  attempts_used: int, min_radius: float) -> Dict:
    max_curv = best["max_curv"]
    r_achieved = (1.0 / max_curv) if max_curv > 1e-9 else None
    return {
        "port": f"port_{int(px)}",
        "px": px,
        "goal_xy": [px, PORT_Y],
        "lane_join_xy": list(goal),
        "smooth_mode": best["mode"],
        "smoothing_s": round(best["s"], 5),
        "success": True,
        "attempts": attempts_used,
        "open_waypoints": [list(q) for q in open_path],
        "rrt_raw_path": [list(q) for q in raw_path],
        "full_waypoints": [list(q) for q in best["full"]],
        "centerline": [list(q) for q in best["dense"]],
        "corridor_left": [list(q) for q in best["left"]],
        "corridor_right": [list(q) for q in best["right"]],
        "length_m": round(best["length"], 3),
        "max_curvature_1_per_m": round(max_curv, 6),
        "min_turn_radius_m": (round(r_achieved, 3) if r_achieved else None),
        "min_radius_constraint_m": min_radius,
        "min_obstacle_clearance_m": round(best["min_obs"], 3),
        "min_wall_clearance_m": round(best["min_wall"], 3),
        "last_error": "",
    }


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--half-width", type=float, default=DEFAULT_HALF_WIDTH,
                    help="走廊半宽(总宽=2x)，默认 1.5")
    ap.add_argument("--margin", type=float, default=DEFAULT_MARGIN,
                    help="搜索期中心线额外安全裕度，默认 1.0")
    ap.add_argument("--lane-join-y", type=float, default=DEFAULT_LANE_JOIN_Y,
                    help="入港直线段起点 y，默认 90")
    ap.add_argument("--min-radius", type=float, default=DEFAULT_MIN_RADIUS,
                    help=f"硬性最小转弯半径(m)，全程曲率<=1/该值；默认 {DEFAULT_MIN_RADIUS}；"
                         "<=0 关闭")
    ap.add_argument("--no-shortcut", action="store_true",
                    help="实验用：跳过 shortcut，直接用 RRT 原始路径")
    ap.add_argument("--docking-free", action="store_true",
                    help="方案甲：允许斜入港（终点=港中心，不做口部前掰直）")
    ap.add_argument("--attempts", type=int, default=8)
    ap.add_argument("--seed", type=int, default=None,
                    help="随机种子(不传则由系统随机，每次不同)")
    ap.add_argument("--no-plot", action="store_true",
                    help="不生成示意图")
    args = ap.parse_args()

    meta = json.loads(META_JSON.read_text(encoding="utf-8"))
    obstacle_polys = [o["polygon_xy"] for o in meta["obstacles"]]
    if args.seed is None:
        seed = random.SystemRandom().randint(0, 2**31 - 1)
    else:
        seed = args.seed

    results = []
    for p in PORTS:
        r = plan_one_port(p["px"], obstacle_polys, args.half_width,
                          args.margin, args.lane_join_y, args.min_radius,
                          args.attempts, seed, args.no_shortcut,
                          args.docking_free)
        results.append(r)
        if r["success"]:
            rr = r.get("min_turn_radius_m")
            print(f"[OK] {r['port']}: 长度 {r['length_m']:.1f} m | "
                  f"max|curv| {r['max_curvature_1_per_m']:.4f} "
                  f"(R≈{rr if rr else 'inf'} m, 约束>= {args.min_radius}) | "
                  f"min_obs {r['min_obstacle_clearance_m']:.2f} m | "
                  f"min_wall {r['min_wall_clearance_m']:.2f} m "
                  f"(attempt {r['attempts']})")
        else:
            print(f"[FAIL] {r['port']}: {r['last_error']}")

    # 走廊内缘半径 = 中心线最小转弯半径 - 走廊半宽（视觉上更直观）
    for r in results:
        if r.get("success") and r.get("min_turn_radius_m"):
            r["corridor_inner_radius_m"] = round(
                r["min_turn_radius_m"] - args.half_width, 3)
        else:
            r["corridor_inner_radius_m"] = None

    summary = {
        "world": "plan_world",
        "obstacle_meta_seed": meta.get("seed"),
        "start_xy": list(START),
        "water_region_xy": [list(WATER[:2]), list(WATER[2:])],
        "corridor_half_width_m": args.half_width,
        "search_margin_m": args.margin,
        "lane_join_y_m": args.lane_join_y,
        "min_radius_constraint_m": args.min_radius,
        "planner_seed": seed,
        "ports": results,
    }
    OUT_JSON.write_text(json.dumps(summary, indent=2, ensure_ascii=False),
                        encoding="utf-8")

    # CSV：每条航道一行汇总
    import csv as _csv
    with OUT_CSV.open("w", newline="", encoding="utf-8") as f:
        w = _csv.writer(f)
        w.writerow(["port", "px", "success", "length_m", "max_curvature_1_per_m",
                    "min_turn_radius_m", "min_radius_constraint_m",
                    "min_obstacle_clearance_m", "min_wall_clearance_m",
                    "attempts", "last_error"])
        for r in results:
            w.writerow([r["port"], r["px"], r["success"],
                        r.get("length_m", ""), r.get("max_curvature_1_per_m", ""),
                        r.get("min_turn_radius_m", ""),
                        r.get("min_radius_constraint_m", ""),
                        r.get("min_obstacle_clearance_m", ""),
                        r.get("min_wall_clearance_m", ""),
                        r["attempts"], r.get("last_error", "")])

    print(f"\n输出: {OUT_JSON}\n      {OUT_CSV}")
    if not args.no_plot:
        _plot(meta, results, args.half_width, OUT_PNG)
        print(f"      {OUT_PNG}")
    return 0 if all(r["success"] for r in results) else 1


def _plot(meta, results, hw: float, out_png: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPoly

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect("equal")
    ax.set_xlim(-6, 106)
    ax.set_ylim(-6, 106)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]  (East)")
    ax.set_ylabel("y [m]  (North)")
    ax.set_title(f"plan_world docking-corridor planning v0 (corridor half-width {hw:.2f} m)")

    # 障碍物
    for o in meta["obstacles"]:
        poly = o["polygon_xy"]
        ax.add_patch(MplPoly(poly, closed=True, facecolor="#e07b39",
                             edgecolor="k", alpha=0.85, zorder=3))
        cx, cy = G.poly_centroid(poly)
        ax.text(cx, cy, str(o["id"]), ha="center", va="center", fontsize=8,
                zorder=4)

    # 三座码头（画成 U 形）
    for p in PORTS:
        px = p["px"]
        for wrect in dock_wall_rects(px):
            xs, ys = zip(*wrect + [wrect[0]])
            ax.plot(xs, ys, color="0.25", lw=2, zorder=2)
            ax.fill(xs, ys, color="#8a7a66", alpha=0.9, zorder=2)

    # 航道：成功画走廊；失败也尽量给出候选/中心线诊断
    colors = ["#d62728", "#2ca02c", "#1f77b4"]
    for r, col in zip(results, colors):
        is_ok = bool(r.get("success"))
        # 所有候选(灰点=shortcut控制点, 浅色线=开阔段)
        cands = r.get("candidates") or []
        for ci, cd in enumerate(cands):
            ccl = np.asarray(cd.get("cl", []))
            op = np.asarray(cd.get("op", []))
            chosen = (ci == r.get("chosen_candidate_idx", -1)) and is_ok
            if len(op):
                ax.plot(op[:, 0], op[:, 1], "o", ms=3, color="0.4",
                        zorder=2)
            if len(ccl):
                ax.plot(ccl[:, 0], ccl[:, 1], color=col,
                        lw=2.2 if chosen else 0.7,
                        ls="-" if chosen else "--",
                        alpha=1.0 if chosen else 0.45, zorder=4)
        cl = np.asarray(r.get("centerline") or [])
        if is_ok and len(cl):
            lo = np.asarray(r["corridor_left"])
            ro = np.asarray(r["corridor_right"])
            ax.fill(np.r_[lo[:, 0], ro[::-1, 0]],
                    np.r_[lo[:, 1], ro[::-1, 1]],
                    color=col, alpha=0.18, zorder=1)
            lbl = f'{r["port"]}  ({r["length_m"]:.1f} m'
            if r.get("dock_dir"):
                lbl += f", {r['dock_dir']}"
            lbl += ")"
            ax.plot(cl[:, 0], cl[:, 1], color=col, lw=2, zorder=5,
                    label=lbl)
            for pt in (r.get("dock_ctrl_pts") or []):
                ax.plot(pt[0], pt[1], marker="*", ms=16, color="k",
                        ls="none", zorder=7)
        elif not is_ok:
            ax.plot([], [], color=col, ls="--",
                    label=f'{r["port"]}  [FAIL] '
                          f'{r.get("last_error", "")}')
            if len(cl):
                ax.plot(cl[:, 0], cl[:, 1], color=col, lw=1.2, ls="--",
                        alpha=0.8, zorder=3)
        else:
            continue

    # 起点
    ax.plot(*START, "k*", ms=18, zorder=5)
    ax.annotate("start (50,0)", START, xytext=(51, 3), fontsize=9)
    for p in PORTS:
        ax.plot(p["px"], PORT_Y, "k^", ms=10, zorder=5)

    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
