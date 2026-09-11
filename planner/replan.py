# -*- coding: utf-8 -*-
"""
在线避障重规划（离线原型）：感知障碍 -> 判定走廊被挡 -> 局部栅格 A* 绕行
-> B样条平滑 -> 曲率/安全校验 -> 输出图 + JSON。

配合 planner/occ_grid.py（栅格+A*）。全局航道仍沿用 plan_corridors.py 的结果
（RRT*+B样条+走廊）；这里只把“被新感知障碍挡住的那一小段”替换成绕行段。

用法（离线，不需要 Gazebo）：
  cd ~/vrx_ws/my_plan_world
  python3 -m planner.replan --lane 1                 # 用当前 obstacles_meta 里被感知到的球
  python3 -m planner.replan --lane 1 --inject always # 人为在航道上放一个障碍(演示必被挡)
输出：
  planner/replan_demo.png   原走廊 vs 绕行走廊 + 障碍圆 + A* 原始折线
  planner/replan_output.json 绕行中心线/走廊/指标（供后续接入仿真/可视化）

设计口径（用户已确认）：
  - 船知道自身位姿(真值)与港口位置；不知道障碍位置；
  - 黑球经 lidar 感知后拟合为圆(cx,cy,r)；
  - 走廊被挡判据：中心线到圆面距离 < 半宽 hw + 安全裕度 margin；
  - 栅格 inflation = hw + margin（保证绕行中心线生成的走廊不碰障碍）。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import geometry as G
from . import bspline as BS
from .occ_grid import (Circle, OccupancyGrid, min_clearance,
                       polyline_clear, shortcut_circles)

HERE = Path(__file__).resolve().parent
WORLD_DIR = HERE.parent
OUT_JSON = HERE / "planner_output.json"
META_JSON = WORLD_DIR / "obstacles_meta.json"
REPLAN_JSON = HERE / "replan_output.json"
REPLAN_PNG = HERE / "replan_demo.png"

WATER = (0.0, 0.0, 100.0, 100.0)
PORTS_X = (0.0, 50.0, 100.0)
Point = Tuple[float, float]


# --------------------------------------------------------------------------- #
# 数据装载
# --------------------------------------------------------------------------- #
def load_lane(lane: int, path: Path = OUT_JSON) -> Dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    for p in data["ports"]:
        if p.get("success") and int(round(p["px"] / 50.0)) == lane:
            return p
    raise SystemExit(f"lane {lane} 无可用规划结果({path})")


def load_circles(meta_path: Path = META_JSON) -> List[Circle]:
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    out = []
    for ob in meta["obstacles"]:
        if ob.get("shape") == "ball" and ob.get("radius_m"):
            out.append(Circle(ob["x"], ob["y"], ob["radius_m"], f"ball{ob['id']}"))
        else:
            poly = [(float(a), float(b)) for a, b in ob["polygon_xy"]]
            cx, cy = G.poly_centroid(poly)
            r = max(G.dist_point_point((cx, cy), v) for v in poly)
            out.append(Circle(cx, cy, r, f"poly{ob['id']}"))
    return out


# --------------------------------------------------------------------------- #
# 走廊被挡检测
# --------------------------------------------------------------------------- #
def centerline_blocked_range(centerline: Sequence[Point],
                             circles: Sequence[Circle],
                             clearance: float) -> Optional[Tuple[int, int]]:
    """返回第一段被挡(中心线到圆面 < clearance)的索引区间 [i0,i1]。"""
    bad = []
    for i, p in enumerate(centerline):
        if any(c.dist_surface(p) < clearance for c in circles):
            bad.append(i)
    if not bad:
        return None
    i0 = bad[0]
    i1 = i0
    for i in bad:
        if i > i1 + 1:
            break
        i1 = i
    return i0, i1


def _walk_clear(centerline: Sequence[Point], circles: Sequence[Circle],
                clearance: float, idx: int, direction: int) -> int:
    """从 idx 沿 direction 走到第一个“对所有圆都 >= clearance”的点。"""
    i = idx
    n = len(centerline)
    while 0 <= i < n:
        p = centerline[i]
        if all(c.dist_surface(p) >= clearance for c in circles):
            return i
        i += direction
    return 0 if direction < 0 else n - 1


def _advance_by_arc(centerline: Sequence[Point], idx: int,
                    direction: int, dist_m: float) -> int:
    """在折线上从 idx 沿 direction 再走 dist_m(米)，返回新索引。"""
    i = idx
    acc = 0.0
    n = len(centerline)
    while 0 < i < n - 1 and acc < dist_m:
        j = i + direction
        acc += G.dist_point_point(centerline[i], centerline[j])
        i = j
    return i


# --------------------------------------------------------------------------- #
# 局部 A* + B样条
# --------------------------------------------------------------------------- #
def _seed_for_spline(w_lo: int, i_entry: int, i_exit: int, w_hi: int,
                     ref: Sequence[Point], raw: Sequence[Point]) -> List[Point]:
    """构造参与样条拟合的点：入口前一小段 + A* 绕行段 + 出口后一小段。"""
    pre = list(ref[w_lo:i_entry + 1])
    post = list(ref[i_exit:w_hi + 1])
    mid = list(raw)
    pts = pre + mid + post
    pts = G.resample_polyline(G.remove_dup(pts), 1.0)   # 均匀加密，利于样条
    dec = [pts[i] for i in G.decimate_indices(pts, 2.0)]
    if len(dec) < 4:
        dec = [pts[i] for i in G.decimate_indices(pts, 1.0)]
    return dec



def _snap_ends(pts: Sequence[Point], p_start: Point, p_end: Point,
               k: int = 24) -> List[Point]:
    """把平滑样条(s>0 时端点不一定精确经过拼接点)的两端线性回拉到
    精确的拼接点，避免与原航道接出 V 形回折。"""
    out = [tuple(p) for p in pts]
    n = len(out)
    k = max(3, min(k, n // 3))
    d0 = (p_start[0] - out[0][0], p_start[1] - out[0][1])
    d1 = (p_end[0] - out[-1][0], p_end[1] - out[-1][1])
    for i in range(k):
        w = 1.0 - i / (k - 1)          # 1 -> 0
        out[i] = (out[i][0] + d0[0] * w, out[i][1] + d0[1] * w)
        j = n - 1 - i
        out[j] = (out[j][0] + d1[0] * w, out[j][1] + d1[1] * w)
    out[0] = tuple(p_start); out[-1] = tuple(p_end)
    return out


def replan_local(centerline: Sequence[Point],
                 circles: Sequence[Circle],
                 hw: float = 1.5,
                 margin: float = 1.0,
                 cell: float = 0.5,
                 min_radius: float = 5.0,
                 lookback: float = 10.0,
                 lookahead: float = 10.0,
                 bounds_pad: float = 8.0,
                 context: float = 8.0,
                 window_scales: Sequence[float] = (1.0, 1.5, 2.0, 2.5, 3.0),
                 ) -> Dict:
    """把被挡段替换成“A*绕行 + B样条”的局部重规划结果。

    若曲率/安全/码头校验不通过，会自动按 window_scales 加大绕行前后过渡窗重试
    （R_min 越大，横向让出障碍所需的过渡距离越长）。
    """
    ref = [tuple(p) for p in centerline]
    clearance = hw + margin
    blocked = centerline_blocked_range(ref, circles, clearance)
    if blocked is None:
        return {"blocked": False, "reason": "当前参考走廊没有被感知障碍挡住"}
    i0, i1 = blocked
    ref_len = _polyline_length(ref)
    ref_step = ref_len / max(1, len(ref) - 1)
    from .plan_corridors import dock_wall_rects
    walls = [poly for px_ in PORTS_X for poly in dock_wall_rects(px_)]

    best_any = None
    for scale in window_scales:
        lb, la = lookback * scale, lookahead * scale
        i_entry = _walk_clear(ref, circles, clearance, i0, -1)
        i_exit = _walk_clear(ref, circles, clearance, i1, +1)
        i_entry = _advance_by_arc(ref, i_entry, -1, lb)
        i_exit = _advance_by_arc(ref, i_exit, +1, la)
        if i_entry >= i_exit:
            continue

        sub = ref[i_entry:i_exit + 1]
        xs = [p[0] for p in sub]; ys = [p[1] for p in sub]
        for c in circles:
            xs += [c.x - c.r - clearance, c.x + c.r + clearance]
            ys += [c.y - c.r - clearance, c.y + c.r + clearance]
        x0 = max(WATER[0], min(xs) - bounds_pad); y0 = max(WATER[1], min(ys) - bounds_pad)
        x1 = min(WATER[2], max(xs) + bounds_pad); y1 = min(WATER[3], max(ys) + bounds_pad)
        grid = OccupancyGrid((x0, y0, x1, y1), cell=cell,
                             inflation=clearance, soft_margin=1.5, soft_cost=3.0)
        grid.add_circles([c for c in circles
                          if (c.x + c.r + clearance >= x0 and c.x - c.r - clearance <= x1
                              and c.y + c.r + clearance >= y0 and c.y - c.r - clearance <= y1)])
        raw = grid.astar(ref[i_entry], ref[i_exit])
        if raw is None:
            continue
        simple = shortcut_circles(raw, circles, clearance, step=0.4)

        ctx = int(round(context / max(ref_step, 1e-6)))
        w_lo = max(0, i_entry - ctx)
        w_hi = min(len(ref) - 1, i_exit + ctx)
        seed = _seed_for_spline(w_lo, i_entry, i_exit, w_hi, ref, simple)
        p_start, p_end = ref[w_lo], ref[w_hi]
        best = None
        for s in (0.0, 0.5, 2.0, 8.0, 20.0, 60.0):
            try:
                det = BS.fit_cubic_bspline(seed, n_samples=400, s=s)
            except ValueError:
                continue
            det = _snap_ends(det, p_start, p_end)
            assembled = list(ref[:w_lo]) + list(det) + list(ref[w_hi + 1:])
            dense_c = G.resample_uniform(G.remove_dup(assembled), 0.5)
            mc_center = min_clearance(dense_c, circles)
            curv = G.curvature_from_pts(dense_c)
            max_curv = max((abs(v) for v in curv), default=0.0)
            r_min = (1.0 / max_curv) if max_curv > 1e-9 else float("inf")
            min_wall = min((G.dist_point_poly(p_, poly) for p_ in dense_c
                            for poly in walls), default=float("inf"))
            ok_clear = mc_center >= hw + 1e-3
            ok_curv = (min_radius <= 0) or (r_min >= min_radius - 1e-6)
            ok_wall = min_wall >= hw - 1e-6
            cand = {"dense_c": dense_c, "s": s, "mc": mc_center,
                    "max_curv": max_curv, "r_min": r_min, "min_wall": min_wall,
                    "ok_clear": ok_clear, "ok_curv": ok_curv, "ok_wall": ok_wall,
                    "entry_idx": i_entry, "exit_idx": i_exit,
                    "raw": raw, "simple": simple, "scale": scale}
            all_ok = ok_clear and ok_curv and ok_wall
            if best is None or all_ok:
                best = cand
            if all_ok:
                break
        if best is None:
            continue
        result = {
            "blocked": True,
            "success": bool(best["ok_clear"] and best["ok_curv"] and best["ok_wall"]),
            "blocked_idx": [i0, i1],
            "entry_idx": best["entry_idx"], "exit_idx": best["exit_idx"],
            "entry_xy": list(ref[best["entry_idx"]]),
            "exit_xy": list(ref[best["exit_idx"]]),
            "window_scale": best["scale"],
            "lookback_m": round(lookback * best["scale"], 2),
            "lookahead_m": round(lookahead * best["scale"], 2),
            "sensed_circles": [list(c.as_tuple()) for c in circles],
            "astar_raw": [list(p) for p in best["raw"]],
            "astar_shortcut": [list(p) for p in best["simple"]],
            "detour_spline_s": best["s"],
            "detour_ok_clear": best["ok_clear"],
            "detour_ok_curv": best["ok_curv"],
            "detour_ok_wall": best["ok_wall"],
            "centerline": [list(p) for p in best["dense_c"]],
            "corridor_left": [list(p) for p in _make_offsets(best["dense_c"], hw)[0]],
            "corridor_right": [list(p) for p in _make_offsets(best["dense_c"], hw)[1]],
            "length_m": round(_polyline_length(best["dense_c"]), 3),
            "min_obstacle_clearance_m": round(best["mc"], 3),
            "min_wall_clearance_m": round(best["min_wall"], 3),
            "max_curvature_1_per_m": round(best["max_curv"], 6),
            "min_turn_radius_m": (round(best["r_min"], 3)
                                  if math.isfinite(best["r_min"]) else None),
            "params": {"hw": hw, "margin": margin, "cell": cell,
                       "min_radius": min_radius, "lookback": lookback,
                       "lookahead": lookahead, "context": context},
        }
        if best_any is None or (result["success"] and not best_any["success"]):
            best_any = result
        if result["success"]:
            return result
    if best_any is not None:
        best_any["reason"] = "窗口已自动扩大仍不满足全部约束，返回最优尝试"
        return best_any
    return {"blocked": True, "success": False,
            "reason": "局部 A* 未找到路径(窗口太小/障碍太密)"}


def _make_offsets(dense: Sequence[Point], hw: float):
    return (G.offset_polyline(dense, hw, left=True),
            G.offset_polyline(dense, hw, left=False))


def _build_corridor(centerline: Sequence[Point], hw: float, step: float = 0.5):
    dense = G.resample_uniform(centerline, step)
    left, right = _make_offsets(dense, hw)
    return dense, left, right


def _polyline_length(pts: Sequence[Point]) -> float:
    return sum(G.dist_point_point(a, b) for a, b in zip(pts[:-1], pts[1:]))


# --------------------------------------------------------------------------- #
# 出图
# --------------------------------------------------------------------------- #
def plot_demo(lane: Dict, circles: Sequence[Circle], result: Dict,
              hw: float, out_png: Path = REPLAN_PNG) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ref = [tuple(p) for p in lane["centerline"]]
    fig, ax = plt.subplots(figsize=(7.5, 6.5), dpi=130)
    # 水域 & 港口
    ax.add_patch(plt.Rectangle((0, 0), 100, 100, fill=False, ec="0.6", lw=1))
    from .plan_corridors import dock_wall_rects
    for px in PORTS_X:
        for poly in dock_wall_rects(px):
            ax.add_patch(plt.Polygon(poly, closed=True, color="0.35", alpha=0.9))
    # 障碍：全部(灰) / 感知(红)
    for c in load_circles():
        ax.add_patch(plt.Circle((c.x, c.y), c.r, color="0.25", alpha=0.35))
    for c in circles:
        ax.add_patch(plt.Circle((c.x, c.y), c.r, color="crimson", alpha=0.9))
        ax.add_patch(plt.Circle((c.x, c.y), c.r + hw, fill=False,
                                ec="crimson", ls=":", lw=1))
    # 原走廊
    dense, l0, r0 = _build_corridor(ref, hw)
    ax.fill([p[0] for p in l0] + [p[0] for p in r0[::-1]],
            [p[1] for p in l0] + [p[1] for p in r0[::-1]],
            color="0.75", alpha=0.35, label="original corridor")
    ax.plot([p[0] for p in ref], [p[1] for p in ref],
            color="0.45", lw=1.2, ls="--", label="original centerline")
    # A* 原始
    if result.get("astar_shortcut"):
        ax.plot([p[0] for p in result["astar_shortcut"]],
                [p[1] for p in result["astar_shortcut"]],
                color="darkorange", lw=1.1, ls="-.", label="A* detour (shortcut)")
    # 新走廊
    if result.get("success"):
        l1 = result["corridor_left"]; r1 = result["corridor_right"]
        ax.fill([p[0] for p in l1] + [p[0] for p in r1[::-1]],
                [p[1] for p in l1] + [p[1] for p in r1[::-1]],
                color="tab:green", alpha=0.30, label="replanned corridor")
        ax.plot([p[0] for p in result["centerline"]],
                [p[1] for p in result["centerline"]],
                color="tab:green", lw=1.8, label="replanned centerline")
        ax.plot(*result["entry_xy"], marker="o", color="tab:blue", ms=6,
                label="detour entry")
        ax.plot(*result["exit_xy"], marker="s", color="tab:purple", ms=6,
                label="detour exit")
    ax.set_aspect("equal"); ax.set_xlim(-5, 105); ax.set_ylim(-5, 110)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    title = f"lane {int(round(lane['px']/50.0))}: local A* detour"
    if result.get("success"):
        title += f"  (R_min={result['min_turn_radius_m']}m, clear={result['min_obstacle_clearance_m']}m)"
    ax.set_title(title)
    ax.legend(loc="lower left", fontsize=8)
    ax.grid(True, alpha=0.25)
    fig.tight_layout(); fig.savefig(out_png); plt.close(fig)


# --------------------------------------------------------------------------- #
# 演示 CLI
# --------------------------------------------------------------------------- #
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--lane", type=int, default=1, choices=(0, 1, 2))
    ap.add_argument("--inject", choices=("auto", "always", "never"), default="auto",
                    help="没有真实障碍挡路时是否人为放一个障碍(演示用)")
    ap.add_argument("--half-width", type=float, default=1.5)
    ap.add_argument("--margin", type=float, default=1.0)
    ap.add_argument("--cell", type=float, default=0.5)
    ap.add_argument("--min-radius", type=float, default=5.0)
    ap.add_argument("--lookback", type=float, default=10.0)
    ap.add_argument("--lookahead", type=float, default=10.0)
    ap.add_argument("--s-curve", action="store_true",
                    help="注入两个障碍(错开)演示 S 形绕行")
    ap.add_argument("--sense-range", type=float, default=40.0,
                    help="模拟 lidar 感知范围：只把离航道中心线 < 该距离的障碍算作已感知")
    args = ap.parse_args()

    lane = load_lane(args.lane)
    ref = [tuple(p) for p in lane["centerline"]]
    all_c = load_circles()
    # 模拟“路上已被 lidar 感知到”的障碍：离航道较近的那些
    sensed = []
    for c in all_c:
        d = min(G.dist_point_point((c.x, c.y), p) for p in ref)
        if d <= args.sense_range + c.r:
            sensed.append(c)
    clearance = args.half_width + args.margin
    blocked = centerline_blocked_range(ref, sensed, clearance)
    injected = False
    if blocked is None and args.inject != "never":
        i = int(len(ref) * (0.45 if args.s_curve else 0.55))
        px, py = ref[i]
        sensed.append(Circle(px, py, 1.6, "inject-demo"))
        if args.s_curve:
            j = int(len(ref) * 0.68)
            qx, qy = ref[j]
            sensed.append(Circle(qx + 4.5, qy, 1.6, "inject-demo2"))
        injected = True
    print(f"lane {args.lane}: 感知圆 {len(sensed)} 个"
          + ("（含演示注入障碍）" if injected else ""))

    result = replan_local(ref, sensed, hw=args.half_width, margin=args.margin,
                          cell=args.cell, min_radius=args.min_radius,
                          lookback=args.lookback, lookahead=args.lookahead)
    print(json.dumps({k: v for k, v in result.items()
                      if k not in ("centerline", "corridor_left",
                                   "corridor_right", "astar_raw",
                                   "astar_shortcut", "sensed_circles")},
                     ensure_ascii=False, indent=2))
    plot_demo(lane, sensed, result, args.half_width)
    REPLAN_JSON.write_text(json.dumps(
        {"lane": args.lane, "injected": injected,
         "sensed_circles": [list(c.as_tuple()) for c in sensed],
         "result": result}, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"图: {REPLAN_PNG}\nJSON: {REPLAN_JSON}")
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
