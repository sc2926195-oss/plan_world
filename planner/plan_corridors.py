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
MOUTH_Y = PORT_Y - 3.5                    # 泊位口内沿 y (=96.5)，开口朝南
BERTH_HALF_W = 2.5                        # 泊位内净宽半宽
WALL_HALF_W = 0.5                         # 墙厚一半
WALL_HALF_H = 3.5                         # 侧墙半长(=内深7/2)
BACK_OFF = 4.0                            # 后墙中心离泊位中心距离(局部y)

DEFAULT_HALF_WIDTH = 1.5                  # 走廊半宽(走廊总宽3.0m)
DEFAULT_MARGIN = 2.0                      # 搜索时的额外安全裕度(中心线离障碍>=hw+margin)
DEFAULT_LANE_JOIN_Y = 90.0                # 入港直线段起点 y（对接点）


def dock_search_rects(px: float) -> List[G.Poly]:
    """搜索时“码头整体不可进入”的外包矩形（含泊位+墙体+后墙）。"""
    # 世界系：墙体外沿 x∈[px-3.5, px+3.5]，y∈[96.5, 104.5]
    return [G.rect_poly(px, (MOUTH_Y + PORT_Y + 4.5) / 2.0,
                        WALL_HALF_W + BERTH_HALF_W, 4.0)]


def dock_wall_rects(px: float) -> List[G.Poly]:
    """校验用码头墙体矩形（世界系），泊位内部不算障碍。"""
    # 两侧墙：x∈[px±2.5, px±3.5]，y∈[96.5, 103.5]
    side = [
        G.rect_poly(px + BERTH_HALF_W + WALL_HALF_W, (MOUTH_Y + 103.5) / 2.0,
                    WALL_HALF_W, WALL_HALF_H),
        G.rect_poly(px - BERTH_HALF_W - WALL_HALF_W, (MOUTH_Y + 103.5) / 2.0,
                    WALL_HALF_W, WALL_HALF_H),
    ]
    # 后墙：x∈[px-3.5, px+3.5]，y∈[103.5, 104.5]
    back = G.rect_poly(px, 104.0, BERTH_HALF_W + WALL_HALF_W, 0.5)
    return side + [back]


# --------------------------------------------------------------------------- #
# 走廊校验
# --------------------------------------------------------------------------- #
def validate_centerline(centerline: Sequence[G.Point],
                        obstacle_polys: Sequence[G.Poly],
                        wall_polys: Sequence[G.Poly],
                        hw: float,
                        step: float = 0.5) -> Tuple[bool, float, float]:
    """校验中心线：离所有障碍与码头墙体 >= hw。

    返回 (ok, min_obstacle_clearance, min_wall_clearance)。
    """
    # 先按步长抽稀采样点，避免点数过多拖慢
    sampled = G.resample_polyline(centerline, step)
    min_obs = min(G.dist_point_poly(p, poly)
                  for p in sampled for poly in obstacle_polys)
    min_wall = min(G.dist_point_poly(p, poly)
                   for p in sampled for poly in wall_polys)
    ok = (min_obs >= hw - 1e-6) and (min_wall >= hw - 1e-6)
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
def plan_one_port(px: float, obstacle_polys: Sequence[G.Poly],
                  hw: float, margin: float, lane_join_y: float,
                  attempts: int, seed: int) -> Optional[Dict]:
    search_clear = hw + margin
    search_obstacles = list(obstacle_polys)
    for p in PORTS:
        search_obstacles.extend(dock_search_rects(p["px"]))
    wall_polys = dock_wall_rects(px)
    goal = (px, lane_join_y)
    last_err = ""
    fallback = None          # 若某次尝试路径过尖，先记下备用，再换种子找更顺的
    for attempt in range(attempts):
        rng_seed = seed + attempt if seed is not None else None
        planner = RRTStar(search_obstacles, clearance=search_clear,
                          bounds=WATER, seed=rng_seed)
        res = planner.plan(START, goal, max_iterations=1500, improve_iters=250)
        if not res["found"]:
            last_err = "RRT* 未找到路径"
            continue

        # 路径末端吸附到对接点，再做视线简化
        path = list(res["path"])
        path[-1] = goal
        open_path = shortcut(path, search_obstacles, search_clear)

        # 拼接直线入港段（对接点 -> 港中心，逐点加密保证样条末段笔直）
        tail = []
        y = lane_join_y
        while y < PORT_Y - 1e-6:
            tail.append((px, y))
            y = min(y + 0.5, PORT_Y)
        tail.append((px, PORT_Y))

        # 平滑候选（在所有“通过校验”的候选中选最大曲率最小的）：
        #   1) smooth     : 对加密折线做轻微平滑的三次 B 样条 s=0.002*n（转弯最柔和，优先）
        #   2) densified  : s=0 严格插值加密折线（最贴折线、过冲小，保底）
        #   3) shortcut   : s=0 严格插值 shortcut 折线（点最少，最“直”）
        # 注意：FITPACK 的 s>0 平滑在点数很少时可能大幅漂移，因此平滑候选基于
        # “按 2.5 m 加密后的折线”而不是稀疏的 shortcut 折线。
        tail = list(tail)
        dense_open = G.resample_polyline(open_path, 2.0)
        densified_full = list(dense_open) + tail
        shortcut_full = list(open_path) + tail
        candidates = [
            ("smooth", densified_full, 0.002 * len(densified_full)),
            ("densified_s0", densified_full, 0.0),
            ("shortcut_s0", shortcut_full, 0.0),
        ]

        valid = []
        for mode, full, s in candidates:
            try:
                centerline = fit_cubic_bspline(full, n_samples=1201, s=s)
            except ValueError as e:
                last_err = f"B 样条拟合失败({mode}): {e}"
                continue
            # 入港段拉直：y>=92 起精确贴泊位轴线(91~92 渐进过渡)，
            # 保证泊位口(y=96.5)前至少 4.5 m 笔直朝北
            centerline = G.straighten_tail_to_axis(
                centerline, px, 92.0, ease=1.0)
            ok, min_obs, min_wall = validate_centerline(centerline,
                                                        obstacle_polys,
                                                        wall_polys, hw)
            if not ok:
                last_err = (f"校验失败({mode}) min_obs={min_obs:.2f} "
                            f"min_wall={min_wall:.2f} (< {hw})")
                continue
            dense, left, right = make_corridor(centerline, hw)
            curv = G.curvature_from_pts(dense)
            length = sum(G.dist_point_point(a, b)
                         for a, b in zip(dense[:-1], dense[1:]))
            valid.append({
                "mode": mode, "s": s, "full": full,
                "centerline": centerline, "dense": dense,
                "left": left, "right": right,
                "max_curv": max(abs(c) for c in curv),
                "length": length, "min_obs": min_obs, "min_wall": min_wall,
            })

        if not valid:
            continue
        # 长度保护：样条若比“最短候选”长出太多(>10%)，说明它绕远/外飘，
        # 即使曲率小也不选。在“长度接近最短”的候选中再挑最大曲率最小的。
        min_len = min(v["length"] for v in valid)
        shortlisted = [v for v in valid if v["length"] <= min_len * 1.10]
        best = min(shortlisted, key=lambda v: (v["max_curv"], v["length"]))
        if best["max_curv"] > 1.0:        # 转弯半径<1m，太尖：留作备用，换种子再找
            last_err = (f"路径过尖(max_curv={best['max_curv']:.2f})，"
                        f"换随机种子重试")
            if fallback is None or best["max_curv"] < fallback["max_curv"]:
                fallback = best
            continue
        return {
            "port": f"port_{int(px)}",
            "px": px,
            "goal_xy": [px, PORT_Y],
            "lane_join_xy": list(goal),
            "smooth_mode": best["mode"],
            "smoothing_s": round(best["s"], 5),
            "success": True,
            "attempts": attempt + 1,
            "open_waypoints": [list(q) for q in open_path],
            "full_waypoints": [list(q) for q in best["full"]],
            "centerline": [list(q) for q in best["dense"]],
            "corridor_left": [list(q) for q in best["left"]],
            "corridor_right": [list(q) for q in best["right"]],
            "length_m": round(best["length"], 3),
            "max_curvature_1_per_m": round(best["max_curv"], 6),
            "min_obstacle_clearance_m": round(best["min_obs"], 3),
            "min_wall_clearance_m": round(best["min_wall"], 3),
            "last_error": "",
        }
    if fallback is not None:
        best = fallback
        return {
            "port": f"port_{int(px)}",
            "px": px,
            "goal_xy": [px, PORT_Y],
            "lane_join_xy": list(goal),
            "smooth_mode": best["mode"],
            "smoothing_s": round(best["s"], 5),
            "success": True,
            "attempts": attempts,
            "open_waypoints": [list(q) for q in open_path],
            "full_waypoints": [list(q) for q in best["full"]],
            "centerline": [list(q) for q in best["dense"]],
            "corridor_left": [list(q) for q in best["left"]],
            "corridor_right": [list(q) for q in best["right"]],
            "length_m": round(best["length"], 3),
            "max_curvature_1_per_m": round(best["max_curv"], 6),
            "min_obstacle_clearance_m": round(best["min_obs"], 3),
            "min_wall_clearance_m": round(best["min_wall"], 3),
            "last_error": "sharp_path_accepted",
        }
    return {"port": f"port_{int(px)}", "px": px,
            "goal_xy": [px, PORT_Y], "success": False,
            "attempts": attempts, "last_error": last_err}


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
    ap.add_argument("--attempts", type=int, default=6)
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
                          args.margin, args.lane_join_y, args.attempts, seed)
        results.append(r)
        if r["success"]:
            print(f"[OK] {r['port']}: 长度 {r['length_m']:.1f} m | "
                  f"max|curv| {r['max_curvature_1_per_m']:.4f} | "
                  f"min_obs {r['min_obstacle_clearance_m']:.2f} m | "
                  f"min_wall {r['min_wall_clearance_m']:.2f} m "
                  f"(attempt {r['attempts']})")
        else:
            print(f"[FAIL] {r['port']}: {r['last_error']}")

    summary = {
        "world": "plan_world",
        "obstacle_meta_seed": meta.get("seed"),
        "start_xy": list(START),
        "water_region_xy": [list(WATER[:2]), list(WATER[2:])],
        "corridor_half_width_m": args.half_width,
        "search_margin_m": args.margin,
        "lane_join_y_m": args.lane_join_y,
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
                    "min_obstacle_clearance_m", "min_wall_clearance_m",
                    "attempts", "last_error"])
        for r in results:
            w.writerow([r["port"], r["px"], r["success"],
                        r.get("length_m", ""), r.get("max_curvature_1_per_m", ""),
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

    # 航道
    colors = ["#d62728", "#2ca02c", "#1f77b4"]
    for r, col in zip(results, colors):
        if not r["success"]:
            continue
        cl = np.asarray(r["centerline"])
        lo = np.asarray(r["corridor_left"])
        ro = np.asarray(r["corridor_right"])
        ax.fill(np.r_[lo[:, 0], ro[::-1, 0]], np.r_[lo[:, 1], ro[::-1, 1]],
                color=col, alpha=0.18, zorder=1)
        ax.plot(cl[:, 0], cl[:, 1], color=col, lw=2, zorder=3,
                label=f'{r["port"]}  ({r["length_m"]:.1f} m)')
        ax.plot(lo[:, 0], lo[:, 1], color=col, lw=0.8, ls="--", alpha=0.7)
        ax.plot(ro[:, 0], ro[:, 1], color=col, lw=0.8, ls="--", alpha=0.7)

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
