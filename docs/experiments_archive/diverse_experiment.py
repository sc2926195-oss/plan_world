# -*- coding: utf-8 -*-
"""多样候选实验：一次生成 3 条“差异够大”的开阔路径，再统一做入港对接。

差异指标：
  1) 末端入港朝向 psi (bin ~10deg)
  2) 路径几何(抽稀点平均距离>=阈值)   -> 保证“开头控制点/走法”也不一样
  3) 关闭 Informed 椭圆收敛 + 多种子，让首解更多样

对接：每条候选试 3 个入港方向(正中/偏西/偏东两控制点)，取满足
曲率<=1/R_min 且碰撞安全的最短解。

输出：planner/diverse_output.json / diverse_overview.png (独立，不动主版本)
用法：
  python3 -m planner.diverse_experiment --seed 5555 [--min-radius 5.0]
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from . import geometry as G
from .bspline import fit_cubic_bspline
from .rrt_star import RRTStar, shortcut
from . import plan_corridors as pc

HERE = Path(__file__).resolve().parent
META_JSON = HERE.parent / "obstacles_meta.json"
OUT_JSON = HERE / "diverse_output.json"
OUT_PNG = HERE / "diverse_overview.png"
DOCK_DIRS = [
    ("center", lambda px: ((px, 92.0), (px, 96.0))),
    ("west",   lambda px: ((px - 4.0, 92.0), (px, 96.0))),
    ("east",   lambda px: ((px + 4.0, 92.0), (px, 96.0))),
]


def open_candidate(px, obs, Rmin, seed, lane_y=90.0):
    """RRT(不收敛)+shortcut+开阔平滑。返回 cl 或 None。"""
    search_obs = list(obs)
    for q in pc.PORTS:
        search_obs.extend(pc.dock_search_rects(q["px"]))
    goal = (px, lane_y)
    pl = RRTStar(search_obs, clearance=3.5, bounds=pc.WATER, seed=seed)
    res = pl.plan((50.0, 0.0), goal, max_iterations=1200,
                  improve_iters=60, informed=False)   # 不做椭圆收敛，保持多样
    if not res["found"]:
        return None
    path = list(res["path"])
    path[-1] = goal
    op = shortcut(path, search_obs, 3.5)

    dense_open = G.resample_polyline(op, 2.0)
    base_s = 0.002 * len(dense_open)
    for k in (0.0, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0):
        try:
            c = fit_cubic_bspline(dense_open, n_samples=2001,
                                  s=(base_s * k if k else 0.0))
        except Exception:
            continue
        dc = G.resample_polyline(c, 0.25)
        okc, _, _ = pc.validate_centerline(dc, obs, [], 1.5)
        if not okc:
            continue
        kk = G.curvature_from_pts(dc)
        if max(abs(x) for x in kk) <= 1.0 / Rmin + 1e-9:
            return {"cl": dc, "op": op, "psi_deg": math.degrees(
                math.atan2(sum(math.sin(a) for a in G.sample_heading(dc)[-6:]),
                           sum(math.cos(a) for a in G.sample_heading(dc)[-6:])))}
    return None


def path_mean_dist(a, b, k=40):
    """两条折线按弧长重采样后对应点的平均距离。"""
    sa = G.resample_polyline(a, max(G.dist_point_point(a[0], a[-1]) / k, 0.1))
    sb = G.resample_polyline(b, max(G.dist_point_point(b[0], b[-1]) / k, 0.1))
    n = min(len(sa), len(sb))
    return sum(G.dist_point_point(sa[i], sb[i]) for i in range(0, n, max(1, n // k))) / max(1, n // k + 1)


def collect_diverse(px, obs, Rmin, want=3, max_trials=30):
    kept = []
    tried = 0
    for seed in range(1, max_trials + 1):
        if len(kept) >= want:
            break
        tried += 1
        cand = open_candidate(px, obs, Rmin, seed=2026 + seed)
        if cand is None:
            continue
        new = True
        for k in kept:
            if abs(k["psi_deg"] - cand["psi_deg"]) < 12 and \
                    path_mean_dist(k["cl"], cand["cl"]) < 8.0:
                new = False
                break
        if new:
            cand["seed"] = seed
            kept.append(cand)
    return kept, tried


def dock_candidate(px, cl, obs, Rmin, hw=1.5):
    """用 3 个入港方向对接，返回最短可用 or None。"""
    walls = pc.dock_wall_rects(px)
    best = None
    for name, mk in DOCK_DIRS:
        P1, P2 = mk(px)
        ctrl = G.remove_dup([tuple(v) for v in cl] + [P1, P2,
                                                      (px, pc.PORT_Y)])
        for k in (0.0, 1.0, 2.0, 4.0, 8.0):
            try:
                c = fit_cubic_bspline(ctrl, n_samples=2001,
                                      s=(0.002 * (len(ctrl) // 10) * k
                                         if k else 0.0))
            except Exception:
                continue
            dc = G.resample_polyline(c, 0.25)
            okc, mo, mw = pc.validate_centerline(dc, obs, walls, hw)
            if not okc:
                continue
            kk = G.curvature_from_pts(dc)
            mx = max(abs(x) for x in kk)
            if mx > 1.0 / Rmin + 1e-9:
                continue
            L = sum(G.dist_point_point(a, b) for a, b in zip(dc[:-1], dc[1:]))
            item = {"dir": name, "P1": P1, "P2": P2, "dense": dc,
                    "mx": mx, "L": L, "mo": mo, "mw": mw}
            if best is None or L < best["L"]:
                best = item
    return best


def _plot(meta, results, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPoly

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect("equal"); ax.set_xlim(-6, 106); ax.set_ylim(-6, 106)
    ax.grid(True, alpha=0.3)
    ax.set_title("diverse-candidate experiment (grey dots=shortcut ctrl pts)")
    for o in meta["obstacles"]:
        ax.add_patch(MplPoly(o["polygon_xy"], closed=True,
                             facecolor="#e07b39", edgecolor="k",
                             alpha=0.85, zorder=3))
    for q in pc.PORTS:
        for w in pc.dock_wall_rects(q["px"]):
            xs, ys = zip(*w + [w[0]])
            ax.plot(xs, ys, color="0.25", lw=2); ax.fill(xs, ys,
                                                         color="#8a7a66",
                                                         alpha=0.9)
    colors = ["#d62728", "#2ca02c", "#1f77b4"]
    for r, col in zip(results, colors):
        px = r["px"]
        if r.get("candidates"):
            for ci, c in enumerate(r["candidates"]):
                cl = np.asarray(c["cl"])
                ls = "-" if ci == r.get("chosen_idx") else "--"
                lw = 2.5 if ci == r.get("chosen_idx") else 0.8
                ax.plot(cl[:, 0], cl[:, 1], color=col, ls=ls, lw=lw,
                        alpha=0.7 if ci != r.get("chosen_idx") else 1.0)
                op = np.asarray(c["op"])
                ax.plot(op[:, 0], op[:, 1], "o", ms=3, color="0.35",
                        zorder=2)
        if r.get("success") and r.get("centerline"):
            cl = np.asarray(r["centerline"])
            lo = np.asarray(r["corridor_left"]); ro = np.asarray(r["corridor_right"])
            ax.fill(np.r_[lo[:, 0], ro[::-1, 0]],
                    np.r_[lo[:, 1], ro[::-1, 1]], color=col, alpha=0.18)
            ax.plot(cl[:, 0], cl[:, 1], color=col, lw=2,
                    label=f"{r['port']} ({r['length_m']:.0f}m, "
                          f"{r.get('dock_dir','')})")
            for pt in (r.get("dock_ctrl_pts") or []):
                ax.plot(pt[0], pt[1], marker="*", ms=16, color="k",
                        ls="none")
        else:
            ax.plot([], [], color=col,
                    label=f"{r['port']} [FAIL] {r.get('last_error','')}")
    ax.plot(*pc.START, "k*", ms=16)
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout(); fig.savefig(out_png, dpi=130); plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--min-radius", type=float, default=5.0)
    ap.add_argument("--n-candidates", type=int, default=3)
    ap.add_argument("--max-trials", type=int, default=30)
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    meta = json.loads(META_JSON.read_text(encoding="utf-8"))
    obs = [o["polygon_xy"] for o in meta["obstacles"]]
    results = []
    for port in pc.PORTS:
        px = port["px"]
        cands, tried = collect_diverse(px, obs, args.min_radius,
                                       want=args.n_candidates,
                                       max_trials=args.max_trials)
        print(f"[{port['name']}] 收集到 {len(cands)} 条候选"
              f"(试了 {tried} 次)", end=" ")
        best = None
        chosen = -1
        for ci, c in enumerate(cands):
            b = dock_candidate(px, c["cl"], obs, args.min_radius)
            if b is not None and (best is None or b["L"] < best["L"]):
                best = b
                chosen = ci
        if best is not None:
            cc = cands[chosen]
            dc = best["dense"]
            _, left, right = pc.make_corridor(dc, 1.5)
            r = {"port": port["name"], "px": px, "success": True,
                 "candidates": [{"cl": [list(v) for v in c["cl"]],
                                 "op": [list(v) for v in c["op"]],
                                 "psi_deg": round(c["psi_deg"], 1),
                                 "seed": c["seed"]} for c in cands],
                 "chosen_idx": chosen,
                 "dock_dir": best["dir"],
                 "dock_ctrl_pts": [list(best["P1"]), list(best["P2"])],
                 "centerline": [list(v) for v in dc],
                 "corridor_left": [list(v) for v in left],
                 "corridor_right": [list(v) for v in right],
                 "length_m": round(best["L"], 3),
                 "max_curvature_1_per_m": round(best["mx"], 6),
                 "min_turn_radius_m": (round(1 / best["mx"], 3)
                                       if best["mx"] > 1e-9 else None),
                 "min_obstacle_clearance_m": round(best["mo"], 3),
                 "min_wall_clearance_m": round(best["mw"], 3),
                 "last_error": ""}
            print(f"-> OK dir={best['dir']} len={best['L']:.1f}")
            results.append(r)
        else:
            print("-> FAIL (无候选对接成功)")
            results.append({"port": port["name"], "px": px,
                            "success": False,
                            "candidates": [{"cl": [list(v) for v in c["cl"]],
                                            "op": [list(v) for v in c["op"]],
                                            "psi_deg": round(c["psi_deg"], 1)}
                                           for c in cands],
                            "last_error": "三方向均无法对接"})
    OUT_JSON.write_text(json.dumps({"obstacle_meta_seed": meta["seed"],
                                    "min_radius_m": args.min_radius,
                                    "ports": results},
                                   indent=2, ensure_ascii=False),
                        encoding="utf-8")
    print("json:", OUT_JSON)
    if not args.no_plot:
        _plot(meta, results, OUT_PNG)
        print("png:", OUT_PNG)
    return 0 if all(p["success"] for p in results) else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
