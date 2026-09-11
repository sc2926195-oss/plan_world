# -*- coding: utf-8 -*-
"""实验：港前“两控制点对接线 + R_min 圆弧接驳”(带失败诊断可视化)。

思路：港前轴线上两个硬性控制点(y=92,96)，轨迹必须经过它们 => 入港方向被钉死；
RRT 开阔段任意来向，用 R>=R_min 的圆弧接上对接线。

独立输出(不影响主版本)：
  planner/twopoint_output.json
  planner/twopoint_overview.png   (成功/失败都画：RRT点+控制点+走廊+原因)

用法：
  python3 -m planner.twopoint_experiment --seed 5555 [--min-radius 5.0]
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
OUT_JSON = HERE / "twopoint_output.json"
OUT_PNG = HERE / "twopoint_overview.png"
AXIS_PTS = (92.0, 96.0)          # 对接线上两个硬性控制点 y


def _arc_fillet_to_north(A, E, R):
    hx, hy = E[0] - A[0], E[1] - A[1]
    L = math.hypot(hx, hy)
    if L < 1e-9:
        return None, []
    h = (hx / L, hy / L)
    cross = h[0]
    Delta = math.atan2(abs(cross), h[1])
    if Delta < 1e-3:
        return None, None
    lt = R * math.tan(Delta / 2.0)
    if lt > L - 1e-6:
        return None, []
    T1 = (E[0] - h[0] * lt, E[1] - h[1] * lt)
    sigma = 1.0 if cross > 0 else -1.0
    left = (-h[1], h[0])
    C = (T1[0] + sigma * R * left[0], T1[1] + sigma * R * left[1])
    T2 = (E[0], E[1] + lt)
    p1 = math.atan2(T1[1] - C[1], T1[0] - C[0])
    p2 = math.atan2(T2[1] - C[1], T2[0] - C[0])
    base = math.atan2(math.sin(p2 - p1), math.cos(p2 - p1))
    dphi = base + sigma * 2 * math.pi if base * sigma < 0 else base
    n = max(2, int(abs(dphi) * R / 0.25) + 1)
    pts = [(C[0] + R * math.cos(p1 + dphi * i / n),
            C[1] + R * math.sin(p1 + dphi * i / n))
           for i in range(n + 1)]
    return T1, pts


def _diag(px, op, raw=None, cl=None, err="", **kw):
    d = {"port": f"port_{int(px)}", "px": px, "success": False,
         "last_error": err, "open_waypoints": [list(v) for v in op]}
    if raw is not None:
        d["rrt_raw_path"] = [list(v) for v in raw]
    if cl is not None:
        d["centerline"] = [list(v) for v in cl]
    d.update(kw)
    return d


def _plan_one(px, obs, Rmin, seed, lane_y=90.0, avoid_polys=None):
    search_obs = list(obs)
    if avoid_polys:
        search_obs.extend(list(avoid_polys))
    for q in pc.PORTS:
        search_obs.extend(pc.dock_search_rects(q["px"]))
    E = (px, lane_y)
    walls = pc.dock_wall_rects(px)
    pl = RRTStar(search_obs, clearance=3.5, bounds=pc.WATER, seed=seed)
    res = pl.plan((50, 0), E, max_iterations=1500, improve_iters=250)
    if not res["found"]:
        return _diag(px, [(50.0, 0.0), E], err="RRT未找到")
    path = list(res["path"])
    path[-1] = E
    op = shortcut(path, search_obs, 3.5)
    raw = list(path)

    dense_open = G.resample_polyline(op, 2.0)
    base_s = 0.002 * len(dense_open)
    cl = None
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
            cl = dc
            break
    if cl is None:
        return _diag(px, op, raw=raw, err="开阔段平滑不满足R_min")

    # 三候选方向(硬性控制点对)：正中 / 偏西4m / 偏东4m
    cand = [
        {"name": "center", "P1": (px, 92.0), "P2": (px, 96.0)},
        {"name": "west",   "P1": (px - 4.0, 92.0), "P2": (px, 96.0)},
        {"name": "east",   "P1": (px + 4.0, 92.0), "P2": (px, 96.0)},
    ]
    base_s = 0.002 * (len(cl) // 10)
    best = None
    for ci, cd in enumerate(cand):
        ctrl = [tuple(v) for v in cl] + [cd["P1"], cd["P2"], (px, pc.PORT_Y)]
        ctrl = G.remove_dup(ctrl)
        for k in (0.0, 1.0, 2.0, 4.0, 8.0, 16.0):
            try:
                cc = fit_cubic_bspline(ctrl, n_samples=2001,
                                       s=(base_s * k if k else 0.0))
            except Exception:
                continue
            dc = G.resample_polyline(cc, 0.25)
            okc, mo, mw = pc.validate_centerline(dc, obs, walls, 1.5)
            if not okc:
                continue
            kk = G.curvature_from_pts(dc)
            mx = max(abs(c) for c in kk)
            if mx > 1.0 / Rmin + 1e-9:
                continue
            L = sum(G.dist_point_point(a, b) for a, b in zip(dc[:-1], dc[1:]))
            _, left, right = pc.make_corridor(dc, 1.5)
            item = {"name": cd["name"], "idx": ci, "P1": cd["P1"],
                    "P2": cd["P2"], "dense": dc, "left": left, "right": right,
                    "mx": mx, "L": L, "mo": mo, "mw": mw, "s": k}
            if best is None or item["L"] < best["L"]:
                best = item
    if best is not None:
        dense = best["dense"]
        return {
            "port": f"port_{int(px)}", "px": px, "success": True,
            "rrt_raw_path": [list(v) for v in raw],
            "open_waypoints": [list(v) for v in op],
            "dock_candidate": best["name"],
            "dock_ctrl_pts": [list(best["P1"]), list(best["P2"])],
            "centerline": [list(v) for v in dense],
            "corridor_left": [list(v) for v in best["left"]],
            "corridor_right": [list(v) for v in best["right"]],
            "length_m": round(best["L"], 3),
            "max_curvature_1_per_m": round(best["mx"], 6),
            "min_turn_radius_m": (round(1 / best["mx"], 3)
                                  if best["mx"] > 1e-9 else None),
            "min_obstacle_clearance_m": round(best["mo"], 3),
            "min_wall_clearance_m": round(best["mw"], 3),
            "attempt": seed,
            "last_error": "",
        }
    return _diag(px, op, raw=raw, cl=cl,
                 err="三方向均不可用(平滑/曲率/碰撞)")
    return _diag(px, op, raw=raw, cl=cl, err="无可用R(圆弧接不上)")


def _plot(meta, results, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPoly

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect("equal"); ax.set_xlim(-6, 106); ax.set_ylim(-6, 106)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m] (East)"); ax.set_ylabel("y [m] (North)")
    ax.set_title("two-point docking line experiment (failed ports also shown)")

    for o in meta["obstacles"]:
        ax.add_patch(MplPoly(o["polygon_xy"], closed=True,
                             facecolor="#e07b39", edgecolor="k",
                             alpha=0.85, zorder=3))
    for q in pc.PORTS:
        px = q["px"]
        for w in pc.dock_wall_rects(px):
            xs, ys = zip(*w + [w[0]])
            ax.plot(xs, ys, color="0.25", lw=2, zorder=2)
            ax.fill(xs, ys, color="#8a7a66", alpha=0.9, zorder=2)

    colors = ["#d62728", "#2ca02c", "#1f77b4"]
    for r, col in zip(results, colors):
        px = r["px"]
        # 两个入港硬性控制点 + 对接点
        for yy in AXIS_PTS + (90.0,):
            ax.plot(px, yy, marker="x", ms=9, color="k", ls="none", zorder=6)
        # RRT 原始路径(点) + shortcut 控制点
        rp = np.asarray(r.get("rrt_raw_path", []))
        if len(rp):
            ax.plot(rp[:, 0], rp[:, 1], "o", ms=3, color="0.4", zorder=2)
        op = np.asarray(r.get("open_waypoints", []))
        if len(op):
            ax.plot(op[:, 0], op[:, 1], color=col, lw=0.8, ls=":",
                    alpha=0.9, zorder=4)
            ax.plot(op[:, 0], op[:, 1], "s", ms=4, color="k", zorder=5)

        # 选中的入港方向控制点(P1/P2)用大星标出
        for pt in (r.get("dock_ctrl_pts") or []):
            ax.plot(pt[0], pt[1], marker="*", ms=18, color="k",
                    ls="none", zorder=7)
        if r.get("success"):
            cl = np.asarray(r["centerline"])
            lo = np.asarray(r["corridor_left"]); ro = np.asarray(r["corridor_right"])
            ax.fill(np.r_[lo[:, 0], ro[::-1, 0]], np.r_[lo[:, 1], ro[::-1, 1]],
                    color=col, alpha=0.18, zorder=1)
            ax.plot(cl[:, 0], cl[:, 1], color=col, lw=2, zorder=3,
                    label=f'{r["port"]}  ({r["length_m"]:.1f} m)')
        else:
            label = f'{r["port"]}  [FAIL]'
            cl = np.asarray(r.get("centerline", []))
            if len(cl):
                ax.plot(cl[:, 0], cl[:, 1], color=col, lw=1.2, ls="--",
                        alpha=0.8, zorder=3, label=label)
            else:
                ax.plot([], [], color=col, lw=1.2, ls="--", label=label)
            # 在港中心附近写失败原因(英文，避免字体缺字)
            err_en = {"开阔段平滑不满足R_min": "smooth open: R_min fail",
                      "无可用R(圆弧接不上)": "no R-arc fits",
                      "RRT未找到": "RRT not found"}.get(
                r.get("last_error", ""), r.get("last_error", ""))
            ax.text(px, 103.5, err_en, ha="center", fontsize=8, color=col,
                    bbox=dict(facecolor="white", alpha=0.7, boxstyle="round"))

    ax.plot(*pc.START, "k*", ms=16, zorder=5)
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--min-radius", type=float, default=5.0)
    ap.add_argument("--seed-attempts", type=int, default=3)
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    meta = json.loads(META_JSON.read_text(encoding="utf-8"))
    obs = [o["polygon_xy"] for o in meta["obstacles"]]
    results = []
    for port in pc.PORTS:
        px = port["px"]
        got = None
        avoid = []                     # 失败路径的空间记忆(避让方块)
        for k in range(1, args.seed_attempts + 1):
            t0 = time.time()
            r = _plan_one(px, obs, args.min_radius, seed=2026 + k,
                          avoid_polys=avoid)
            if r.get("success"):
                print(f"[OK] {r['port']}: len={r['length_m']} "
                      f"R={r['min_turn_radius_m']} cand={r.get('dock_candidate')} "
                      f"({time.time()-t0:.1f}s, seed={k})")
                got = r
                break
            print(f"  [{port['name']}] seed={k} 失败: {r['last_error']}")
            got = r
            # 把失败路径的 shortcut 控制点(中段 y<86)记入避让区(少量大方块)
            ows = r.get("open_waypoints") or []
            if ows:
                buf_half = 4.0
                for (x, y) in ows:
                    if y < 86.0:
                        avoid.append(G.rect_poly(x, y, buf_half, buf_half))
                print(f"       -> 已把该路径 {len(ows)} 个控制点中 "
                      f"{sum(1 for _, y in ows if y < 86)} 个加入避让区")
        results.append(got)

    OUT_JSON.write_text(json.dumps({
        "obstacle_meta_seed": meta["seed"],
        "min_radius_m": args.min_radius,
        "ports": results,
    }, indent=2, ensure_ascii=False), encoding="utf-8")
    print("json:", OUT_JSON)
    if not args.no_plot:
        _plot(meta, results, OUT_PNG)
        print("png:", OUT_PNG)
    return 0 if all(p["success"] for p in results) else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
