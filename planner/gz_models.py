# -*- coding: utf-8 -*-
"""
把规划结果(planner_output.json)以“纯视觉静态模型”显示到 Gazebo（推荐方案）。

为什么用模型而不是 gz marker：
  marker 走 GUI 侧渲染，某些环境下可能不显示；而用 /world/<world>/create 服务
  注入一个“只有 visual、没有 collision、static”的模型属于仿真服务器端操作，
  Gazebo GUI 一定能看到。plan_world.sdf 已带 gz-sim-user-commands-system。

实现：
  每条航道 = 一个模型 plan_corridor_<idx>，沿中心线铺一排很薄的半透明盒子
  （无物理），整体高于水面(z≈0.8)避免被波浪遮挡。显示/清除 = create/remove。

用法(仿真 GUI 运行后，另开终端，在 ~/vrx_ws/my_plan_world 下)：
  python3 -m planner.gz_models show  --lane 0|1|2   # 显示某条航道
  python3 -m planner.gz_models clear                # 清除三条航道
  python3 -m planner.gz_models demo  --interval 4   # 依次展示 0->1->2
"""
from __future__ import annotations

import argparse
import json
import math
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Sequence, Tuple

from . import geometry as G

HERE = Path(__file__).resolve().parent
OUT_JSON = HERE / "planner_output.json"
WORLD = "plan_world"

Z = 0.8                 # 走廊显示高度（高于波浪顶）
THICK = 0.06            # 盒子厚度
SEG_STEP = 3.0          # 铺盒子用的中心线抽稀步长
COLORS = {
    0: (0.90, 0.20, 0.20),
    1: (0.15, 0.75, 0.25),
    2: (0.10, 0.45, 0.80),
}
POINT = Tuple[float, float]


# --------------------------------------------------------------------------- #
# SDF 生成
# --------------------------------------------------------------------------- #
def _xml_escape(s: str) -> str:
    return s.replace("\\", "\\\\").replace('"', '\\"')


def lane_sdf(lane: dict, idx: int, hw: float) -> str:
    rgb = COLORS[idx]
    raw = [tuple(p) for p in lane["centerline"]]
    keep = G.decimate_indices(raw, SEG_STEP)
    pts = [raw[i] for i in keep]

    mat = (f"<ambient>{rgb[0]} {rgb[1]} {rgb[2]} 0.55</ambient>"
           f"<diffuse>{rgb[0]} {rgb[1]} {rgb[2]} 0.55</diffuse>"
           "<specular>0.1 0.1 0.1 1</specular>")
    links = []
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        L = math.hypot(x1 - x0, y1 - y0)
        if L < 1e-6:
            continue
        mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        yaw = math.atan2(y1 - y0, x1 - x0)
        # 盒子长度沿局部 X，因此把它转 yaw 角铺在航向上
        links.append(
            f'<link name="seg_{i}">'
            f'<visual name="v"><pose>{mx:.3f} {my:.3f} {Z:.3f} '
            f'0 0 {yaw:.4f}</pose>'
            '<geometry><box>'
            f'<size>{L:.3f} {2.0 * hw:.3f} {THICK}</size>'
            '</box></geometry>'
            f'<material>{mat}</material></visual></link>')
    name = f"plan_corridor_{idx}"
    sdf = ('<?xml version="1.0" ?>\n<sdf version="1.9">\n'
           f'<model name="{name}">\n<static>true</static>\n'
           '<self_collide>false</self_collide>\n'
           + "\n".join(links) +
           "\n</model>\n</sdf>\n")
    return sdf


# --------------------------------------------------------------------------- #
# 调用 gz service
# --------------------------------------------------------------------------- #
def _call(service: str, reqtype: str, reptype: str, req: str,
          dry: bool = False, timeout_ms: int = 6000) -> int:
    cmd = ["gz", "service", "-s", service, "--reqtype", reqtype,
           "--reptype", reptype, "--timeout", str(timeout_ms),
           "--req", req]
    if dry:
        print(" ".join(shlex.quote(c) for c in cmd))
        return 0
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    out = ((r.stdout or "") + (r.stderr or "")).strip()
    if r.returncode != 0:
        print(f"[warn] {service}: {out[:200]}")
    return r.returncode


def _create_service(world: str) -> str:
    return f"/world/{world}/create"


def _remove_service(world: str) -> str:
    return f"/world/{world}/remove"


def show_lane(idx: int, world: str, dry: bool) -> bool:
    data = json.loads(OUT_JSON.read_text(encoding="utf-8"))
    lane = next(p for p in data["ports"]
                if p["success"] and int(p["px"] / 50.0) == idx)
    model_name = f"plan_corridor_{idx}"
    # 先删后建（create 对同名实体是 no-op）
    if not dry:
        _call(_remove_service(world), "gz.msgs.Entity", "gz.msgs.Boolean",
              f'name: "{model_name}" type: MODEL', dry=False)
        time.sleep(0.2)
    hw = float(data["corridor_half_width_m"])
    sdf = lane_sdf(lane, idx, hw)
    if len(sdf) > 100_000:
        print(f"[warn] 该航道 SDF 过大({len(sdf)} B)，显示可能失败")
    sdf_oneline = re.sub(r"\s+", " ", sdf).strip()   # XML 换行/缩进去掉
    req = f'sdf: "{_xml_escape(sdf_oneline)}"'
    rc = _call(_create_service(world), "gz.msgs.EntityFactory",
               "gz.msgs.Boolean", req, dry=dry)
    if not dry and rc != 0:
        print(f"提示: 创建 {model_name} 失败，请确认仿真世界名为 {world} "
              f"(gz service --list | grep /world/)")
        return False
    return True


def clear(world: str, dry: bool):
    for idx in (0, 1, 2):
        model_name = f"plan_corridor_{idx}"
        if dry:
            print(f"# remove {model_name}")
        _call(_remove_service(world), "gz.msgs.Entity", "gz.msgs.Boolean",
              f'name: "{model_name}" type: MODEL', dry=dry)


def demo(interval: float, world: str, dry: bool, loop: bool = False):
    while True:
        for idx in (0, 1, 2):
            clear(world, dry)
            time.sleep(0.3)
            print(f"--- 显示 lane {idx} ---")
            show_lane(idx, world, dry)
            time.sleep(interval)
        if not loop:
            break
    clear(world, dry)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--world", default=WORLD, help="gz world 名(默认 plan_world)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p_show = sub.add_parser("show")
    p_show.add_argument("--lane", type=int, required=True, choices=(0, 1, 2))
    p_show.add_argument("--dry-run", action="store_true")
    p_clr = sub.add_parser("clear")
    p_clr.add_argument("--dry-run", action="store_true")
    p_demo = sub.add_parser("demo")
    p_demo.add_argument("--interval", type=float, default=4.0)
    p_demo.add_argument("--loop", action="store_true")
    p_demo.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if args.cmd == "show":
        ok = show_lane(args.lane, args.world, args.dry_run)
        return 0 if ok else 1
    elif args.cmd == "clear":
        clear(args.world, args.dry_run)
    elif args.cmd == "demo":
        demo(args.interval, args.world, args.dry_run, args.loop)
    return 0


if __name__ == "__main__":
    sys.exit(main())
