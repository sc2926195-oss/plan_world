# -*- coding: utf-8 -*-
"""
把规划结果(planner_output.json)以 gz-sim Marker 的形式显示到 Gazebo GUI。

特性：
  - 动态增删：可单独显示/清除某一条航道(port_0/port_1/port_2)，适合“依次展示三条航道”。
  - 不参与物理，纯 GUI 渲染。plan_world.sdf 的 GUI 已含 MarkerManager 插件。
  - 每条航道画：半透明走廊面片(TRIANGLE_LIST) + 中线/左右边界(LINE_STRIP)。

注意：
  Linux 对单个命令行参数有 128 KB 上限，因此走廊面片被切成多个小 marker
  (不同 id) 分批发送，避免 "Argument list too long"。

用法(仿真 GUI 运行后，另开终端)：
  python3 -m planner.gz_markers show   --lane 0          # 显示 port_0 航道
  python3 -m planner.gz_markers clear                    # 清除所有航道
  python3 -m planner.gz_markers demo   --interval 4      # 依次展示 0->1->2，每4秒一条
  python3 -m planner.gz_markers show   --lane 1 --dry-run  # 只打印命令不发送
"""
from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Sequence, Tuple

from . import geometry as G

HERE = Path(__file__).resolve().parent
OUT_JSON = HERE / "planner_output.json"

NS = "plan_corridor"
Z_FILL = 1.00          # 走廊面片高度（明显高于波浪，保证可见）
Z_LINE = 1.10          # 线条略高于面片，避免闪烁
COLORS = {
    0: (0.85, 0.16, 0.16),   # port_0 红
    1: (0.20, 0.72, 0.22),   # port_1 绿
    2: (0.12, 0.47, 0.71),   # port_2 蓝
}
START = (50.0, 0.0)
DISPLAY_STEP = 2.0           # 显示用抽稀步长(米)，越大数据量越小
MAX_REQ_CHARS = 90_000       # 单次 gz service --req 字符上限(留余量给 128KB)
FILL_POINTS_PER_MARKER = 150 # 每个面片 marker 的顶点数预算(6 个点=1个四边形=2个三角形)

Point = Tuple[float, float]


# --------------------------------------------------------------------------- #
# protobuf text 生成
# --------------------------------------------------------------------------- #
def _fmt(v: float) -> str:
    return f"{v:.3f}"


def _vec3(p: Point, z: float) -> str:
    return f"point {{ x: {_fmt(p[0])} y: {_fmt(p[1])} z: {_fmt(z)} }}"


def _material(rgb, a: float) -> str:
    r, g, b = rgb
    return (f"material {{ "
            f"ambient {{ r: {r:.3f} g: {g:.3f} b: {b:.3f} a: {a:.3f} }} "
            f"diffuse {{ r: {r:.3f} g: {g:.3f} b: {b:.3f} a: {a:.3f} }} "
            f"lighting: true }}")


def _line_marker(mid: int, rgb, points: Sequence[Point], z: float) -> str:
    s = (f"marker {{ ns: \"{NS}\" id: {mid} action: ADD_MODIFY "
         f"type: LINE_STRIP {_material(rgb, 1.0)} ")
    for p in points:
        s += _vec3(p, z) + " "
    return s + "}"


def _fill_marker(mid: int, rgb, tri_points: Sequence[Point]) -> str:
    s = (f"marker {{ ns: \"{NS}\" id: {mid} action: ADD_MODIFY "
         f"type: TRIANGLE_LIST {_material(rgb, 0.45)} ")
    for p in tri_points:
        s += _vec3(p, Z_FILL) + " "
    return s + "}"


def _delete_all_marker() -> str:
    return ("marker { ns: \"plan_corridor\" id: 0 action: DELETE_ALL } "
            "marker { ns: \"plan_common\" id: 0 action: DELETE_ALL }")


def _strip_fill_markers(center: Sequence[Point], left: Sequence[Point],
                        right: Sequence[Point], base_id: int,
                        rgb) -> List[str]:
    """把走廊条带切成多个 TRIANGLE_LIST marker(不同 id)，每个顶点数受限。"""
    n = len(center)
    markers = []
    k = max(1, FILL_POINTS_PER_MARKER // 6)     # 每个 marker 覆盖的区间数
    mid = 0
    start = 0
    while start < n - 1:
        end = min(start + k, n - 1)
        tri: List[Point] = []
        for i in range(start, end):
            tri += [left[i], right[i], right[i + 1],
                    left[i], right[i + 1], left[i + 1]]
        markers.append(_fill_marker(base_id + mid, rgb, tri))
        mid += 1
        start = end
    return markers


def lane_marker_texts(lane: dict, idx: int) -> List[str]:
    """返回该航道的一批 Marker_V 文本（每条一个 marker，均小于命令行上限）。"""
    rgb = COLORS[idx]
    raw_c = [tuple(p) for p in lane["centerline"]]
    raw_l = [tuple(p) for p in lane["corridor_left"]]
    raw_r = [tuple(p) for p in lane["corridor_right"]]
    # 三条折线同源于同一 centerline，用同一组索引抽稀，保证点数一致
    keep = G.decimate_indices(raw_c, DISPLAY_STEP)
    center = [raw_c[i] for i in keep]
    left = [raw_l[i] for i in keep]
    right = [raw_r[i] for i in keep]
    base = idx * 100

    msgs: List[str] = []
    # 走廊面片（分块）
    msgs += _strip_fill_markers(center, left, right, base + 0, rgb)
    # 左右边界 + 中线
    msgs.append(_line_marker(base + 50, rgb, left, Z_LINE))
    msgs.append(_line_marker(base + 51, rgb, right, Z_LINE))
    msgs.append(_line_marker(base + 52, (1.0, 1.0, 1.0), center, Z_LINE))

    texts = [f"marker: [ {m} ]" for m in msgs]
    return texts


# --------------------------------------------------------------------------- #
# 发送
# --------------------------------------------------------------------------- #
def _service_list() -> List[str]:
    try:
        r = subprocess.run(["gz", "service", "--list"], capture_output=True,
                           text=True, timeout=5)
        return [ln.strip() for ln in r.stdout.splitlines() if ln.strip()]
    except Exception:
        return []


def _pick_services(world_prefix: str):
    cand_marker = []
    cand_array = []
    if world_prefix:
        cand_marker.append(f"/world/{world_prefix}/marker")
        cand_array.append(f"/world/{world_prefix}/marker_array")
    cand_marker.append("/marker")
    cand_array.append("/marker_array")
    svc = _service_list()
    for s in svc:
        if s.endswith("/marker_array") or s.endswith("/marker_array/"):
            cand_array.insert(0, s)
        elif s.endswith("/marker") or s.endswith("/marker/"):
            cand_marker.insert(0, s)
    return cand_marker, cand_array, svc


def _call(service: str, reqtype: str, reptype: str, text: str,
          timeout_ms: int = 4000, dry: bool = False) -> Optional[str]:
    cmd = ["gz", "service", "-s", service,
           "--reqtype", reqtype, "--reptype", reptype,
           "--timeout", str(timeout_ms), "--req", text]
    if dry:
        print(" ".join(shlex.quote(c) for c in cmd))
        return None
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    out = (r.stdout or "") + (r.stderr or "")
    if r.returncode != 0:
        print(f"[warn] gz service 调用失败({service}): {out.strip()[:200]}")
        return out
    return out


def send_marker_texts(texts: List[str], world_prefix: str, dry: bool) -> bool:
    markers, arrays, svc = _pick_services(world_prefix)
    if not arrays and not svc:
        print("提示: 未检测到任何 gz service(仿真 GUI 是否已启动？)。"
              "请先运行 run_plan_world.py / start_plan_world.sh 再执行本脚本。")
        return False
    ok_any = False
    for text in texts:
        if len(text) > MAX_REQ_CHARS:
            print(f"[warn] 单条请求过长({len(text)}> {MAX_REQ_CHARS})，已跳过")
            continue
        sent = False
        for svc_name in arrays:
            out = _call(svc_name, "gz.msgs.Marker_V", "gz.msgs.Boolean",
                        text, dry=dry)
            if dry or (out is not None and "失败" not in out and
                       "timed out" not in out.lower()):
                sent = True
                break
        ok_any = ok_any or sent
    if not ok_any and not dry:
        print("提示: marker_array 服务不可用，可尝试 --world-prefix plan_world "
              "或先 `gz service --list | grep marker` 确认服务名。")
    return ok_any


def clear(world_prefix: str, dry: bool):
    send_marker_texts(["marker: [ " + _delete_all_marker() + " ]"],
                      world_prefix, dry)


def show_lane(idx: int, world_prefix: str, dry: bool):
    data = json.loads(OUT_JSON.read_text(encoding="utf-8"))
    lane = next(p for p in data["ports"]
                if p["success"] and int(p["px"] / 50.0) == idx)
    texts = lane_marker_texts(lane, idx)
    if dry:
        print(f"# lane {idx}: 共 {len(texts)} 条 marker 请求")
    send_marker_texts(texts, world_prefix, dry)


def demo(interval: float, world_prefix: str, dry: bool, loop: bool = False):
    while True:
        for idx in (0, 1, 2):
            clear(world_prefix, dry)
            time.sleep(0.3)
            print(f"--- 显示 lane {idx} ---")
            show_lane(idx, world_prefix, dry)
            time.sleep(interval)
        if not loop:
            break
    clear(world_prefix, dry)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p_show = sub.add_parser("show", help="显示某条航道")
    p_show.add_argument("--lane", type=int, required=True, choices=(0, 1, 2))
    p_show.add_argument("--world-prefix", default="",
                        help="如 plan_world -> 尝试 /world/plan_world/marker")
    p_show.add_argument("--dry-run", action="store_true")
    p_clr = sub.add_parser("clear", help="清除全部航道")
    p_clr.add_argument("--world-prefix", default="")
    p_clr.add_argument("--dry-run", action="store_true")
    p_demo = sub.add_parser("demo", help="依次展示三条航道")
    p_demo.add_argument("--interval", type=float, default=4.0)
    p_demo.add_argument("--world-prefix", default="")
    p_demo.add_argument("--loop", action="store_true")
    p_demo.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if args.cmd == "show":
        show_lane(args.lane, args.world_prefix, args.dry_run)
    elif args.cmd == "clear":
        clear(args.world_prefix, args.dry_run)
    elif args.cmd == "demo":
        demo(args.interval, args.world_prefix, args.dry_run, args.loop)
    return 0


if __name__ == "__main__":
    sys.exit(main())
