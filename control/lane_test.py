# -*- coding: utf-8 -*-
"""
应急“航道可行性”连续跟踪测试（边开边调向，不停车转弯）。

只验证“沿规划走廊能走到港、能低速进港”, 不做正式控制:
  - 过冲抑制: 航向误差大时先收油(低速下转向更稳), 差速=比例-角速度阻尼,
    限幅+变化率限制, 避免“猛打→转过头→反向猛打”的振荡发散。
  - 快到港减速: 距目标 < --approach-dist 进入进港段, 期望速度线性降到
    --crawl-speed(~0.2m/s), 低速对直滑进泊位。
  - 进港即停车: 进入 --stop-radius 后直接停车(港口内轻微碰碰不计较),
    不再做驻留/反复调整。

【话题与量纲】命令话题 /wamv/thrusters/{left,right}/thrust, 值=牛顿 N。
/wamv/thrusters/.../ang_vel 只是反馈, 发那里船不动。

用法: 先开世界并复位到 (50,0), 然后
  python3 control/lane_test.py --lane 1
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORLD_DIR = HERE.parent
SCAN_WORLD = Path(os.environ.get("MY_SCAN_WORLD",
                                 str(Path.home() / "vrx_ws" / "my_scan_world")))
sys.path.insert(0, str(SCAN_WORLD / "autonomy"))
import wamv_common as wc          # 复用其位姿读取(仅基础件)

OUT_JSON = WORLD_DIR / "planner" / "planner_output.json"
_FEEDBACK_SUFFIXES = ("/ang_vel", "/force", "/enable_deadband", "/pos")


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def publish(topic: str, value: float, max_thrust: float = 2353.0) -> None:
    value = clamp(value, -max_thrust, max_thrust)
    try:
        subprocess.Popen(
            ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double",
             "-p", f"data: {value}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"[THRUSTER] 发布失败 {topic}: {e}")


def detect_thrust_topics(force_left=None, force_right=None):
    if force_left and force_right:
        return force_left, force_right, []
    cands = []
    try:
        out = subprocess.run(["gz", "topic", "-l"], capture_output=True,
                             text=True, timeout=5).stdout
        cands = [ln.strip() for ln in out.splitlines() if "thrust" in ln.lower()]
    except Exception:
        pass
    cmd_cands = [t for t in cands if not t.lower().endswith(_FEEDBACK_SUFFIXES)]

    def pick(side: str) -> str:
        for t in cmd_cands:
            if side in t.lower():
                return t
        return f"/wamv/thrusters/{side}/thrust"

    return (force_left or pick("left"),
            force_right or pick("right"), cands)


def load_centerline(lane: int):
    data = json.loads(OUT_JSON.read_text(encoding="utf-8"))
    port = next(p for p in data["ports"]
                if int(p["px"] / 50.0) == lane and p.get("success"))
    return port["centerline"]


def nearest_index(path, x, y):
    best, bd = 0, float("inf")
    for i, (px, py) in enumerate(path):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < bd:
            bd, best = d, i
    return best


def lookahead_point(path, i0, x, y, Ld):
    acc = 0.0
    px, py = path[i0]
    for i in range(i0 + 1, len(path)):
        nx, ny = path[i]
        acc += math.hypot(nx - px, ny - py)
        px, py = nx, ny
        if acc >= Ld:
            return nx, ny, i
    return path[-1][0], path[-1][1], len(path) - 1


def display_lane_in_gazebo(lane: int, world_dir: Path) -> bool:
    """开跑前把该 lane 的走廊显示到 Gazebo(等价于手动
    python3 -m planner.gz_models show --lane N)。失败只提示不阻塞。"""
    try:
        r = subprocess.run(
            [sys.executable, "-m", "planner.gz_models", "show",
             "--lane", str(lane)],
            cwd=str(world_dir), capture_output=True, text=True, timeout=120)
        if r.returncode == 0:
            print(f"✅ 已在 Gazebo 显示 lane {lane} 走廊 (planner.gz_models show)")
            return True
        err = (r.stderr or r.stdout or "").strip().splitlines()
        print("⚠ 自动显示走廊失败(可手动重试): "
              + (err[-1][:160] if err else f"exit {r.returncode}"))
    except Exception as e:
        print(f"⚠ 自动显示走廊异常: {e}")
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lane", type=int, required=True, choices=(0, 1, 2))
    ap.add_argument("--lookahead", type=float, default=6.0)
    ap.add_argument("--thrust-base", type=float, default=450.0)
    ap.add_argument("--gain", type=float, default=300.0, help="转向比例增益(N/rad)")
    ap.add_argument("--yaw-damp", type=float, default=0.0,
                    help="角速度阻尼(N per rad/s), 抑制转过头")
    ap.add_argument("--max-diff", type=float, default=9000.0, help="差速上限(N, 默认不限制)")
    ap.add_argument("--slew-rate", type=float, default=9000.0,
                    help="差速变化率上限(N/s), 防瞬间反向打满")
    ap.add_argument("--steer-sign", type=float, default=1.0)
    ap.add_argument("--max-thrust", type=float, default=2353.0)
    ap.add_argument("--rate-hz", type=float, default=15.0)
    ap.add_argument("--max-time", type=float, default=300.0)
    ap.add_argument("--topic-left", default="")
    ap.add_argument("--topic-right", default="")
    # 进港减速
    ap.add_argument("--approach-dist", type=float, default=22.0)
    ap.add_argument("--cruise-target", type=float, default=1.2)
    ap.add_argument("--crawl-speed", type=float, default=0.22)
    ap.add_argument("--speed-k", type=float, default=250.0)
    ap.add_argument("--la-shrink-dist", type=float, default=25.0)
    ap.add_argument("--la-min", type=float, default=3.5)
    # 停车
    ap.add_argument("--stop-radius", type=float, default=2.0,
                    help="距目标(泊位中心)小于该距离即直接停车(m)")
    ap.add_argument("--approach-y", type=float, default=None,
                    help="可选: 目标取中心线上第一个 y>=该值的点(停在口外)")
    ap.add_argument("--no-display", action="store_true",
                    help="开跑前不在 Gazebo 里自动显示该 lane 的走廊(默认自动显示)")
    args = ap.parse_args()

    wc.WORLD_NAME = "plan_world"
    wc.POSE_TOPIC = "/world/plan_world/pose/info"
    left_topic, right_topic, cands = detect_thrust_topics(
        args.topic_left or None, args.topic_right or None)
    if cands:
        print("检测到含 thrust 的话题(反馈/状态, 非命令):")
        for t in cands:
            print("   ", t)
    print(f"使用命令话题: L={left_topic}  R={right_topic}  (值=牛顿N)")

    path = load_centerline(args.lane)
    path = [(float(x), float(y)) for x, y in path]
    print(f"lane {args.lane}: 航道点数={len(path)}")
    goal = path[-1]
    if args.approach_y is not None:
        for x_, y_ in path:
            if y_ >= args.approach_y:
                goal = (x_, y_)
                break
    print(f"目标: ({goal[0]:.2f}, {goal[1]:.2f})  "
          f"({'泊位口外' if args.approach_y is not None else '中心线终点(泊位中心)'})")

    # 开跑前自动把该 lane 的走廊显示到 Gazebo(等价手动 gz_models show)
    if not args.no_display:
        display_lane_in_gazebo(args.lane, WORLD_DIR)

    reader = wc.GazeboPoseReader()
    reader.start()
    t0 = time.time()
    pose = None
    while pose is None and time.time() - t0 < 5:
        pose = reader.get()
        time.sleep(0.05)
    if pose is None:
        print("无法读取位姿, 请确认世界已启动")
        return 1

    print(f"开始跟踪 (基础推力={args.thrust_base}N, 增益={args.gain}, "
          f"阻尼={args.yaw_damp}, lookahead={args.lookahead}m)")
    dt = 1.0 / args.rate_hz
    prev = None
    v_sm = 0.0
    diff_prev = 0.0
    stuck_since = None
    verdict = "运行中"
    try:
        while time.time() - t0 < args.max_time:
            now = time.time()
            pose = reader.get()
            if pose is None:
                time.sleep(dt)
                continue
            speed = 0.0
            yaw_rate = 0.0
            if prev is not None:
                dtp = pose.timestamp - prev.timestamp
                if dtp > 1e-4:
                    speed = math.hypot(pose.x - prev.x, pose.y - prev.y) / dtp
                    yaw_rate = wrap(pose.yaw - prev.yaw) / dtp
            prev = pose
            i = nearest_index(path, pose.x, pose.y)
            dist_goal = math.hypot(goal[0] - pose.x, goal[1] - pose.y)

            # 越界保护
            if not (-3.0 <= pose.x <= 103.0 and -3.0 <= pose.y <= 103.0):
                verdict = f"FAIL(越出水域 ({pose.x:.1f},{pose.y:.1f}))"
                print(f"❌ {verdict}, 停车")
                publish(left_topic, 0.0)
                publish(right_topic, 0.0)
                break

            la = args.lookahead
            if dist_goal < args.la_shrink_dist:
                la = max(args.la_min, args.lookahead * dist_goal / args.la_shrink_dist)
            lx, ly, _ = lookahead_point(path, i, pose.x, pose.y, la)
            psi_d = math.atan2(ly - pose.y, lx - pose.x)
            err = wrap(psi_d - pose.yaw)

            # ===== 控制 =====
            err_slow = 1.0   # 默认不收油(v2基线); 如需抑制高速过冲可改小
            if dist_goal < args.approach_dist:
                # 进港段: 期望速度随距离降 -> 爬行, 误差大再乘收油系数
                v_des = args.crawl_speed + (args.cruise_target - args.crawl_speed) \
                    * clamp(dist_goal / args.approach_dist, 0.0, 1.0)
                base = (60.0 + args.speed_k * (v_des - v_sm)) * err_slow
                base = clamp(base, -180.0, args.thrust_base)
            else:
                base = args.thrust_base * err_slow
            # 2) 差速 = 比例 - 角速度阻尼(在转了就别再加, 防转过头)
            diff_cmd = args.steer_sign * (args.gain * err - args.yaw_damp * yaw_rate)
            diff_cmd = clamp(diff_cmd, -args.max_diff, args.max_diff)
            # 3) 差速变化率限幅(不能瞬间反向打满)
            diff = clamp(diff_cmd, diff_prev - args.slew_rate * dt,
                         diff_prev + args.slew_rate * dt)
            diff_prev = diff

            fl = clamp(base - diff, -args.max_thrust, args.max_thrust)
            fr = clamp(base + diff, -args.max_thrust, args.max_thrust)
            publish(left_topic, fl, args.max_thrust)
            publish(right_topic, fr, args.max_thrust)
            v_sm = 0.7 * v_sm + 0.3 * speed
            print(f"t={now-t0:6.1f} x={pose.x:7.2f} y={pose.y:7.2f} "
                  f"yaw={math.degrees(pose.yaw):7.1f} err={math.degrees(err):6.1f} "
                  f"spd={speed:4.2f} L={fl:6.0f} R={fr:6.0f} d={dist_goal:6.2f}",
                  flush=True)

            # 进港即停车: 不再折腾, 直接停这儿
            if dist_goal < args.stop_radius:
                publish(left_topic, 0.0)
                publish(right_topic, 0.0)
                time.sleep(1.0)
                publish(left_topic, 0.0)
                publish(right_topic, 0.0)
                verdict = "PASS(已入港停车)"
                print(f"✅ 已到达目标附近并停车: 船位=({pose.x:.2f},{pose.y:.2f}) "
                      f"yaw={math.degrees(pose.yaw):.1f}°")
                break

            # 卡住提示(仅提示, 不动作)
            pushing = max(abs(fl), abs(fr)) > 100.0
            if pushing and speed < 0.12 and abs(yaw_rate) < 0.25 \
                    and dist_goal > args.stop_radius + 4.0 and now - t0 > 8.0:
                if stuck_since is None:
                    stuck_since = now
                elif now - stuck_since >= 4.0:
                    print(f"⚠ 提示: 疑似卡住(推力{fl:.0f}/{fr:.0f}, 速度{speed:.2f}, "
                          f"距目标{dist_goal:.1f}m), 仅提示不动作", flush=True)
                    stuck_since = now
            else:
                stuck_since = None
            time.sleep(dt)

        if verdict == "运行中":
            verdict = "TIMEOUT(未到达)"
        print(f"结论: {'✅' if verdict.startswith('PASS') else '❌'} {verdict}")
    except KeyboardInterrupt:
        print("\n手动停止")
    finally:
        publish(left_topic, 0.0)
        publish(right_topic, 0.0)
        reader.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
