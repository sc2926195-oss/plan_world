# -*- coding: utf-8 -*-
"""
把小船 lidar 的近水平环实时画进 Gazebo GUI(marker, 与航道 marker 同机制)。

背景: gz 的 <visualize>true</visualize> 对 gpu_ray 不渲染; gz 的水是 visual,
向下束会打到浅海底造成 2-12m 近距杂波。所以这里只取"近水平环"(ring≈8,
俯仰约 +1°), 它飞在水面上方, 只回波到黑球/港口/岸上物, 正是以后避障要用的
2D 观测。真值校验/调参脚本另见 docs/lidar_port_20260909.md。

用法(仿真 GUI 运行后, 另开终端):
  python3 control/lidar_markers.py            # 实时刷新(3Hz), Ctrl-C 停止
  python3 control/lidar_markers.py --ring 8   # 指定环
"""
import argparse, base64, json, math, subprocess, time
import numpy as np

WORLD = "plan_world"
MODEL = "wamv"
PC = (f"/world/{WORLD}/model/{MODEL}/link/{MODEL}/base_link/sensor/"
      f"lidar_sensor/scan/points")
POSE = f"/world/{WORLD}/pose/info"
NS = "lidar_ring"
SENSOR_Z = 0.60          # lidar 相对模型原点的安装高度
MAX_CHARS = 80_000
SEG_GAP_DEG = 8.0        # 相邻点方位角超过此值则断线(避免连到开放水域)
COLOR = (0.10, 1.00, 0.40)   # 绿色

def grab_json(topic):
    p = subprocess.Popen(["gz","topic","-e","-t",topic,"--json-output"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                         text=True, bufsize=1)
    try:
        for line in p.stdout:
            m = json.loads(line)
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
            yaw = 2.0 * math.atan2(qz, qw)
            return (pos.get("x", 0.0), pos.get("y", 0.0),
                    pos.get("z", 0.0), yaw)
    return None

def ring_points(ring: int, max_pts: int = 2500):
    m = grab_json(PC)
    if not m:
        return []
    fields = {f.get("name"): f.get("offset", 0) for f in m.get("field", [])}
    ps = int(m.get("pointStep", 32)); n = int(m.get("width", 0)) * int(m.get("height", 0))
    data = base64.b64decode(m["data"])
    buf = np.frombuffer(data, dtype=np.uint8).reshape(n, ps)
    def f32(name):
        off = fields.get(name)
        if off is None: return None
        return buf[:, off:off+4].copy().view(np.float32)[:, 0]
    x, y, z = f32("x"), f32("y"), f32("z")
    ring_arr = buf[:, fields.get("ring", 24):fields.get("ring", 24)+2].copy().view(np.uint16)[:, 0] \
               if fields.get("ring") is not None else None
    if x is None or ring_arr is None:
        return []
    sel = ring_arr == ring
    fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    r = np.hypot(x, y)
    sel &= fin & (r > 0.5) & (r < 115.0)   # 丢掉无回波(NaN/inf)与自身近距点
    pts = [(float(x[i]), float(y[i]), float(z[i])) for i in np.where(sel)[0]]
    if len(pts) > max_pts:
        pts = pts[::int(math.ceil(len(pts)/max_pts))]
    return pts

def to_world(pose, pts):
    bx, by, bz, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for (px, py, pz) in pts:
        out.append((bx + c*px - s*py, by + s*px + c*py, bz + SENSOR_Z + pz))
    return out

def segments(world_pts, az_local):
    """按方位排序并断线, 返回若干世界系折线。"""
    order = sorted(range(len(world_pts)), key=lambda i: az_local[i])
    segs = []
    cur = []
    for i in order:
        if cur and az_local[i] - az_local[cur[-1]] > math.radians(SEG_GAP_DEG):
            if len(cur) >= 2: segs.append([world_pts[j] for j in cur])
            cur = []
        cur.append(i)
    if len(cur) >= 2:
        segs.append([world_pts[j] for j in cur])
    return segs

def _mat(rgb):
    r, g, b = rgb
    return (f"material {{ ambient {{ r: {r} g: {g} b: {b} a: 1 }} "
            f"diffuse {{ r: {r} g: {g} b: {b} a: 1 }} lighting: true }}")

def marker_text(seg_id, poly):
    t = (f"marker {{ ns: \"{NS}\" id: {seg_id} action: ADD_MODIFY "
         f"type: LINE_STRIP {_mat(COLOR)} ")
    for (x, y, z) in poly:
        t += f"point {{ x: {x:.2f} y: {y:.2f} z: {z:.2f} }} "
    return t + "}"

def send(reqs):
    req = "marker: [ " + " ".join(reqs) + " ]"
    for svc in (f"/world/{WORLD}/marker", "/marker", "/marker_array"):
        r = subprocess.run(["gz","service","-s",svc,"--reqtype","gz.msgs.Marker_V",
                            "--reptype","gz.msgs.Boolean","--timeout","3000",
                            "--req",req], capture_output=True, text=True, timeout=15)
        if r.returncode == 0 and "timeout" not in (r.stdout+r.stderr).lower():
            return True
    return False

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ring", type=int, default=8, help="近水平环序号 0..15(8≈+1°)")
    ap.add_argument("--hz", type=float, default=3.0)
    args = ap.parse_args()
    print(f"实时显示 lidar ring {args.ring} (约 {args.hz} Hz)。Ctrl-C 退出。", flush=True)
    dt = 1.0 / args.hz
    try:
        while True:
            t0 = time.time()
            pose = boat_pose()
            pts = ring_points(args.ring)
            if pose and pts:
                wp = to_world(pose, pts)
                az = [math.atan2(p[1], p[0]) for p in pts]
                reqs = [f"marker {{ ns: \"{NS}\" id: 0 action: DELETE_ALL }}"]
                for k, poly in enumerate(segments(wp, az)):
                    reqs.append(marker_text(k + 1, poly))
                # 拆包防超长
                chunk, out = [], []
                for r_ in reqs:
                    chunk.append(r_)
                    if sum(len(c) for c in chunk) > MAX_CHARS:
                        out.append(chunk); chunk = []
                if chunk: out.append(chunk)
                for c in out:
                    send(c)
            time.sleep(max(0.0, dt - (time.time() - t0)))
    except KeyboardInterrupt:
        send([f"marker {{ ns: \"{NS}\" id: 0 action: DELETE_ALL }}"])
        print("已清除。")

if __name__ == "__main__":
    main()
