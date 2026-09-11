# -*- coding: utf-8 -*-
"""
把小船 lidar 命中点以"纯视觉静态模型"方式显示进 Gazebo(服务器端 create,
GUI 必显示——与 planner/gz_models.py 同一机制; 不用 /marker, 因为该 GUI
的 MarkerManager 不渲染 marker)。

只取近水平几条环(ring 7..10, 俯仰约 -1°..+5°), 避开 gz"水是 visual→打到
浅海底"的近距杂波; 每个命中点画一个绿色小球(半径 0.06, static, 无碰撞)。

用法(仿真 GUI 运行后, 另开终端):
  python3 control/lidar_models.py            # 约 1Hz 刷新快照, Ctrl-C 结束并清除
  python3 control/lidar_models.py --hz 2 --max-pts 400
"""
import argparse, base64, json, math, re, subprocess, time
import numpy as np

WORLD = "plan_world"
MODEL = "wamv"
PC = (f"/world/{WORLD}/model/{MODEL}/link/{MODEL}/base_link/sensor/"
      f"lidar_sensor/scan/points")
POSE = f"/world/{WORLD}/pose/info"
NS_NAME = "lidar_view"
SENSOR_Z = 0.60
RINGS = (7, 8, 9, 10)
RADIUS = 0.06   # 小球直径12cm，约为船长的1/10，避免喧宾夺主
COLOR = (0.15, 1.0, 0.45)

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
    if not m: return None
    for p in m.get("pose", []):
        if p.get("name") == MODEL:
            pos=p.get("position", {}); ori=p.get("orientation", {})
            qz=ori.get("z",0.0); qw=ori.get("w",1.0)
            return (pos.get("x",0.0), pos.get("y",0.0), pos.get("z",0.0),
                    2.0*math.atan2(qz,qw))
    return None

def hits_world(max_pts=400):
    m = grab_json(PC)
    pose = boat_pose()
    if not m or not pose: return []
    fields = {f.get("name"): f.get("offset", 0) for f in m.get("field", [])}
    ps = int(m.get("pointStep", 32))
    n = int(m.get("width",0)) * int(m.get("height",0))
    data = base64.b64decode(m["data"])
    buf = np.frombuffer(data, dtype=np.uint8).reshape(n, ps)
    def f32(name):
        off = fields.get(name)
        return buf[:, off:off+4].copy().view(np.float32)[:, 0] if off is not None else None
    x,y,z = f32("x"), f32("y"), f32("z")
    ring = buf[:, fields.get("ring",24):fields.get("ring",24)+2].copy().view(np.uint16)[:,0]
    if x is None: return []
    sel = np.isin(ring, list(RINGS)) & np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    r = np.hypot(x[sel], y[sel])
    keep = r > 0.8
    xs, ys, zs = x[sel][keep], y[sel][keep], z[sel][keep]
    if len(xs) > max_pts:
        step = int(math.ceil(len(xs)/max_pts)); xs, ys, zs = xs[::step], ys[::step], zs[::step]
    bx,by,bz,yaw = pose; c,s = math.cos(yaw), math.sin(yaw)
    out=[]
    for px,py,pz in zip(xs,ys,zs):
        out.append((bx + c*px - s*py, by + s*px + c*py, bz + SENSOR_Z + float(pz)))
    return out

def view_sdf(pts):
    mat = (f"<ambient>{COLOR[0]} {COLOR[1]} {COLOR[2]} 1</ambient>"
           f"<diffuse>{COLOR[0]} {COLOR[1]} {COLOR[2]} 1</diffuse>"
           "<specular>0.2 0.2 0.2 1</specular>")
    vis=[]
    for i,(x,y,z) in enumerate(pts):
        vis.append(f'<visual name="p{i}"><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>'
                   '<geometry><sphere><radius>%.2f</radius></sphere></geometry>'
                   f'<material>{mat}</material></visual>' % RADIUS)
    return ('<?xml version="1.0"?>\n<sdf version="1.9">\n'
            f'<model name="{NS_NAME}"><static>true</static><self_collide>false</self_collide>\n'
            '<link name="l">' + "".join(vis) + '</link>\n</model>\n</sdf>\n')

def esc(s): return s.replace("\\","\\\\").replace('"','\\"')
def call(service, reqtype, reptype, req):
    subprocess.run(["gz","service","-s",service,"--reqtype",reqtype,
                    "--reptype",reptype,"--timeout","5000","--req",req],
                   capture_output=True, text=True, timeout=20)

def remove():
    call(f"/world/{WORLD}/remove","gz.msgs.Entity","gz.msgs.Boolean",
         f'name: "{NS_NAME}" type: MODEL')
def create(sdf):
    oneline = esc(re.sub(r"\s+", " ", sdf).strip())
    call(f"/world/{WORLD}/create","gz.msgs.EntityFactory","gz.msgs.Boolean",
         f'sdf: "{oneline}"')

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--hz",type=float,default=1.0)
    ap.add_argument("--max-pts",type=int,default=400)
    args=ap.parse_args()
    print(f"实时显示 lidar 命中点(rings {RINGS}, ~{args.hz}Hz)。Ctrl-C 结束并清除。",flush=True)
    dt=1.0/args.hz
    try:
        while True:
            t0=time.time()
            pts=hits_world(args.max_pts)
            if pts:
                remove(); time.sleep(0.15); create(view_sdf(pts))
                print(f"刷新: {len(pts)} 个命中点",flush=True)
            time.sleep(max(0.0, dt-(time.time()-t0)))
    except KeyboardInterrupt:
        remove(); print("已清除。")

if __name__=="__main__":
    main()
