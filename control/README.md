# 应急航道可行性测试(连续跟踪, 非正式控制)

只验证“沿规划走廊能走到港、能低速进港”。控制非本组职责, 港口内轻微碰碰不计较。

## ⚠ 话题与量纲
- 命令话题 = `/wamv/thrusters/left/thrust`、`/wamv/thrusters/right/thrust`
  (gz.msgs.Double, 单位=牛顿 N, 单桨约 ±2353 N)
- `/wamv/thrusters/left/thrust/ang_vel` 只是反馈, 发那里船不动;
  `gz topic -l` 不列命令话题(无发布者)是正常的。

## 行为
1. 巡航段: 纯跟踪 + 差速转向(v2基线, 曾三条航道都跑到港);
2. 距目标 <22m 进港段: 期望速度线性降到 ~0.22 m/s 低速滑进;
3. 进入 --stop-radius 后直接停车(不驻留、不反复调);
4. 越出水域紧急停车; 疑似卡住仅提示不动作。

过冲抑制(如仍振荡可调):
- 提高 `--lookahead`(默认6) → 跟踪更平滑
- 打开 `--yaw-damp`(默认0=关) → 抑制转过头, 若感觉船转不动就调小/关
- 降 `--gain`(默认300)

## 流程(务必先干净启动世界, 长时间跑过的实例会因多次复位/碰撞而退化)
1) 干净启动(固定 seed 999999 与世界一致):
       cd ~/vrx_ws/my_plan_world
       python3 run_plan_world.py --seed 999999
2) 另开终端, 先复位确认船在 (50,0):
       gz service -s /world/plan_world/control \
         --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
         --timeout 5000 --req 'reset { all: true }'
3) 跑(开跑前会自动把该 lane 走廊显示到 Gazebo, 无需手动 gz_models show;
   不想自动显示加 --no-display):
       python3 control/lane_test.py --lane 1     # 先直线港验证链路
       python3 control/lane_test.py --lane 2
       python3 control/lane_test.py --lane 0

参数: --lookahead --thrust-base(450) --gain(300) --approach-dist(22)
      --crawl-speed(0.22) --stop-radius(2.0) --approach-y(可选,停在口外)
