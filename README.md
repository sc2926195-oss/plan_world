# my_plan_world — 面向“规划”的 100 m x 100 m 水域世界

与 my_port_world / my_scan_world 平级，复用 VRX + Gazebo(gz-sim7) 仿真。

## 布局（ENU，z=0 为水面）
- 水域：x∈[0,100]，y∈[0,100]（开放水面，参考 scan 世界，无岸墙）
- 港口（3 座，浮动 U 形泊位码头，参考 scan 世界 dock 的漂浮形式）：
  - port_0 @ (0,100)、port_1 @ (50,100)、port_2 @ (100,100)
  - **泊位内净空 1.5 m(宽) x 2.0 m(深)，墙厚/墙高 0.5 m（实战尺寸，2026-09-10 起）**；
  - 开口朝南(-Y)朝向水域；模型位于 port_berth/，本地原点=泊位中心，开口=本地+Y。
- 船（boat_real/）：等比新建的小型双体 USV，尺寸 1.20 m x 0.85 m x ~0.45 m，
  与实战船一致；生成于 (50, 0, 0.15)，yaw=+90°（船头指向 (50,100) 的 port_1）。
  仿真内实体名仍叫 wamv、推力话题仍为 /wamv/thrusters/{left,right}/thrust，
  因此规划/控制脚本无需改动。船不系泊，启动后自由漂浮。
  感知载荷：前视 RGB 相机(front_camera_sensor) + 360°x16线 lidar(lidar_sensor)。
  注意：gz 水是 visual，向下 lidar 束会打到浅海底(2-12m 杂波)，做感知要按
  近水平束/高度过滤(见 docs/lidar_port_20260909.md)。
- 障碍物：10 个静态漂浮障碍（正三角形或正方形，占地约 10 m^2，高 0.6 m）
  - **每次启动世界前由 generate_obstacles.py 重新随机生成**（默认随机种子；
    传 --seed 可复现）。写在世界文件的 AUTO-GENERATED OBSTACLES 标记之间；
    本次布局真值见 obstacles_meta.json / obstacles_meta.csv。
- 波浪：coast_waves + 波浪参数发布(gain=0.3，与 scan 世界一致)，波浪可见。

## 启动（推荐）
```bash
# 一键：先随机生成障碍，再用你惯用的 vrx_environment 启动（带 GUI、自动开跑）
~/vrx_ws/my_plan_world/start_plan_world.sh
# 固定布局复现：
~/vrx_ws/my_plan_world/start_plan_world.sh --seed 20260906
```
等价于手动两步：
```bash
python3 ~/vrx_ws/my_plan_world/generate_obstacles.py            # 每次随机
ros2 launch vrx_gz vrx_environment.launch.py \
  world:="$(realpath ~/vrx_ws/my_plan_world/plan_world.sdf)"
```
要点：
- 直接用 `gz sim xxx.sdf` 时**必须加 `-r`**（否则仿真暂停：看不到波浪、船不下落）；
  vrx_environment 默认 paused=False，等价自动 `-r`。
- vrx_environment 不传 config_file/robot 时**不会重复生成船**（船已写在世界 SDF 里）；
  对它不认识的 world 名只会额外桥接 clock/wind 等通用话题，无影响。
- 传感器/载荷话题桥接（lidar/scan 等）如需要另开：
```bash
ros2 launch ~/vrx_ws/my_plan_world/plan_world_bridge.launch.py
```
控制脚本里世界名常量请改为 `WORLD_NAME = "plan_world"`；
位姿话题为 `/world/plan_world/pose/info`。

## 重新生成障碍物（单独）
```bash
python3 ~/vrx_ws/my_plan_world/generate_obstacles.py             # 随机
python3 ~/vrx_ws/my_plan_world/generate_obstacles.py --seed 123  # 固定复现
```
会同步更新 plan_world.sdf 与 obstacles_meta.json/csv（真值供规划使用）。


## 规划器（新增，v0，离线只生成路径+走廊）

见 [planner/README.md](planner/README.md)。用法：

```bash
cd ~/vrx_ws/my_plan_world
# 一键：随机障碍 -> 规划三条航道+出图 -> 打开 Gazebo 仿真(推荐)
python3 run_plan_world.py                # 或 --seed 2026 固定复现
python3 run_plan_world.py --no-launch    # 只生成+规划，不启动仿真

# 世界 GUI 起来后，另开终端显示/撤销/依次展示走廊：
python3 -m planner.gz_models show --lane 0       # port_0(推荐)
python3 -m planner.gz_models clear               # 撤销
python3 -m planner.gz_models demo --interval 4   # 依次展示三条
```

注意：`run_plan_world.py` 保证“世界里的障碍 == 规划用的真值/图片”同一个 seed；
若改用 `start_plan_world.sh` 启动，它每次会重新随机障碍，需让它的 seed 与
`plan_corridors` 一致，否则图与仿真里的障碍会对不上。

## 文件
- plan_world.sdf                   主世界（由 scan 世界骨架裁剪：GUI/物理/水/船/港口/障碍）
- boat_real/                       新建的小型双体船模型（120x85x45 cm，替代 WAM-V）
- wamv_port/                        旧的 WAM-V 模型（保留，不再被世界引用，可回退）
- port_berth/                      三座港口共用的 U 形浮动泊位模型
- plan_world_bridge.launch.py      载荷桥接（复用 vrx_gz.payload_bridges）
- generate_obstacles.py            障碍物生成器（默认每次随机）
- start_plan_world.sh              一键启动：随机障碍 + vrx_environment
- obstacles_meta.json / .csv       本次障碍真值（含世界系多边形）
- plan_world_overview.png          俯视示意图（示例布局）
- planner/                         规划器 v0（Informed RRT* + B样条 + 走廊，详见其 README）
- docs/                           学习笔记整理版等文档

## 港口尺寸（RobotX 2026 实战对齐调查，2026-09-09）
结论：官方没有给出泊位“净宽/净深”的具体数字，无法精确对齐，只能按约束反推。

- 官方只写：三座 docking bay 由主办国“标准 docking cubes”现场拼装；
  泊位内放一个白色“building”结构（带两个 25cm 方孔 + 底部 Color Indicator 红/绿灯）。
  - Task 3 描述：https://robonation.gitbook.io/robotx-2026-team-handbook/section-3-autonomy-challenge/3.3-mission-task-descriptions/3.3.4-mission-task-3-coordinated-logistics
  - Docking Bay Structure 搭建图（尺寸只以图片给出，含 building 面板尺寸，无泊位净尺寸）：
    https://robonation.gitbook.io/robotx-2026-team-handbook/robotx-2026-or-task-build-guides/docking-bay-structure
- 可用的硬约束：参赛 USV 必须能放进 2m x 1m x 1m 箱子、总重 ≤60kg
  （https://robonation.gitbook.io/robotx-2026-team-handbook/section-5-rules-and-requirements/5.3-system-requirements/5.3.1-usv-requirements）
  → 泊位净宽至少要 >1m（最大船宽），净深至少要 >2m（最大船长）；实际还要留操纵余量。
- 我们 1.2x0.85m 的小船远小于 2x1m 上限，所以 5x7m 泊位相对它“过于宽松”，
  入港/走廊问题会偏简单，与实战不符；建议下一步单独出一个“缩港版”对比实验，
  例如内净 ~3m(宽) x ~4m(深)，先跑规划看走廊/入港是否仍成立，再决定是否实装。

## lidar 可视化（小船 boat_real，推荐用模型方式）
原理：gz 的 `<visualize>` 对 gpu_ray 不渲染、`/marker` 在此 GUI 也不显示；
改用 `control/lidar_models.py`——把 lidar 命中点以“服务器端纯视觉静态模型”
（create，绿色小球 r=0.06，无碰撞）注入仿真，GUI 必显示。

```bash
# 1) 先启动世界 GUI（一个终端）
ros2 launch vrx_gz vrx_environment.launch.py \
  world:="$(realpath ~/vrx_ws/my_plan_world/plan_world.sdf)"

# 2) 另开终端，运行实时 lidar 显示（默认 1Hz，绿点跟随船位）
cd ~/vrx_ws/my_plan_world
python3 control/lidar_models.py                # 或 --hz 2 --max-pts 250
# Ctrl-C 结束并自动清除绿点
```
说明：只画近水平几条环（ring 7..10，约 -1°..+5°），避开 gz“水是 visual→向下束打到浅海底”的近距杂波；
每个命中点一个小绿球（直径 12cm≈船长 1/10）。黑球/港墙上有绿点贴附即表示 lidar 命中。
（control/lidar_markers.py 是同思路的 marker 版，但此 GUI 的 MarkerManager 不渲染 marker，已弃用。）

## 在线避障重规划（混合结构，2026-09-10）
全局 RRT*+B样条 保持不变；局部被新感知障碍挡住时，用栅格 A* 绕行 + B样条拼回。
```bash
cd ~/vrx_ws/my_plan_world
python3 -m planner.replan --lane 1 --s-curve    # 出图: planner/replan_demo.png
```
详见 [planner/README.md](planner/README.md) 与 [docs/hybrid_replan_design.md](docs/hybrid_replan_design.md)。

## 在线任务（去作弊版）+ 停船/复位
```bash
# 生成"障碍+绿港(同种子可复现)"
python3 generate_obstacles.py --seed 100      # seed 决定布局与哪个港为绿
# 启动世界(你的常规命令)后：
python3 control/online_mission.py --order 0,1,2   # 默认顺序=最短扫掠(约189m)

# 停船 / 复位（重要：复位前必须先停船）
python3 control/stop_boat.py
python3 control/reset_boat.py                 # 先停船再复位(50,0,90°)
```
Gazebo 里：青色小球=实时规划路径，红圈=lidar 感知到的障碍；控制台看
[MISSION]/[COLOR]/[REPORT]。详见 docs/online_mission_20260910.md。

## 在其他电脑复现 / 协作者快速开始

**前置环境**（与开发机一致）：
- Ubuntu 22.04 + ROS 2 Humble + Gazebo Garden(gz-sim7) + VRX（osrf/vrx 的 humble 分支）
- 已按 VRX 方式建好 colcon 工作空间（建议就叫 `~/vrx_ws`），`colcon build` 完成并能用
  `ros2 launch vrx_gz vrx_environment.launch.py` 起 VRX 世界；
- Python: `numpy scipy matplotlib`。

**步骤**：
```bash
# 1) 克隆到 ~/vrx_ws/my_plan_world（脚本约定：仓库位于工作空间下的 my_plan_world）
cd ~/vrx_ws
git clone https://github.com/sc2926195-oss/plan_world.git my_plan_world
cd my_plan_world

# 2) 一键：随机生成障碍 -> 规划三条航道并出图 -> 打开 Gazebo 仿真
python3 run_plan_world.py
#    想看固定布局：python3 run_plan_world.py --seed 2026
#    只出图不启动：python3 run_plan_world.py --no-launch

# 3) 仿真起来后，另开终端在 GUI 里看走廊 / 看 lidar 命中点
python3 -m planner.gz_models show --lane 0      # port_0（0/1/2 三条）
python3 -m planner.gz_models demo --interval 4  # 依次展示三条
python3 -m planner.gz_models clear              # 清除
python3 control/lidar_models.py                 # lidar 命中点绿球（Ctrl-C 自动清除）

# 4) 在线任务（去作弊版）：依次拜访候选港，途中障碍自动绕行
python3 control/online_mission.py --order 0,1,2
#    停船 / 复位（复位前必须先停船）
python3 control/stop_boat.py && python3 control/reset_boat.py
```

说明：
- 本仓库的模型已用 `model://` 引用，启动脚本会自动把仓库目录加入
  `GZ_SIM_RESOURCE_PATH`，不需要改绝对路径；
- `run_plan_world.py` 会自动 source `/opt/ros/humble` 与 `~/vrx_ws/install/setup.bash`；
  若你的工作空间不在 `~/vrx_ws`，先手动 source 后执行（脚本会沿用当前环境）；
- 障碍每次启动随机；世界、规划、出图、显示使用同一 seed，保证一致；
- 汇报材料与思路总结见 `docs/planning_summary_for_report.md`，学习笔记见
  `docs/planning_notes_organized.md`；
- 最新进展：lidar 感知（`docs/lidar_port_20260909.md`）、在线避障重规划
  （`docs/hybrid_replan_design.md`）、在线任务（`docs/online_mission_20260910.md`）、
  船体颜色识别（`docs/boat_color_test_20260909.md`）。
