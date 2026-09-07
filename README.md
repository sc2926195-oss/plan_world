# my_plan_world — 面向“规划”的 100 m x 100 m 水域世界

与 my_port_world / my_scan_world 平级，复用 VRX + Gazebo(gz-sim7) 仿真。

## 布局（ENU，z=0 为水面）
- 水域：x∈[0,100]，y∈[0,100]（开放水面，参考 scan 世界，无岸墙）
- 港口（3 座，浮动 U 形泊位码头，参考 scan 世界 dock 的漂浮形式）：
  - port_0 @ (0,100)、port_1 @ (50,100)、port_2 @ (100,100)
  - 泊位内净空 5.0 m(宽) x 7.0 m(深)，比 WAM-V(~4.9 m x 2.5 m)稍大；
  - 开口朝南(-Y)朝向水域；模型位于 port_berth/，本地原点=泊位中心，开口=本地+Y。
- WAM-V：生成于 (50, 0, 0.5)，yaw=+90°（船头指向 (50,100) 的 port_1）。
  船不系泊，启动后自由漂浮（本世界不含 platform/评分插件，无需 /vrx/release）。
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
- wamv_port/                       复用的 WAM-V 模型（副本，已移除 BallShooter 插件）
- port_berth/                      三座港口共用的 U 形浮动泊位模型
- plan_world_bridge.launch.py      载荷桥接（复用 vrx_gz.payload_bridges）
- generate_obstacles.py            障碍物生成器（默认每次随机）
- start_plan_world.sh              一键启动：随机障碍 + vrx_environment
- obstacles_meta.json / .csv       本次障碍真值（含世界系多边形）
- plan_world_overview.png          俯视示意图（示例布局）
- planner/                         规划器 v0（Informed RRT* + B样条 + 走廊，详见其 README）
- docs/                           学习笔记整理版等文档

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

# 3) 仿真起来后，另开终端显示走廊：
python3 -m planner.gz_models show --lane 0      # port_0（0/1/2 三条）
python3 -m planner.gz_models demo --interval 4  # 依次展示三条
python3 -m planner.gz_models clear              # 清除
```

说明：
- 本仓库的模型已用 `model://` 引用，启动脚本会自动把仓库目录加入
  `GZ_SIM_RESOURCE_PATH`，不需要改绝对路径；
- `run_plan_world.py` 会自动 source `/opt/ros/humble` 与 `~/vrx_ws/install/setup.bash`；
  若你的工作空间不在 `~/vrx_ws`，先手动 source 后执行（脚本会沿用当前环境）；
- 障碍每次启动随机；世界、规划、出图、显示使用同一 seed，保证一致；
- 汇报材料与思路总结见 `docs/planning_summary_for_report.md`，学习笔记见
  `docs/planning_notes_organized.md`。
