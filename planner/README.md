# planner —— 入港走廊规划器 v0（离线）

目标（用户已确认）：**不做运动控制**，只生成三条“平滑 + 无碰撞 + 可入港”的
**路径 + 走廊**；走廊比船略宽（默认半宽 1.5 m = 总宽 3.0 m），船理论上可沿走廊
笔直开进泊位（船头朝北），用于后续展示与交给控制组。

## 依赖
- Python3: numpy / scipy / matplotlib（本机已装）
- 不需要 ROS/Gazebo 参与即可规划；Gazebo 只用于最后展示走廊。

## 快速使用

```bash
cd ~/vrx_ws/my_plan_world

# 0) 一键：随机障碍 -> 规划 -> 出图 -> 打开仿真（推荐入口）
python3 run_plan_world.py                 # 全部随机
python3 run_plan_world.py --seed 2026     # 固定种子
python3 run_plan_world.py --no-launch     # 只出图不启动仿真

# 1) 只重新规划（读取当前 obstacles_meta.json；--seed 可复现）
python3 -m planner.plan_corridors --seed 2026

# 2) 输出
#    planner/planner_output.json  中心线/走廊左右边界/航路点/校验指标（供程序使用）
#    planner/planner_output.csv   三条航道一行汇总
#    planner/planner_overview.png 三条航道画在同一张俯视图（障碍+码头+走廊）

# 3) 在 Gazebo GUI 里显示走廊（先开世界，再另开终端执行）
#    世界启动：python3 run_plan_world.py
#    显示某条 / 清除 / 依次展示（推荐 gz_models：纯视觉模型，必定显示）：
python3 -m planner.gz_models show  --lane 0           # port_0
python3 -m planner.gz_models clear
python3 -m planner.gz_models demo  --interval 4       # 0->1->2 每 4 秒一条
#    若想用 gz marker(实验性，依赖 GUI 渲染)：
python3 -m planner.gz_markers show --lane 0
```

> 每次 `generate_obstacles.py`（重新随机生成障碍）之后，**必须重新运行一次
> `plan_corridors`**，因为真值文件已变。

## 参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `--half-width` | 1.5 m | 走廊半宽（总宽 3.0 m）；船半宽约 1.25 m，泊位内净宽 5 m，故半宽须 < 2.5 m |
| `--margin` | 2.0 m | 搜索期中心线相对障碍的额外裕度（搜索按 half-width+margin 避障） |
| `--lane-join-y` | 90 m | 入港直线段起点 y；从此处笔直朝北进港中心 (px,100) |
| `--attempts` | 6 | 单港口最多尝试次数（路径过尖/校验不过会换随机种子重试） |
| `--seed` | 随机 | 规划随机种子；不传则每次不同 |

## 管线
```
Informed RRT* (起点(50,0) -> (px,90)，障碍按 hw+margin 膨胀、码头整体不可入)
  -> 视线简化 shortcut
  -> 拼接笔直入港段 (px,90)->(px,100)（逐点加密）
  -> 三次 B 样条平滑（候选: 轻微平滑 / 严格插值 shortcut / 严格插值加密折线）
  -> 走廊 = 中心线 ± hw
  -> 校验: 中心线到障碍与码头墙体 >= hw（不足则换种子重试）
  -> 三个港口结果合并输出
```

## 输出字段（planner_output.json 内每个 port）
- `centerline` / `corridor_left` / `corridor_right`：世界系点列（0.5 m 间隔）
- `open_waypoints`：开放水域折线航路点（RRT* 简化后）
- `full_waypoints`：最终参与样条拟合的点（含笔直入港段）
- `length_m`、`max_curvature_1_per_m`（曲率 1/m，转半径=1/曲率）、
  `min_obstacle_clearance_m`、`min_wall_clearance_m`
- `smooth_mode`：实际采用的平滑候选；`smoothing_s`：样条平滑因子

## 在 Gazebo 显示走廊的技术说明（gz marker）
- 机制：gz-sim(Garden) GUI 侧的 MarkerManager 提供 marker 服务（默认 `/marker`,
  `/marker_array`），plan_world.sdf 的 GUI 已含该插件。走廊用
  `TRIANGLE_LIST`(半透明面片) + `LINE_STRIP`(中线/边界) 绘制，无物理、可动态增删。
- `gz_markers.py` 会自动用 `gz service --list` 探测服务名；若带世界名前缀
  （多世界时），加 `--world-prefix plan_world`。
- 若发送失败：确认仿真 GUI 正在运行；可先 `gz service --list | grep marker`
  看服务名是否如预期。
- `--dry-run` 只打印 gz service 命令不发送，便于排查。
