# 小船 lidar 移植 — 2026-09-09

## 已完成
- `boat_real/model.sdf` 新增 3D lidar：`lidar_sensor`（gpu_ray）
  - 水平 360°：1875 ray、垂直 16 线 ±15°、10 Hz、量程 0.1–130 m
  - 安装：船体中轴桅杆顶部 z≈0.60 m（高于甲板/波浪）
- `plan_world_bridge.launch.py` 已改指向 `boat_real/model.sdf`（备份 .bak_20260909）
- headless 实测：世界无报错，话题正常发布：
  - `/world/plan_world/model/wamv/link/wamv/base_link/sensor/lidar_sensor/scan`
  - `/world/plan_world/model/wamv/link/wamv/base_link/sensor/lidar_sensor/scan/points`

## 验证时发现的环境问题（重要，下一步处理层要解决）
自动验证“能否看到黑球”时，头less 逐帧统计被**近距离杂波**淹没，主要来源：
1. **自身**：相机桅杆 ~0.5 m、螺旋桨 ~1 m；
2. **水面/海底**：gz 的水只是 visual（无碰撞），向下打的光束会“穿透水面”打到
   z=-3 m 的海底（本世界海底很浅），在近处（约 2–12 m）产生大量回波；
   真海面上这些向下束只会打水/被吸收。
→ 直接对全帧 ranges 数“短距离点数”无法判定障碍物。
结论：以后做感知要用 **近水平束（水平环）或按点高度过滤（z>水面）**，不能全帧统计。

## 待办/建议（下一步“自感知+自避障”）
1. 写 `perception/` 模块：
   - 从 /scan(/points) 提取近水平环/高度过滤 → 二维障碍点；
   - 聚簇/栅格化 → 局部占用地图（球=大圆、港口墙=线段）；
2. 在线重规划：把 RRT*/走廊结果与“局部感知地图”融合，遇新障碍重规划；
3. 先在 GUI 里用 lidar 可视化人工确认“能看到黑球”，再定阈值。
