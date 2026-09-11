# 实验脚本存档（已被主版本吸收，保留供参考/回退研究）

- twopoint_experiment.py：港前“两控制点对接线 + R_min 圆弧”原型
- diverse_experiment.py：多样候选(末端朝向/路径差异) + 三方向对接原型

这两条思路的最终实现已并入主版本 `planner/plan_corridors.py` 的
`--docking-free`（多样候选 + 三方向两控制点对接）。
若要在当前目录运行这些旧脚本需自行调整导入(仅作代码参考)。
