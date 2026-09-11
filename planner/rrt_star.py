# -*- coding: utf-8 -*-
"""
Informed RRT* —— 用于 plan_world 开放水域的二维路径搜索。

说明：
  - 障碍物用多边形列表表示；做碰撞时把每个多边形向外膨胀 clearance（即“走廊半宽 +
    安全裕度”），保证搜索出的中心线离障碍足够远。
  - 先跑普通 RRT* 得到首条解，之后把采样限制在以 start/goal 为焦点的椭圆内
    （Informed 采样），继续迭代改进，直到达到最大迭代数或长时间无改进。
  - 纯 numpy + 线性最近邻，规模很小（几百~几千节点），足够本场景使用。
"""
from __future__ import annotations

import math
import random
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import geometry as G

Point = Tuple[float, float]


class RRTStar:
    def __init__(self,
                 obstacles: Sequence[G.Poly],
                 clearance: float,
                 bounds=(0.0, 0.0, 100.0, 100.0),
                 max_step: float = 6.0,
                 rewire_radius: float = 12.0,
                 goal_radius: float = 2.0,
                 goal_sample_bias: float = 0.08,
                 seed: Optional[int] = None,
                 layers: Optional[Sequence[Tuple[Sequence[G.Poly],
                                                 float]]] = None):
        """layers: [(polys, clearance), ...] 各层用各自的 clearance 做碰撞。

        不传 layers 时等价于单层 (obstacles, clearance)，保持原行为。
        """
        self.obstacles = list(obstacles)
        self.clearance = clearance
        if layers is not None:
            self.checks = [(G.ObstacleField(pl), clr) for pl, clr in layers]
        else:
            self.checks = [(G.ObstacleField(self.obstacles), clearance)]
        self.xmin, self.ymin, self.xmax, self.ymax = bounds
        self.max_step = max_step
        self.rewire_radius = rewire_radius
        self.goal_radius = goal_radius
        self.goal_sample_bias = goal_sample_bias
        self.rng = random.Random(seed)
        self._rng_np = np.random.default_rng(seed)

    # ------------------------------------------------------------------ #
    def _sample(self, start: np.ndarray, goal: np.ndarray,
                c_best: Optional[float], informed: bool) -> np.ndarray:
        """采样一个点：informed=True 且已有解时在椭圆内采样，否则均匀采样。"""
        if informed and c_best is not None:
            c_min = float(np.linalg.norm(goal - start))
            if c_best - c_min > 1e-6:
                return self._sample_ellipse(start, goal, c_best, c_min)
        x = self.rng.uniform(self.xmin, self.xmax)
        y = self.rng.uniform(self.ymin, self.ymax)
        return np.array([x, y], float)

    def _sample_ellipse(self, start: np.ndarray, goal: np.ndarray,
                        c_best: float, c_min: float) -> np.ndarray:
        """以 start/goal 为焦点的椭圆内均匀采样（二维）。"""
        # 单位圆内均匀采样
        while True:
            v = self._rng_np.normal(size=2)
            v = v / max(np.linalg.norm(v), 1e-12)
            r = math.sqrt(self.rng.random())   # 均匀圆盘
            pt = v * r
            # 缩放成椭圆：长半轴 a=c_best/2，短半轴 b=sqrt(c_best^2-c_min^2)/2
            a = c_best / 2.0
            b = math.sqrt(max(c_best * c_best - c_min * c_min, 0.0)) / 2.0
            x = a * pt[0]
            y = b * pt[1]
            # 旋转到 start->goal 方向
            dx, dy = goal - start
            ang = math.atan2(dy, dx)
            c, s = math.cos(ang), math.sin(ang)
            wx = start[0] + c * x - s * y
            wy = start[1] + s * x + c * y
            if (self.xmin <= wx <= self.xmax and
                    self.ymin <= wy <= self.ymax):
                return np.array([wx, wy], float)

    def _collision_free(self, a: np.ndarray, b: np.ndarray) -> bool:
        for field, clr in self.checks:
            if not field.seg_clear((a[0], a[1]), (b[0], b[1]), clr):
                return False
        return True

    def _nearest(self, pts: np.ndarray, p: np.ndarray) -> int:
        d = (pts[:, 0] - p[0]) ** 2 + (pts[:, 1] - p[1]) ** 2
        return int(np.argmin(d))

    def _steer(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        d = float(np.linalg.norm(b - a))
        if d <= self.max_step:
            return b
        return a + (b - a) * (self.max_step / d)

    # ------------------------------------------------------------------ #
    def plan(self, start: Point, goal: Point,
             max_iterations: int = 2000,
             improve_iters: int = 250,
             informed: bool = True,
             return_tree: bool = False):
        """返回 dict: path/cost/found/...；找不到路径时 path=None。"""
        s = np.asarray(start, float)
        g = np.asarray(goal, float)

        # 直连可行就直接返回（很常见的 shortcut）
        if all(field.seg_clear(start, goal, clr)
               for field, clr in self.checks):
            return {"path": [tuple(start), tuple(goal)],
                    "cost": float(np.linalg.norm(g - s)), "found": True,
                    "iterations": 0, "tree": [] if return_tree else None}

        pts = [s]                    # 节点坐标列表
        parents: List[int] = [-1]   # 根节点父索引为 -1
        cost = [0.0]
        best_path: Optional[List[Point]] = None
        best_cost = math.inf
        goal_idx = -1
        n_since_improve = 0

        for it in range(max_iterations):
            use_goal = (self.rng.random() < self.goal_sample_bias)
            if use_goal:
                sample = g
            else:
                # Informed 采样只有在已经找到首条解之后才启用
                c_for_informed = best_cost if (informed and best_path is not None) else None
                sample = self._sample(s, g, c_for_informed, informed)

            # 找最近节点并朝采样点扩展
            ni = self._nearest(np.array(pts), sample)
            new_p = self._steer(pts[ni], sample)
            if not self._collision_free(pts[ni], new_p):
                continue

            # RRT*: 邻域内选最优父节点
            arr = np.array(pts)
            near_mask = ((arr[:, 0] - new_p[0]) ** 2 +
                         (arr[:, 1] - new_p[1]) ** 2) <= self.rewire_radius ** 2
            near_idx = np.nonzero(near_mask)[0].tolist()
            best_parent, best_parent_cost = ni, cost[ni] + float(
                np.linalg.norm(new_p - pts[ni]))
            for j in near_idx:
                if not self._collision_free(pts[j], new_p):
                    continue
                cand = cost[j] + float(np.linalg.norm(new_p - pts[j]))
                if cand < best_parent_cost:
                    best_parent, best_parent_cost = j, cand
            pts.append(new_p)
            parents.append(best_parent)
            cost.append(best_parent_cost)
            new_idx = len(pts) - 1

            # 重连邻域
            for j in near_idx:
                if j == new_idx or j == 0:   # 根节点不参与重连
                    continue
                cand = cost[new_idx] + float(np.linalg.norm(pts[j] - new_p))
                if cand < cost[j] and self._collision_free(new_p, pts[j]):
                    parents[j] = new_idx
                    cost[j] = cand

            # 是否到达目标
            if float(np.linalg.norm(new_p - g)) <= self.goal_radius \
                    and best_parent_cost < best_cost:
                best_cost = best_parent_cost
                goal_idx = new_idx
                best_path = self._trace(pts, parents, goal_idx)
                n_since_improve = 0
            else:
                n_since_improve += 1

            if best_path is not None and n_since_improve >= improve_iters:
                break

        found = best_path is not None
        result = {"path": best_path, "cost": best_cost if found else None,
                  "found": found, "iterations": it + 1}
        if return_tree:
            result["tree"] = ([(p[0], p[1]) for p in pts],
                              list(parents), list(cost))
        return result

    def _trace(self, pts, parents, idx) -> List[Point]:
        path = []
        while idx != -1:
            path.append((pts[idx][0], pts[idx][1]))
            idx = parents[idx]
        path.reverse()
        return path


def shortcut(path: Sequence[Point], obstacles: Sequence[G.Poly],
             clearance: float, step: float = 0.4) -> List[Point]:
    """贪心视线简化：在保持无碰撞(离障碍>=clearance)前提下删掉冗余转折点。"""
    if len(path) <= 2:
        return list(path)
    out = [tuple(path[0])]
    i = 0
    while i < len(path) - 1:
        # 尽量找从当前点能直连的最远点
        j = len(path) - 1
        while j > i + 1 and not G.seg_poly_clear(path[i], path[j],
                                                 obstacles, clearance, step):
            j -= 1
        out.append(tuple(path[j]))
        i = j
    return out


if __name__ == "__main__":
    # 简单自检：两个矩形障碍之间找一条路
    obs = [G.rect_poly(40, 50, 6, 20), G.rect_poly(60, 50, 6, 20)]
    r = RRTStar(obs, clearance=2.0, seed=7)
    res = r.plan((10, 10), (90, 10), max_iterations=1500, improve_iters=200)
    assert res["found"], "should find a path"
    assert G.polyline_poly_clear(res["path"], obs, 2.0)
    sp = shortcut(res["path"], obs, 2.0)
    assert G.polyline_poly_clear(sp, obs, 2.0)
    print(f"rrt* self-test OK, cost={res['cost']:.1f}, "
          f"waypoints={len(res['path'])} -> shortcut={len(sp)}")
