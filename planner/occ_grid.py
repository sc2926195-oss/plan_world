# -*- coding: utf-8 -*-
"""
在线避障用的二维占据栅格 + A*（局部绕行）。

设计（对应用户确认的混合结构）：
  - 全局航道仍由 RRT*+B样条给出（planner/plan_corridors.py）；
  - 本模块只负责“半路 lidar 新发现障碍 -> 在走廊附近做局部确定性绕行”：
        sensed circles  --(inflate)-->  栅格 blocked/soft  -->  A*  -->  折线
    绕行折线再由 replan.py 做 B样条平滑 + 曲率/安全校验。

障碍建模：lidar 近水平环回波在球上是一段圆弧，拟合后即“圆(cx,cy,r)”。
           栅格把每个圆按 (r + inflation) 标为 blocked，外圈 soft_margin 标为
           高代价(soft)，让 A* 尽量离障碍远一点、路径更顺。

约定：世界系 ENU，水区域 x∈[0,100], y∈[0,100]；cell 默认 0.5 m。
"""
from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]


class Circle:
    """感知到的圆形障碍（理想化：黑球拟合圆）。"""

    __slots__ = ("x", "y", "r", "tag")

    def __init__(self, x: float, y: float, r: float, tag: str = ""):
        self.x, self.y, self.r, self.tag = float(x), float(y), float(r), tag

    def dist_surface(self, p: Point) -> float:
        """点到圆面的距离：内部为负。"""
        return math.hypot(p[0] - self.x, p[1] - self.y) - self.r

    def as_tuple(self):
        return (self.x, self.y, self.r, self.tag)


# --------------------------------------------------------------------------- #
# 圆相关的几何校验（供 replan 使用）
# --------------------------------------------------------------------------- #
def min_clearance(pts: Sequence[Point], circles: Sequence[Circle]) -> float:
    """点列到所有圆面的最小距离（无圆 -> inf）。"""
    if not circles:
        return float("inf")
    best = float("inf")
    for p in pts:
        for c in circles:
            d = c.dist_surface(p)
            if d < best:
                best = d
    return best


def seg_clear(a: Point, b: Point, circles: Sequence[Circle],
              clearance: float, step: float = 0.4) -> bool:
    """线段 [a,b] 是否离所有圆面 >= clearance（抽样判断）。"""
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(math.ceil(d / step)) + 1)
    for i in range(n + 1):
        t = i / n
        p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        for c in circles:
            if c.dist_surface(p) < clearance - 1e-9:
                return False
    return True


def polyline_clear(pts: Sequence[Point], circles: Sequence[Circle],
                   clearance: float, step: float = 0.4) -> bool:
    return all(seg_clear(a, b, circles, clearance, step)
               for a, b in zip(pts[:-1], pts[1:]))


def shortcut_circles(path: Sequence[Point], circles: Sequence[Circle],
                     clearance: float, step: float = 0.4) -> List[Point]:
    """视线简化：在保持离圆 >= clearance 前提下删冗余点。"""
    if len(path) <= 2:
        return [tuple(p) for p in path]
    out = [tuple(path[0])]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not seg_clear(path[i], path[j], circles,
                                          clearance, step):
            j -= 1
        out.append(tuple(path[j]))
        i = j
    return out


# --------------------------------------------------------------------------- #
# 栅格
# --------------------------------------------------------------------------- #
class OccupancyGrid:
    def __init__(self,
                 bounds: Tuple[float, float, float, float] = (0, 0, 100, 100),
                 cell: float = 0.5,
                 inflation: float = 2.5,
                 soft_margin: float = 1.5,
                 soft_cost: float = 3.0):
        """
        inflation : 圆外再膨胀多少米(含船半宽+安全裕度)才 blocked
        soft_margin: blocked 外再软的"别贴太近"范围
        soft_cost : soft 区域每米附加代价(>0 让 A* 倾向远离障碍)
        """
        self.xmin, self.ymin, self.xmax, self.ymax = map(float, bounds)
        self.cell = float(cell)
        self.inflation = float(inflation)
        self.soft_margin = float(soft_margin)
        self.soft_cost = float(soft_cost)
        self.nx = max(1, int(math.ceil((self.xmax - self.xmin) / self.cell)))
        self.ny = max(1, int(math.ceil((self.ymax - self.ymin) / self.cell)))
        self.blocked = np.zeros((self.ny, self.nx), dtype=bool)
        self.soft = np.zeros((self.ny, self.nx), dtype=bool)

    # ---------- 坐标 ---------- #
    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        i = int((x - self.xmin) / self.cell)
        j = int((y - self.ymin) / self.cell)
        return i, j

    def cell_to_world(self, i: int, j: int) -> Point:
        return (self.xmin + (i + 0.5) * self.cell,
                self.ymin + (j + 0.5) * self.cell)

    def in_grid(self, i: int, j: int) -> bool:
        return 0 <= i < self.nx and 0 <= j < self.ny

    def in_bounds_xy(self, x: float, y: float) -> bool:
        return (self.xmin <= x <= self.xmax) and (self.ymin <= y <= self.ymax)

    # ---------- 标障碍 ---------- #
    def add_circle(self, c: Circle) -> None:
        r_block = c.r + self.inflation
        r_soft = r_block + self.soft_margin
        i0, j0 = self.world_to_cell(c.x - r_soft, c.y - r_soft)
        i1, j1 = self.world_to_cell(c.x + r_soft, c.y + r_soft)
        for j in range(max(0, j0), min(self.ny - 1, j1) + 1):
            for i in range(max(0, i0), min(self.nx - 1, i1) + 1):
                wx, wy = self.cell_to_world(i, j)
                d = math.hypot(wx - c.x, wy - c.y)
                if d <= r_block:
                    self.blocked[j, i] = True
                elif d <= r_soft:
                    self.soft[j, i] = True

    def add_circles(self, circles: Sequence[Circle]) -> None:
        for c in circles:
            self.add_circle(c)

    def blocked_xy(self, x: float, y: float) -> bool:
        i, j = self.world_to_cell(x, y)
        return (not self.in_grid(i, j)) or self.blocked[j, i]

    # ---------- A* ---------- #
    def astar(self, start: Point, goal: Point,
              no_corner_cut: bool = True) -> Optional[List[Point]]:
        """8 连通 A*；返回世界系折线(含 start/goal 精确端点)或 None。"""
        si, sj = self.world_to_cell(*start)
        gi, gj = self.world_to_cell(*goal)
        if not (self.in_grid(si, sj) and self.in_grid(gi, gj)):
            return None
        if self.blocked[sj, si] or self.blocked[gj, gi]:
            return None

        def h(i, j):
            dx = abs(i - gi)
            dy = abs(j - gj)
            return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

        nb = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
              (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
        g = {(si, sj): 0.0}
        parent: Dict[Tuple[int, int], Tuple[int, int]] = {}
        openq = [(h(si, sj), 0.0, (si, sj))]
        closed = set()
        while openq:
            _f, gc, cur = heapq.heappop(openq)
            if cur in closed:
                continue
            if cur == (gi, gj):
                # 回溯
                path = [cur]
                while path[-1] in parent and path[-1] != (si, sj):
                    path.append(parent[path[-1]])
                path.reverse()
                pts = [tuple(start)]
                pts += [self.cell_to_world(i, j) for (i, j) in path[1:-1]]
                pts.append(tuple(goal))
                return pts
            closed.add(cur)
            i, j = cur
            for di, dj, w in nb:
                ni, nj = i + di, j + dj
                if not self.in_grid(ni, nj) or self.blocked[nj, ni]:
                    continue
                if no_corner_cut and di != 0 and dj != 0:
                    if self.blocked[j, i + di] or self.blocked[j + dj, i]:
                        continue
                base = self.cell * w
                if self.soft[nj, ni]:
                    base *= (1.0 + self.soft_cost)
                ng = gc + base
                if ng < g.get((ni, nj), float("inf")):
                    g[(ni, nj)] = ng
                    parent[(ni, nj)] = cur
                    heapq.heappush(openq, (ng + self.cell * h(ni, nj), ng, (ni, nj)))
        return None
