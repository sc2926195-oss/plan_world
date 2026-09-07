# -*- coding: utf-8 -*-
"""
plan_world 规划器的二维几何工具（不依赖 shapely，只用 numpy）。

约定：
  - 坐标都是世界系 (x, y)，水面在 z=0，ENU。
  - 多边形 poly 用 [(x0,y0), (x1,y1), ...] 顶点列表表示（首尾不重复，视为闭合）。
"""
from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]
Poly = Sequence[Point]

EPS = 1e-9


# --------------------------------------------------------------------------- #
# 基础距离
# --------------------------------------------------------------------------- #
def dist_point_point(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dist_point_seg(p: Point, a: Point, b: Point) -> float:
    """点到线段 ab 的最短距离。"""
    ax, ay = a
    bx, by = b
    px, py = p
    abx, aby = bx - ax, by - ay
    L2 = abx * abx + aby * aby
    if L2 < EPS:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * abx + (py - ay) * aby) / L2
    t = min(1.0, max(0.0, t))
    qx, qy = ax + t * abx, ay + t * aby
    return math.hypot(px - qx, py - qy)


def point_in_poly(p: Point, poly: Poly) -> bool:
    """射线法判断点是否在多边形内部（含边界）。"""
    x, y = p
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and \
                (x < (xj - xi) * (y - yi) / (yj - yi + EPS) + xi):
            inside = not inside
        j = i
    return inside


def dist_point_poly(p: Point, poly: Poly) -> float:
    """点到多边形（实心）的最短距离：在多边形内部时返回 0。"""
    if point_in_poly(p, poly):
        return 0.0
    n = len(poly)
    return min(dist_point_seg(p, poly[i], poly[(i + 1) % n]) for i in range(n))


def seg_poly_clear(a: Point, b: Point, polys: Iterable[Poly],
                   clearance: float, step: float = 0.4) -> bool:
    """线段 [a,b] 是否离所有多边形都 >= clearance（按 step 抽样，够用即可）。"""
    d = dist_point_point(a, b)
    n = max(2, int(math.ceil(d / step)) + 1)
    for i in range(n + 1):
        t = i / n
        p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        for poly in polys:
            if dist_point_poly(p, poly) < clearance - 1e-6:
                return False
    return True


def polyline_poly_clear(pts: Sequence[Point], polys: Iterable[Poly],
                        clearance: float, step: float = 0.4) -> bool:
    for a, b in zip(pts[:-1], pts[1:]):
        if not seg_poly_clear(a, b, polys, clearance, step):
            return False
    return True


def points_poly_clear(pts: Sequence[Point], polys: Iterable[Poly],
                      clearance: float) -> bool:
    for p in pts:
        for poly in polys:
            if dist_point_poly(p, poly) < clearance - 1e-6:
                return False
    return True


# --------------------------------------------------------------------------- #
# 折线 / 曲线工具
# --------------------------------------------------------------------------- #
def remove_dup(pts: Sequence[Point], tol: float = 1e-6) -> List[Point]:
    out: List[Point] = []
    for p in pts:
        if not out or dist_point_point(out[-1], p) > tol:
            out.append(p)
    return out


def resample_polyline(pts: Sequence[Point], step: float) -> List[Point]:
    """按固定弧长步长重采样折线：保留所有原始顶点，只对长于 step 的段加密。"""
    out: List[Point] = [tuple(pts[0])]
    for a, b in zip(pts[:-1], pts[1:]):
        d = dist_point_point(a, b)
        if d < EPS:
            continue
        n = max(1, int(math.ceil(d / step)))
        for i in range(1, n):          # 中间插值点，不重复首尾
            t = i / n
            out.append((a[0] + (b[0] - a[0]) * t,
                        a[1] + (b[1] - a[1]) * t))
        out.append(tuple(b))           # 保留原始顶点
    return out


def offset_polyline(pts: Sequence[Point], side: float,
                    left: bool) -> List[Point]:
    """把折线沿法向偏移 side（left=True 往左偏），得到走廊一侧边界。

    用相邻段法向的平均做简单平滑；供画图/校验用，不做精确 Minkowski offset。
    """
    out: List[Point] = []
    n = len(pts)
    for i in range(n):
        # 前一段方向
        if i == 0:
            dx = pts[1][0] - pts[0][0]
            dy = pts[1][1] - pts[0][1]
        else:
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
        L = math.hypot(dx, dy)
        if L < EPS:
            out.append(pts[i])
            continue
        ux, uy = dx / L, dy / L
        # 左法向: (-uy, ux); 右法向: (uy, -ux)
        nx, ny = (-uy, ux) if left else (uy, -ux)
        out.append((pts[i][0] + side * nx, pts[i][1] + side * ny))
    return out


def sample_heading(pts: Sequence[Point]) -> List[float]:
    """每点的航向角 yaw (atan2(dy,dx))，用前后差分。返回与 pts 等长。"""
    n = len(pts)
    out: List[float] = [0.0] * n
    for i in range(n):
        if i == 0:
            dx = pts[1][0] - pts[0][0]
            dy = pts[1][1] - pts[0][1]
        elif i == n - 1:
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
        else:
            dx = pts[i + 1][0] - pts[i - 1][0]
            dy = pts[i + 1][1] - pts[i - 1][1]
        out[i] = math.atan2(dy, dx)
    return out


def curvature_from_pts(pts: Sequence[Point]) -> List[float]:
    """用三点外接圆近似曲率 1/R（带符号），供报告。"""
    out: List[float] = [0.0] * len(pts)
    for i in range(1, len(pts) - 1):
        p0 = np.asarray(pts[i - 1], float)
        p1 = np.asarray(pts[i], float)
        p2 = np.asarray(pts[i + 1], float)
        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p1 - p0)
        c = np.linalg.norm(p2 - p0)
        if a < EPS or b < EPS or c < EPS:
            continue
        # 海伦公式求面积 -> 外接圆半径 R = abc/(4S)
        s = (a + b + c) / 2.0
        area2 = max(0.0, s * (s - a) * (s - b) * (s - c))
        S = math.sqrt(area2)
        if S < EPS:
            continue
        R = a * b * c / (4.0 * S)
        # 转向符号：用叉积 z 分量
        cross = (p1 - p0)[0] * (p2 - p1)[1] - (p1 - p0)[1] * (p2 - p1)[0]
        out[i] = math.copysign(1.0 / R, cross)
    return out


def rect_poly(cx: float, cy: float, hw: float, hh: float) -> Poly:
    """中心 (cx,cy)，半宽 hw、半高 hh 的轴对齐矩形。"""
    return [(cx - hw, cy - hh), (cx + hw, cy - hh),
            (cx + hw, cy + hh), (cx - hw, cy + hh)]


def poly_area(poly: Poly) -> float:
    a = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2.0


def poly_centroid(poly: Poly) -> Point:
    a = 0.0
    cx = cy = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        cr = x1 * y2 - x2 * y1
        a += cr
        cx += (x1 + x2) * cr
        cy += (y1 + y2) * cr
    a *= 0.5
    if abs(a) < EPS:
        return (poly[0][0], poly[0][1])
    return (cx / (6 * a), cy / (6 * a))


if __name__ == "__main__":
    # 极小的自检
    tri = [(0, 0), (4, 0), (2, 3)]
    assert abs(dist_point_poly((2, 1), tri) - 0.0) < 1e-9          # inside
    assert abs(dist_point_poly((6, 0), tri) - 2.0) < 1e-9          # on edge line
    assert abs(dist_point_poly((2, 5), tri) - 2.0) < 1e-9
    assert point_in_poly((2, 1), tri)
    assert not point_in_poly((5, 1), tri)
    off = offset_polyline([(0, 0), (10, 0)], 1.5, left=True)
    assert abs(off[0][1] - 1.5) < 1e-9
    print("geometry self-test OK")


class ObstacleField:
    """障碍集合：带外接圆快速剔除的距离查询（避免逐点多边形全查）。

    用于 RRT*/校验里“点/线段离所有障碍 >= clearance”的高频判断。
    """

    def __init__(self, polys: Iterable[Poly]):
        self.items = []
        for poly in polys:
            poly = [tuple(p) for p in poly]
            c = poly_centroid(poly)
            r = max(dist_point_point(c, v) for v in poly) + 1e-9
            self.items.append((poly, c, r))

    def dist(self, p: Point) -> float:
        """到最近障碍(实心多边形)的距离；在多边形内部返回 0。"""
        best = math.inf
        for poly, c, r in self.items:
            if dist_point_point(p, c) - r >= best:
                continue          # 外接圆都够不到当前 best，直接跳过
            d = dist_point_poly(p, poly)
            if d < best:
                best = d
        return best

    def clear(self, p: Point, clearance: float) -> bool:
        return self.dist(p) >= clearance - 1e-6

    def seg_clear(self, a: Point, b: Point, clearance: float,
                  step: float = 0.5) -> bool:
        d = dist_point_point(a, b)
        n = max(2, int(math.ceil(d / step)) + 1)
        for i in range(n + 1):
            t = i / n
            p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            if self.dist(p) < clearance - 1e-6:
                return False
        return True


def straighten_tail_to_axis(pts: Sequence[Point], axis_x: float,
                            y_start: float, ease: float = 2.0) -> List[Point]:
    """把 y>=y_start 的部分精确拉直到轴线 x=axis_x（用于入港笔直段）。

    为避免“硬切”造成小拐角，在 y∈[y_start-ease, y_start] 内把横向偏移
    线性过渡到 0（渐进拉直），y>=y_start 后严格贴轴线(x=axis_x)，
    从而保证末段笔直、航向 = +Y(90°)。
    """
    out: List[Point] = []
    lo = y_start - ease
    for p in pts:
        x, y = p
        if y < lo - 1e-9:
            out.append((x, y))
        elif y < y_start - 1e-9:
            t = (y - lo) / ease          # 0->1
            xb = axis_x + (x - axis_x) * (1.0 - t)
            out.append((xb, y))
        else:
            out.append((axis_x, y))
    return out


def decimate_indices(pts: Sequence[Point], min_step: float) -> List[int]:
    """贪心抽稀：返回保留点的索引(相邻保留点弧距>=min_step，恒含首尾)。

    用于显示等“减点”场景；与 resample_polyline(加密)互补。
    """
    if not pts:
        return []
    keep = [0]
    last = pts[0]
    for i in range(1, len(pts)):
        if dist_point_point(pts[i], last) >= min_step - 1e-9:
            keep.append(i)
            last = pts[i]
    if keep[-1] != len(pts) - 1:
        keep.append(len(pts) - 1)
    return keep
