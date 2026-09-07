# -*- coding: utf-8 -*-
"""三次 B 样条（FITPACK, scipy.interpolate.splprep/splev）平滑封装。

splprep 输出的是标准 B 样条表示 (t, c, k)。s=0 时严格插值经过所有给定点；
对折线航路点做 s=0 三次样条插值即可得到 C2 连续、穿过每个航路点的平滑曲线，
再配合“走廊/障碍校验”兜底（样条在航路点之间的小幅过冲会被校验抓住）。
"""
from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np

from . import geometry as G

Point = Tuple[float, float]


def fit_cubic_bspline(pts: Sequence[Point],
                      n_samples: int = 801,
                      s: float = 0.0) -> List[Point]:
    """把点列拟合成三次 B 样条并重采样为 n_samples 个点（含两端）。"""
    from scipy import interpolate as si

    pts = G.remove_dup(pts)
    if len(pts) < 4:
        raise ValueError(f"need >=4 distinct points for cubic spline, got {len(pts)}")
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    tck, _u = si.splprep([xs, ys], s=s, k=3)
    unew = np.linspace(0.0, 1.0, n_samples)
    ox, oy = si.splev(unew, tck)
    return [(float(x), float(y)) for x, y in zip(ox, oy)]


if __name__ == "__main__":
    # 自检：U 形点列，样条经过首尾，长度合理
    pts = [(0, 0), (10, 0), (10, 10), (0, 10)]
    c = fit_cubic_bspline(pts, n_samples=101)
    assert abs(c[0][0] - 0) < 1e-6 and abs(c[0][1] - 0) < 1e-6
    assert abs(c[-1][0] - 0) < 1e-6 and abs(c[-1][1] - 10) < 1e-6
    # 中间应当平滑靠近折线
    print("bspline self-test OK, n=", len(c))
