# -*- coding: utf-8 -*-
"""转弯半径标尺图：直观对比 R=3/5/8 m 的 90° 弯有多大。

也给出对应“走廊内缘半径”(走廊半宽1.5m)与“入港48°实际转角”的弧长，
方便对 R_min 建立直觉。只画图，不影响规划。
"""
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

BOAT_L, BOAT_W = 4.9, 2.5
CORR_HW = 1.5
radii = [3.0, 5.0, 8.0]
colors = {3.0: "#d62728", 5.0: "#1f77b4", 8.0: "#2ca02c"}

fig, ax = plt.subplots(figsize=(10, 8))
ax.set_aspect("equal")
ax.set_xlim(-12, 13)
ax.set_ylim(-12, 13)

# 直角弯的两条“路”：从西向东，再转向北(90°左转)，拐角在原点
ax.annotate("", xy=(13, 0), xytext=(-12, 0),
            arrowprops=dict(arrowstyle="-|>", color="0.3", lw=1.5))
ax.annotate("", xy=(0, 13), xytext=(0, -12),
            arrowprops=dict(arrowstyle="-|>", color="0.3", lw=1.5))
ax.text(9, -1.4, "approach", ha="center")
ax.text(1.2, 10.5, "north / dock axis", ha="left")
ax.plot(0, 0, "ko", ms=5)

# 不同半径的 90° 圆角(弧)，并画出切点位置
for r in radii:
    C = (-r, r)
    alpha = [math.radians(a) for a in range(-90, 1)]
    xs = [C[0] + r * math.cos(a) for a in alpha]
    ys = [C[1] + r * math.sin(a) for a in alpha]
    ax.plot(xs, ys, color=colors[r], lw=2.5,
            label=f"R={r:g} m")
    # 切点连线(虚线)与圆心
    ax.plot([-r, 0], [0, r], color=colors[r], ls=":", lw=1)
    ax.plot(C[0], C[1], marker="+", color=colors[r], ms=8)
    # 在弧中点放船(4.9x2.5)，航向=切向
    am = math.radians(-45)
    mx, my = C[0] + r * math.cos(am), C[1] + r * math.sin(am)
    hd = am + math.pi / 2            # 船首朝向(切向)
    ux, uy = math.cos(hd), math.sin(hd)
    px_, py_ = -uy, ux               # 船宽方向
    hx, hy = ux * BOAT_L / 2, uy * BOAT_L / 2
    wx_, wy_ = px_ * BOAT_W / 2, py_ * BOAT_W / 2
    corners = [(mx + hx + wx_, my + hy + wy_),
               (mx + hx - wx_, my + hy - wy_),
               (mx - hx - wx_, my - hy - wy_),
               (mx - hx + wx_, my - hy + wy_)]
    ax.add_patch(Polygon(corners, closed=True, facecolor=colors[r],
                         edgecolor="k", alpha=0.35, lw=0.8))
    ax.text(mx, my - 0.7, f"R={r:g}", ha="center", fontsize=9,
            color=colors[r], fontweight="bold")

# 船身比例尺(放在左下，朝东)
ref = Rectangle((-10.5, -9.5), BOAT_L, BOAT_W, angle=0, facecolor="0.75",
                edgecolor="k", lw=1)
ax.add_patch(ref)
ax.text(-10.5 + BOAT_L / 2, -8.2, "boat 4.9 x 2.5 m", ha="center", fontsize=9)
ax.text(-10.5, -11.0, "scale bar", fontsize=8, color="0.3")

# 数值说明
txt = (
    "90-degree turn arc length:\n"
    "  R=3 m  -> ~4.7 m   (shorter than the boat!)  \n"
    "  R=5 m  -> ~7.9 m   (> boat length, calmer)   \n"
    "  R=8 m  -> ~12.6 m\n\n"
    "Corridor inner-edge radius (half-width 1.5 m):\n"
    "  R=3 -> 1.5 m | R=5 -> 3.5 m | R=8 -> 6.5 m\n\n"
    "Our docking turn is ~48 deg, so arc length:\n"
    "  R=3 -> 2.5 m | R=5 -> 4.2 m | R=8 -> 6.7 m"
)
ax.text(0.02, 0.02, txt, transform=ax.transAxes, va="bottom", ha="left",
        fontsize=10, family="monospace",
        bbox=dict(boxstyle="round", fc="white", alpha=0.85))

ax.set_title("Turning radius feel-check  (90-deg corner; boat drawn to scale)",
             fontsize=12)
ax.grid(True, alpha=0.2)
ax.legend(loc="upper right", fontsize=10)
fig.tight_layout()
out = __import__("pathlib").Path(__file__).resolve().parent / "turn_radius_compare.png"
fig.savefig(out, dpi=130)
print("saved:", out)
