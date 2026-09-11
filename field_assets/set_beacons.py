#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
设置主世界三个港口信标的颜色: 指定/随机一个港为绿(正确), 其余红。
信标安装在后墙顶面上(y=101.25, z=0.5), 不再悬空。
实战: 只有港口上方的信标灯(低位 Color Indicator 已移除)。
信标位置在泊位后墙之后(y=102.0)，随新的 1.5x2.0m 泊位一起内移。
只改写 plan_world.sdf 的 BEACONS 段(obstacles 段不动)。

用法:
  python3 field_assets/set_beacons.py --green 1        # port_1 绿(默认演示)
  python3 field_assets/set_beacons.py --random          # 每次随机一个绿港
  python3 field_assets/set_beacons.py --sdf /tmp/x.sdf --green 2   # 改别的文件
"""
import argparse, random, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_SDF = HERE.parent / "plan_world.sdf"
PORTS = {0: (0.0, "port_0"), 1: (50.0, "port_1"), 2: (100.0, "port_2")}
BEGIN = "    <!-- ===== BEACONS (BEGIN) ===== -->"
END = "    <!-- ===== BEACONS (END) ===== -->"



def beacon_model(name: str, px: float, color: str) -> str:
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{px} 101.25 0.5 0 0 0</pose>
      <link name="beacon">
        <visual name="base">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>0.30 0.50 0.06</size>
            </box>
          </geometry>
          <material>
            <ambient>0.25 0.22 0.20 1</ambient>
            <diffuse>0.35 0.32 0.30 1</diffuse>
          </material>
        </visual>
        <visual name="pole">
          <pose>0 0 0.8 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>1.6</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.35 0.30 0.25 1</ambient>
            <diffuse>0.45 0.40 0.35 1</diffuse>
          </material>
        </visual>
        <visual name="lamp">
          <pose>0 0 1.65 0 0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.08 0.30</size>
            </box>
          </geometry>
          <material>
            <ambient>0.03 0.03 0.03 1</ambient>
            <diffuse>0.03 0.03 0.03 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
          <plugin name="vrx::LightBuoyPlugin" filename="libLightBuoyPlugin.so">
            <color_1>{color}</color_1>
            <color_2>{color}</color_2>
            <color_3>{color}</color_3>
            <visuals>
              <visual>{name}::beacon::lamp</visual>
            </visuals>
          </plugin>
        </visual>
      </link>
    </model>"""


def green_for_seed(seed: int) -> int:
    """由世界种子确定唯一绿港(同 seed 可复现)。"""
    return random.Random(int(seed) * 7919 + 13).randrange(3)


def apply_green(sdf_path: Path, green: int) -> None:
    """把指定绿港写进 sdf 的 BEACONS 段(供 generate_obstacles 调用)。"""
    sdf = Path(sdf_path)
    s = sdf.read_text(encoding="utf-8")
    i0 = s.index(BEGIN); i1 = s.index(END) + len(END)
    s = s[:i0] + build_block(green) + s[i1:]
    sdf.write_text(s, encoding="utf-8")


def build_block(green: int) -> str:
    lines = ["    <!-- ===== BEACONS (BEGIN) ===== -->"]
    for idx, px in ((0, 0.0), (1, 50.0), (2, 100.0)):
        color = "green" if idx == green else "red"
        lines.append(beacon_model(f"beacon_port_{idx}", px, color))
    lines.append("    <!-- ===== BEACONS (END) ===== -->")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--green", type=int, choices=(0, 1, 2), help="绿港编号")
    g.add_argument("--random", action="store_true", help="随机选绿港(系统随机)")
    g.add_argument("--seed", type=int, help="由世界种子确定绿港(可复现)")
    ap.add_argument("--sdf", type=Path, default=DEFAULT_SDF)
    args = ap.parse_args()
    if args.random:
        green = random.choice((0, 1, 2))
    elif args.seed is not None:
        green = green_for_seed(args.seed)
    else:
        green = args.green

    apply_green(Path(args.sdf), green)
    print(f"已设置: {PORTS[green][1]} = 绿(正确), 其余 = 红  -> {args.sdf}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
