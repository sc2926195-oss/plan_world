#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate the random obstacles for the planning world (plan_world.sdf).

Obstacles:  10 static floating objects, each either an equilateral triangle
            or a square, footprint ~10 m^2, scattered inside the 100x100 m
            water region (x/y in [0,100]).

It rewrites the block between the two markers
    <!-- ===== AUTO-GENERATED OBSTACLES (BEGIN) ===== -->
    <!-- ===== AUTO-GENERATED OBSTACLES (END) ===== -->
inside plan_world.sdf, and writes ground-truth metadata:
    obstacles_meta.json  (full info incl. footprint polygon in world frame)
    obstacles_meta.csv   (compact summary)

By default a NEW random layout is generated on every run
(seed chosen from the system clock). Pass --seed to reproduce a layout.
"""
import argparse
import csv
import json
import math
import random
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORLD_FILE = HERE / "plan_world.sdf"
META_JSON = HERE / "obstacles_meta.json"
META_CSV = HERE / "obstacles_meta.csv"

# ---------------------------------------------------------------------------
# Scenario parameters
# ---------------------------------------------------------------------------
N_OBSTACLES = 10
AREA = 10.0                          # footprint area [m^2]
OBSTACLE_HEIGHT = 0.6                # height above water [m]
# random placement region (keep clear of the 100x100 edges)
REGION = (6.0, 94.0, 6.0, 94.0)      # (xmin, xmax, ymin, ymax)
SPAWN = (50.0, 0.0)                  # WAM-V spawn point
SPAWN_CLEAR = 10.0                   # keep obstacles away from spawn
# keep the approach to each port clear: berth centres are at these x
PORT_XS = (0.0, 50.0, 100.0)
PORT_CLEAR_X = 9.0                   # half-width of the clear lane (x)
PORT_CLEAR_Y = 85.0                  # obstacles must stay south of this y
MIN_SEPARATION = 8.0                 # min distance between obstacle centres
DEFAULT_SEED = None   # None => random seed chosen at every run

# colors (r g b a)
COLOR_SQUARE = (0.80, 0.20, 0.15, 1.0)
COLOR_TRIANGLE = (0.90, 0.55, 0.10, 1.0)


def square_geom(side):
    """Local-frame footprint polygon of a square (centred at origin)."""
    h = side / 2.0
    return [(-h, -h), (h, -h), (h, h), (-h, h)]


def triangle_geom(side):
    """Local-frame footprint polygon of an equilateral triangle whose
    centroid is at the origin and whose apex points along +Y."""
    r = side / math.sqrt(3.0)          # circumradius
    pts = []
    for deg in (90.0, 210.0, 330.0):
        a = math.radians(deg)
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def rot_yaw(poly, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x * c - y * s, x * s + y * c) for x, y in poly]


def add(p, q):
    return (p[0] + q[0], p[1] + q[1])


def build_obstacle_model(idx, kind, cx, cy, yaw):
    """Return the SDF <model> text for one static floating obstacle."""
    if kind == "square":
        side = math.sqrt(AREA)
        geom = (
            "        <geometry>\n"
            "          <box>\n"
            f"            <size>{side:.4f} {side:.4f} {OBSTACLE_HEIGHT:.4f}</size>\n"
            "          </box>\n"
            "        </geometry>\n"
        )
        # box centre sits OBSTACLE_HEIGHT/2 above the model frame (water)
        local_z = OBSTACLE_HEIGHT / 2.0
    else:  # triangle
        side = math.sqrt(4.0 * AREA / math.sqrt(3.0))
        pts = triangle_geom(side)
        poly = "\n".join(
            f"              <point>{x:.4f} {y:.4f}</point>" for x, y in pts
        )
        geom = (
            "        <geometry>\n"
            "          <polyline>\n"
            f"{poly}\n"
            f"              <height>{OBSTACLE_HEIGHT:.4f}</height>\n"
            "          </polyline>\n"
            "        </geometry>\n"
        )
        local_z = 0.0  # polyline is extruded from z=0 up to height

    color = COLOR_SQUARE if kind == "square" else COLOR_TRIANGLE
    return f"""    <model name="obstacle_{idx}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {local_z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="obstacle">
        <collision name="collision">
{geom}        </collision>
        <visual name="visual">
{geom}          <material>
            <ambient>{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} {color[3]:.1f}</ambient>
            <diffuse>{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} {color[3]:.1f}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def sample_clear(rng):
    xmin, xmax, ymin, ymax = REGION
    while True:
        x = rng.uniform(xmin, xmax)
        y = rng.uniform(ymin, ymax)
        # far enough from the boat spawn
        if math.hypot(x - SPAWN[0], y - SPAWN[1]) < SPAWN_CLEAR:
            continue
        # clear approach lanes to the three ports
        if y > PORT_CLEAR_Y and any(
            abs(x - px) < PORT_CLEAR_X for px in PORT_XS
        ):
            continue
        return x, y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=DEFAULT_SEED,
                        help="fixed seed for a reproducible layout "
                             "(default: random every run)")
    ap.add_argument("--count", type=int, default=N_OBSTACLES)
    ap.add_argument("--area", type=float, default=AREA)
    args = ap.parse_args()

    if args.seed is None:
        args.seed = random.SystemRandom().randint(0, 2**31 - 1)
    rng = random.Random(args.seed)
    obstacles = []
    models = []
    attempts = 0
    while len(obstacles) < args.count and attempts < 10000:
        attempts += 1
        kind = rng.choice(("square", "triangle"))
        cx, cy = sample_clear(rng)
        # keep a minimum separation between obstacle centres
        if any(math.hypot(cx - o["x"], cy - o["y"]) < MIN_SEPARATION
               for o in obstacles):
            continue
        yaw = rng.uniform(0.0, 2.0 * math.pi)
        side = (math.sqrt(args.area) if kind == "square"
                else math.sqrt(4.0 * args.area / math.sqrt(3.0)))
        local = square_geom(side) if kind == "square" else triangle_geom(side)
        world_poly = [add(rot_yaw(local, yaw)[i], (cx, cy)) for i in range(len(local))]
        # keep every footprint vertex inside the 100x100 water region
        if not all(0.0 <= px <= 100.0 and 0.0 <= py <= 100.0
                   for px, py in world_poly):
            continue
        idx = len(obstacles)
        obstacles.append({
            "id": idx,
            "shape": kind,
            "area_m2": round(args.area, 3),
            "side_m": round(side, 3),
            "x": round(cx, 3),
            "y": round(cy, 3),
            "yaw_rad": round(yaw, 4),
            "polygon_xy": [[round(px, 3), round(py, 3)] for px, py in world_poly],
        })
        models.append(build_obstacle_model(idx, kind, cx, cy, yaw))

    if len(obstacles) < args.count:
        raise RuntimeError(
            f"only placed {len(obstacles)}/{args.count} obstacles "
            "(placement constraints too tight?)")

    # ---- rewrite the marker region in the world file ---------------------
    text = WORLD_FILE.read_text(encoding="utf-8")
    begin = "    <!-- ===== AUTO-GENERATED OBSTACLES (BEGIN) ===== -->"
    end = "    <!-- ===== AUTO-GENERATED OBSTACLES (END) ===== -->"
    i0 = text.find(begin)
    i1 = text.find(end)
    assert i0 != -1 and i1 != -1, "obstacle markers not found in world file"
    i1 += len(end)
    new_region = begin + "\n" + "\n".join(models) + "\n" + end
    text = text[:i0] + new_region + text[i1:]
    WORLD_FILE.write_text(text, encoding="utf-8")

    # ---- metadata ---------------------------------------------------------
    META_JSON.write_text(
        json.dumps({
            "world": "plan_world",
            "water_region_xy": [[0, 0], [100, 100]],
            "seed": args.seed,
            "obstacle_height_m": OBSTACLE_HEIGHT,
            "obstacles": obstacles,
        }, indent=2, ensure_ascii=False),
        encoding="utf-8")

    with META_CSV.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["id", "shape", "area_m2", "side_m", "x", "y", "yaw_rad"])
        for o in obstacles:
            w.writerow([o["id"], o["shape"], o["area_m2"], o["side_m"],
                        o["x"], o["y"], o["yaw_rad"]])

    print(f"seed={args.seed}: placed {len(obstacles)} obstacles")
    for o in obstacles:
        print(f"  obstacle_{o['id']:02d} {o['shape']:8s} "
              f"centre=({o['x']:7.2f},{o['y']:7.2f}) yaw={o['yaw_rad']:.2f}")


if __name__ == "__main__":
    main()
