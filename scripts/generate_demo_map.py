#!/usr/bin/env python3
"""Generate the demo_store static map from the Gazebo cafe world.

The map imitates what a 2D LiDAR (about 0.3 m above the floor) would record
after a SLAM run in the EMPTY cafe: walls, counter, table pedestals and
planters.  Chairs and table tops are intentionally NOT in the map:

* chairs stand for movable obstacles (people, chairs, belongings) that must be
  handled by live sensing and avoidance, not by the static map;
* table tops are above the LiDAR scan plane.

Standard library only.  Output is deterministic (no timestamps, LF newlines),
so the same inputs give byte-identical map.pgm / map.yaml / map_meta.yaml on
every machine.  Replace the generated files with a real SLAM result later;
keep the same file names and nothing else has to change.

Usage:  python3 scripts/generate_demo_map.py [--out-dir DIR] [--world SDF]
"""
import argparse
import math
import struct
import sys
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_WORLD = (
    REPO_ROOT / "ros_ws/src/serving_robot_gazebo/worlds/cafe.world.sdf"
)
DEFAULT_OUT = REPO_ROOT / "maps/stores/demo_store"

# (model-name prefix, only this collision name or None for every collision)
STATIC_RULES = [
    ("cafe_structure", None),
    ("cafe_counter", None),
    ("table_set_", "table_pedestal_collision"),
    ("planter_", None),
]

FREE, OCC, UNKNOWN = 254, 0, 205
EPS = 1e-9


def parse_pose(elem, label):
    """Return ((x, y, yaw), z) from an SDF <pose> element (may be None)."""
    if elem is None or not (elem.text or "").strip():
        return (0.0, 0.0, 0.0), 0.0
    v = [float(t) for t in elem.text.split()]
    v += [0.0] * (6 - len(v))
    x, y, z, roll, pitch, yaw = v[:6]
    if abs(roll) > EPS or abs(pitch) > EPS:
        raise SystemExit(f"{label}: roll/pitch in <pose> is not supported")
    return (x, y, yaw), z


def compose(parent, child):
    px, py, pyaw = parent
    cx, cy, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (px + c * cx - s * cy, py + s * cx + c * cy, pyaw + cyaw)


def collect_shapes(world_path, scan_height):
    world = ET.parse(world_path).getroot().find("world")
    if world is None:
        raise SystemExit(f"{world_path}: no <world> element")
    shapes = []
    for model in world.findall("model"):
        name = model.get("name", "")
        rule = [r for r in STATIC_RULES if name.startswith(r[0])]
        if not rule:
            continue
        only = rule[0][1]
        m_pose, m_z = parse_pose(model.find("pose"), name)
        for link in model.findall("link"):
            l_pose, l_z = parse_pose(link.find("pose"), name)
            base = compose(m_pose, l_pose)
            for col in link.findall("collision"):
                cname = col.get("name", "")
                if only is not None and cname != only:
                    continue
                c_pose, c_z = parse_pose(col.find("pose"), f"{name}/{cname}")
                pose = compose(base, c_pose)
                zc = m_z + l_z + c_z
                geom = col.find("geometry")
                box = geom.find("box")
                cyl = geom.find("cylinder")
                if box is not None:
                    sx, sy, sz = [float(t) for t in box.find("size").text.split()]
                    shape = dict(kind="box", size=(sx, sy), height=sz)
                elif cyl is not None:
                    r = float(cyl.find("radius").text)
                    ln = float(cyl.find("length").text)
                    shape = dict(kind="cylinder", radius=r, height=ln)
                else:
                    raise SystemExit(f"{name}/{cname}: unsupported geometry")
                shape.update(model=name, label=f"{name}/{cname}", pose=pose)
                z0, z1 = zc - shape["height"] / 2, zc + shape["height"] / 2
                if not (z0 - EPS <= scan_height <= z1 + EPS):
                    print(f"warning: {shape['label']} does not reach the scan "
                          f"height {scan_height} m (z {z0:.2f}..{z1:.2f})",
                          file=sys.stderr)
                shapes.append(shape)
    if not shapes:
        raise SystemExit("no static shapes found in the world file")
    return shapes


def aabb(sh):
    cx, cy, yaw = sh["pose"]
    if sh["kind"] == "box":
        sx, sy = sh["size"]
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        hx = c * sx / 2 + s * sy / 2
        hy = s * sx / 2 + c * sy / 2
    else:
        hx = hy = sh["radius"]
    return (cx - hx, cy - hy, cx + hx, cy + hy)


def inside(sh, x, y):
    cx, cy, yaw = sh["pose"]
    dx, dy = x - cx, y - cy
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * dx - s * dy, s * dx + c * dy
    if sh["kind"] == "box":
        sx, sy = sh["size"]
        return abs(lx) <= sx / 2 + EPS and abs(ly) <= sy / 2 + EPS
    return lx * lx + ly * ly <= sh["radius"] ** 2 + EPS


def rasterize(shapes, bounds, res, seed_xy):
    xmin, ymin, xmax, ymax = bounds
    w = int(round((xmax - xmin) / res))
    h = int(round((ymax - ymin) / res))
    if abs((xmax - xmin) / res - w) > 1e-6 or abs((ymax - ymin) / res - h) > 1e-6:
        raise SystemExit("bounds must be a whole multiple of the resolution")

    occ = [[False] * w for _ in range(h)]
    for sh in shapes:
        x0, y0, x1, y1 = aabb(sh)
        if x0 < xmin or y0 < ymin or x1 > xmax or y1 > ymax:
            raise SystemExit(f"{sh['label']} lies outside the map bounds")
        c0 = max(0, int(math.floor((x0 - xmin) / res)))
        c1 = min(w - 1, int(math.ceil((x1 - xmin) / res)))
        r0 = max(0, int(math.floor((ymax - y1) / res)))
        r1 = min(h - 1, int(math.ceil((ymax - y0) / res)))
        for r in range(r0, r1 + 1):
            y = ymax - (r + 0.5) * res
            for c in range(c0, c1 + 1):
                if inside(sh, xmin + (c + 0.5) * res, y):
                    occ[r][c] = True

    # Free space = flood fill from a seed inside the cafe, limited to the
    # outer bounding box of the cafe structure.  Everything beyond the walls
    # (including outside the entrance opening) stays "unknown", like in a
    # real SLAM map.
    walls = [aabb(s) for s in shapes if s["model"] == "cafe_structure"]
    if not walls:
        raise SystemExit("cafe_structure not found; cannot find the interior")
    bx0 = min(a[0] for a in walls)
    by0 = min(a[1] for a in walls)
    bx1 = max(a[2] for a in walls)
    by1 = max(a[3] for a in walls)

    def in_box(c, r):
        x = xmin + (c + 0.5) * res
        y = ymax - (r + 0.5) * res
        return bx0 - EPS <= x <= bx1 + EPS and by0 - EPS <= y <= by1 + EPS

    sc = int((seed_xy[0] - xmin) / res)
    sr = int((ymax - seed_xy[1]) / res)
    if occ[sr][sc]:
        raise SystemExit("flood-fill seed lies on an obstacle")
    grid = bytearray([UNKNOWN]) * (w * h)
    stack = [(sc, sr)]
    seen = {(sc, sr)}
    while stack:
        c, r = stack.pop()
        grid[r * w + c] = FREE
        for nc, nr in ((c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)):
            if (0 <= nc < w and 0 <= nr < h and (nc, nr) not in seen
                    and not occ[nr][nc] and in_box(nc, nr)):
                seen.add((nc, nr))
                stack.append((nc, nr))
    for r in range(h):
        for c in range(w):
            if occ[r][c]:
                grid[r * w + c] = OCC
    return grid, w, h, (bx0, by0, bx1, by1)


def write_text(path, text):
    with open(path, "w", newline="\n", encoding="utf-8") as f:
        f.write(text)


def write_pgm(path, grid, w, h):
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(bytes(grid))


def write_png(path, grid, w, h, scale):
    raw = bytearray()
    for r in range(h):
        line = bytearray()
        for c in range(w):
            line.extend(bytes([grid[r * w + c]]) * scale)
        raw.extend((b"\x00" + bytes(line)) * scale)

    def chunk(tag, payload):
        crc = zlib.crc32(tag + payload) & 0xFFFFFFFF
        return struct.pack(">I", len(payload)) + tag + payload + struct.pack(">I", crc)

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w * scale, h * scale, 8, 0, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(bytes(raw), 9))
           + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)


def map_yaml(res, origin):
    return (
        "image: map.pgm\n"
        "mode: trinary\n"
        f"resolution: {res}\n"
        f"origin: [{origin[0]}, {origin[1]}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        # 0.196, not 0.25: the unknown pixel 205 gives (255-205)/255 = 0.19608,
        # which must NOT count as free (with 0.25 the outside became free space)
        "free_thresh: 0.196\n"
    )


def map_meta_yaml(shapes, w, h, res, bounds, scan_height, world_rel, box):
    xmin, ymin, xmax, ymax = bounds
    lines = [
        "# Coordinate reference table for the demo_store static map.",
        "# Generated by scripts/generate_demo_map.py - do not edit by hand.",
        "schema_version: 1",
        "map_revision: 1",
        "frame_id: map",
        "source: synthetic   # built from the Gazebo world, NOT produced by SLAM",
        "generator: scripts/generate_demo_map.py",
        f"world_file: {world_rel}",
        "image: map.pgm",
        f"resolution_m: {res}",
        f"size_px: {{width: {w}, height: {h}}}",
        f"size_m: {{width: {xmax - xmin:.2f}, height: {ymax - ymin:.2f}}}",
        f"origin: {{x_m: {xmin}, y_m: {ymin}, yaw_rad: 0.0}}",
        "aligned_with: gazebo_world_origin   # map (0, 0) == Gazebo world (0, 0)",
        f"lidar_scan_height_m: {scan_height}   # approximate",
        "pixel_values: {occupied: 0, free: 254, unknown: 205}",
        f"cafe_outer_bounds_m: {{x_min: {box[0]:.2f}, y_min: {box[1]:.2f}, "
        f"x_max: {box[2]:.2f}, y_max: {box[3]:.2f}}}",
        "included:",
        "  - cafe_structure   # outer walls (entrance opening in the south wall)",
        "  - cafe_counter",
        "  - table_pedestals  # table_set_01..04",
        "  - planters",
        "excluded:",
        "  - chairs           # movable: handled by live sensing and avoidance",
        "  - table_tops       # above the LiDAR scan plane",
        "  - people_and_belongings",
        "static_obstacles:",
    ]
    for sh in shapes:
        cx, cy, yaw = sh["pose"]
        lines.append(f"  - name: {sh['label']}")
        lines.append(f"    kind: {sh['kind']}")
        lines.append(f"    center_m: [{cx:.3f}, {cy:.3f}]")
        lines.append(f"    yaw_rad: {yaw:.3f}")
        if sh["kind"] == "box":
            lines.append(f"    size_m: [{sh['size'][0]:.3f}, {sh['size'][1]:.3f}]")
        else:
            lines.append(f"    radius_m: {sh['radius']:.3f}")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--bounds", type=float, nargs=4,
                    metavar=("XMIN", "YMIN", "XMAX", "YMAX"),
                    default=(-4.5, -3.5, 4.5, 3.5))
    ap.add_argument("--scan-height", type=float, default=0.30)
    ap.add_argument("--seed", type=float, nargs=2, default=(0.0, 0.0),
                    metavar=("X", "Y"), help="a point inside the cafe")
    ap.add_argument("--preview-scale", type=int, default=6,
                    help="0 disables map_preview.png")
    a = ap.parse_args()

    shapes = collect_shapes(a.world, a.scan_height)
    grid, w, h, box = rasterize(shapes, tuple(a.bounds), a.resolution, a.seed)

    a.out_dir.mkdir(parents=True, exist_ok=True)
    write_pgm(a.out_dir / "map.pgm", grid, w, h)
    write_text(a.out_dir / "map.yaml",
               map_yaml(a.resolution, (a.bounds[0], a.bounds[1])))
    try:
        world_rel = a.world.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        world_rel = a.world.as_posix()
    write_text(a.out_dir / "map_meta.yaml",
               map_meta_yaml(shapes, w, h, a.resolution, tuple(a.bounds),
                             a.scan_height, world_rel, box))
    if a.preview_scale > 0:
        write_png(a.out_dir / "map_preview.png", grid, w, h, a.preview_scale)

    n_occ = grid.count(OCC)
    n_free = grid.count(FREE)
    n_unk = grid.count(UNKNOWN)
    print(f"map {w}x{h} px @ {a.resolution} m  ->  {a.out_dir}")
    print(f"occupied={n_occ} free={n_free} unknown={n_unk}")
    print(f"shapes in map: {len(shapes)}")
    for sh in shapes:
        print("  -", sh["label"])


if __name__ == "__main__":
    main()
