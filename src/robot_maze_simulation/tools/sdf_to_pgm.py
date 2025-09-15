#!/usr/bin/env python3
import argparse
import math
import os
import xml.etree.ElementTree as ET
from PIL import Image

# Simple SDF->PGM rasterizer for axis-aligned box walls in a flat world.
# Assumptions:
# - Maze walls are <box> visuals/collisions inside a static <model> at z ~ 0.
# - Walls are axis-aligned (yaw approx 0 or pi/2). Small floating rounding tolerated.
# - World bounds inferred from a square ground plane size or from wall extents.


def parse_sdf_boxes(sdf_path):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    ns = ''  # SDF 1.7 doesn't require a namespace when parsed like this

    boxes = []  # list of (cx, cy, yaw, sx, sy)

    def parse_pose(text):
        x, y, z, roll, pitch, yaw = [float(v) for v in text.strip().split()]  # noqa
        return x, y, z, roll, pitch, yaw

    # infer ground size if present
    ground_size = None
    for plane in root.findall('.//plane'):
        size = plane.find('size')
        if size is not None:
            try:
                sx, sy = [float(v) for v in size.text.strip().split()]
                ground_size = (sx, sy)
            except Exception:
                pass
            break

    # iterate all links with a <box> geometry (either under collision or visual)
    for link in root.findall('.//link'):
        pose_el = link.find('pose')
        if pose_el is None:
            continue
        cx, cy, cz, rr, pp, yaw = parse_pose(pose_el.text)
        # find a box size; prefer collision, else visual
        size_el = link.find('.//collision//box//size')
        if size_el is None:
            size_el = link.find('.//visual//box//size')
        if size_el is None:
            continue
        try:
            sx, sy, sz = [float(v) for v in size_el.text.strip().split()]
        except Exception:
            continue
        # keep X/Y footprint only
        boxes.append((cx, cy, yaw, sx, sy))

    return boxes, ground_size


def rasterize(boxes, ground_size, resolution, padding=0.2):
    # Determine bounds from ground_size or boxes
    if ground_size:
        half_x = ground_size[0] / 2.0
        half_y = ground_size[1] / 2.0
        min_x, max_x = -half_x, half_x
        min_y, max_y = -half_y, half_y
    else:
        min_x = min((cx - max(sx, sy) for cx, cy, yaw, sx, sy in boxes), default=-4.0) - padding
        max_x = max((cx + max(sx, sy) for cx, cy, yaw, sx, sy in boxes), default=4.0) + padding
        min_y = min((cy - max(sx, sy) for cx, cy, yaw, sx, sy in boxes), default=-4.0) - padding
        max_y = max((cy + max(sx, sy) for cx, cy, yaw, sx, sy in boxes), default=4.0) + padding

    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))

    # start with free space (255), mark walls as occupied (0)
    img = Image.new('L', (width, height), 255)
    px = img.load()

    def world_to_pixel(x, y):
        col = int((x - min_x) / resolution)
        row = int((max_y - y) / resolution)  # y downwards
        return col, row

    def mark_rect(cx, cy, yaw, sx, sy):
        # rasterize an axis-aligned rectangle; if yaw ~ 0 or 90 deg, we can approximate by rotating size
        # snap small yaws to 0 or pi/2
        yaw_norm = ((yaw + math.pi) % (2 * math.pi)) - math.pi
        if abs(yaw_norm) < math.radians(5):
            ax, ay = sx, sy
        elif abs(abs(yaw_norm) - math.pi / 2) < math.radians(5):
            ax, ay = sy, sx
        else:
            # fall back to conservative bounding box
            ax = max(sx, sy)
            ay = max(sx, sy)
        # corners in world
        x0, y0 = cx - ax / 2.0, cy - ay / 2.0
        x1, y1 = cx + ax / 2.0, cy + ay / 2.0
        c0, r0 = world_to_pixel(x0, y0)
        c1, r1 = world_to_pixel(x1, y1)
        c0, c1 = max(0, min(c0, c1)), min(width - 1, max(c0, c1))
        r0, r1 = max(0, min(r0, r1)), min(height - 1, max(r0, r1))
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                px[c, r] = 0

    for (cx, cy, yaw, sx, sy) in boxes:
        mark_rect(cx, cy, yaw, sx, sy)

    origin = (min_x, min_y, 0.0)
    return img, origin


def main():
    ap = argparse.ArgumentParser(description='Convert SDF maze.world into PGM/YAML occupancy map')
    ap.add_argument('--sdf', required=True, help='Path to worlds/maze.world (SDF)')
    ap.add_argument('--pgm', required=True, help='Output PGM path')
    ap.add_argument('--yaml', required=True, help='Output YAML path')
    ap.add_argument('--resolution', type=float, default=0.05, help='Map resolution (m/pixel)')
    ap.add_argument('--occupied', type=int, default=100, help='Occupancy value for obstacles (0-100)')
    ap.add_argument('--free', type=int, default=0, help='Occupancy value for free space (0-100)')
    args = ap.parse_args()

    boxes, ground = parse_sdf_boxes(args.sdf)
    if not boxes:
        print('Warning: No boxes found in SDF. Map may be empty.')
    img, origin = rasterize(boxes, ground, args.resolution)

    # Save PGM (ROS expects 0=occupied black, 255=free white); we already encoded that
    img.save(args.pgm, format='PPM')  # PIL uses PPM/PGM based on mode; ensure L mode wrote PGM

    yaml = f"""
image: {os.path.basename(args.pgm)}
resolution: {args.resolution}
origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(args.yaml, 'w') as f:
        f.write(yaml)

    print(f"Wrote map: {args.pgm} and {args.yaml}")

if __name__ == '__main__':
    main()
