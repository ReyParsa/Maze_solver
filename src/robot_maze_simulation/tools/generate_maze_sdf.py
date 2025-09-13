#!/usr/bin/env python3
"""Generate a maze SDF file (static).

Produces /tmp/maze_world_generated.sdf by default.
Supports --compact to emit a single model containing all wall boxes (faster to load).
Reserves a clear center spawn area so the robot won't be embedded in walls.
"""
import argparse
import math
import random
from typing import List, Tuple


def generate_maze(rows: int, cols: int, seed: int = None, cell_size: float = 0.4) -> List[List[dict]]:
    if seed is not None:
        random.seed(seed)

    # initialize all walls present
    walls = [[{'top': True, 'right': True, 'bottom': True, 'left': True} for _ in range(cols)] for _ in range(rows)]

    visited = [[False] * cols for _ in range(rows)]
    stack = [(0, 0)]
    visited[0][0] = True

    while stack:
        r, c = stack[-1]
        nbs = []
        if r > 0 and not visited[r - 1][c]:
            nbs.append((r - 1, c))
        if c < cols - 1 and not visited[r][c + 1]:
            nbs.append((r, c + 1))
        if r < rows - 1 and not visited[r + 1][c]:
            nbs.append((r + 1, c))
        if c > 0 and not visited[r][c - 1]:
            nbs.append((r, c - 1))

        if nbs:
            nr, nc = random.choice(nbs)
            # remove wall between
            if nr == r - 1:
                walls[r][c]['top'] = False
                walls[nr][nc]['bottom'] = False
            elif nr == r + 1:
                walls[r][c]['bottom'] = False
                walls[nr][nc]['top'] = False
            elif nc == c - 1:
                walls[r][c]['left'] = False
                walls[nr][nc]['right'] = False
            elif nc == c + 1:
                walls[r][c]['right'] = False
                walls[nr][nc]['left'] = False
            visited[nr][nc] = True
            stack.append((nr, nc))
        else:
            stack.pop()

    # reserve a center clear area so the robot spawns free.
    # Compute how many cells are needed based on an approximate robot footprint (0.8m default).
    # Ensure at least a 3x3 patch for small mazes.
    min_clear_m = 0.8
    clear_cells = max(3, int(math.ceil(min_clear_m / cell_size)))
    if rows >= clear_cells and cols >= clear_cells:
        cr = rows // 2
        cc = cols // 2
        half = clear_cells // 2
        for r in range(cr - half, cr - half + clear_cells):
            for c in range(cc - half, cc - half + clear_cells):
                if 0 <= r < rows and 0 <= c < cols:
                    walls[r][c] = {'top': False, 'right': False, 'bottom': False, 'left': False}

    return walls


def walls_to_segments(walls, rows, cols, cell_size) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    segments = []
    maze_width = cols * cell_size
    maze_height = rows * cell_size
    x0 = -maze_width / 2.0
    y0 = -maze_height / 2.0

    for r in range(rows):
        for c in range(cols):
            cx = x0 + c * cell_size + cell_size / 2.0
            cy = y0 + r * cell_size + cell_size / 2.0
            w = walls[r][c]
            half = cell_size / 2.0
            if w.get('top', False):
                segments.append(((cx - half, cy - half), (cx + half, cy - half)))
            if w.get('bottom', False):
                segments.append(((cx - half, cy + half), (cx + half, cy + half)))
            if w.get('left', False):
                segments.append(((cx - half, cy - half), (cx - half, cy + half)))
            if w.get('right', False):
                segments.append(((cx + half, cy - half), (cx + half, cy + half)))

    # dedupe
    uniq = []
    seen = set()
    for a, b in segments:
        key = (round(a[0], 4), round(a[1], 4), round(b[0], 4), round(b[1], 4))
        key_rev = (round(b[0], 4), round(b[1], 4), round(a[0], 4), round(a[1], 4))
        if key in seen or key_rev in seen:
            continue
        seen.add(key)
        uniq.append((a, b))
    return uniq


# module-level wall sizing (used by SDF and single-mesh export)
WALL_THICKNESS = 0.06
WALL_HEIGHT = 0.4


def create_world_sdf(segments, ground_size_x, ground_size_y, compact=False):
        header = """<?xml version='1.0'?>\n<sdf version='1.7'>\n  <world name='maze_world'>\n"""
        footer = "\n  </world>\n</sdf>\n"
        parts = [header]

        parts.append("""
        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <direction>-0.5 0.1 -0.9</direction>
        </light>
        """)

        parts.append(f"  <model name='ground'>\n    <static>true</static>\n    <link name='link'>\n      <collision name='collision'>\n        <geometry>\n          <plane>\n            <normal>0 0 1</normal>\n            <size>{ground_size_x} {ground_size_y}</size>\n          </plane>\n        </geometry>\n      </collision>\n      <visual name='visual'>\n        <geometry>\n          <plane>\n            <normal>0 0 1</normal>\n            <size>{ground_size_x} {ground_size_y}</size>\n          </plane>\n        </geometry>\n      </visual>\n    </link>\n  </model>\n")

        # wall sizing constants (ensure available for camera placement)
        wall_thickness = WALL_THICKNESS
        wall_height = WALL_HEIGHT

        # Add a default GUI camera position so the GUI centers on the maze when opened
        cam_z = max(1.0, wall_height * 4.0)
        parts.append(f"  <gui>\n    <camera name='camera'>\n      <pose>0 0 {cam_z} 0 -0.6 0</pose>\n    </camera>\n  </gui>\n")

        # Always emit a single top-level model 'maze' containing links for each wall
        parts.append("  <model name='maze'>\n    <static>true</static>\n")
        for i, seg in enumerate(segments):
                (sx, sy), (ex, ey) = seg
                length = math.hypot(ex - sx, ey - sy)
                cx = (sx + ex) / 2.0
                cy = (sy + ey) / 2.0
                ang = math.atan2(ey - sy, ex - sx)
                parts.append(f"    <link name='maze_wall_{i}_link'>\n      <pose>{cx} {cy} {wall_height/2.0} 0 0 {ang}</pose>\n      <collision name='collision_{i}'>\n        <geometry>\n          <box>\n            <size>{length} {wall_thickness} {wall_height}</size>\n          </box>\n        </geometry>\n      </collision>\n      <visual name='visual_{i}'>\n        <geometry>\n          <box>\n            <size>{length} {wall_thickness} {wall_height}</size>\n          </box>\n        </geometry>\n      </visual>\n    </link>\n")
        parts.append("  </model>\n")

        parts.append(footer)
        return ''.join(parts)


def write_boxes_to_stl(segments, wall_thickness, wall_height, output_path):
    """Write an ASCII STL containing all wall boxes (merged into one mesh)."""
    def add_box_triangles(cx, cy, length, thickness, height, ang, triangles_out):
        # local half-sizes
        hx = length / 2.0
        hy = thickness / 2.0
        hz = height / 2.0
        # local corner points
        pts = [
            ( hx,  hy,  hz),
            (-hx,  hy,  hz),
            (-hx, -hy,  hz),
            ( hx, -hy,  hz),
            ( hx,  hy, -hz),
            (-hx,  hy, -hz),
            (-hx, -hy, -hz),
            ( hx, -hy, -hz),
        ]
        # rotate and translate
        ca = math.cos(ang)
        sa = math.sin(ang)
        def world(p):
            x = p[0] * ca - p[1] * sa + cx
            y = p[0] * sa + p[1] * ca + cy
            z = p[2]
            return (x, y, z)

        w = [world(p) for p in pts]

        # faces as tuples of vertex indices (triangles will be two per face)
        faces = [
            (0,1,2,3), # top
            (4,7,6,5), # bottom
            (0,3,7,4), # +x
            (1,5,6,2), # -x
            (0,4,5,1), # +y
            (3,2,6,7), # -y
        ]

        for face in faces:
            a,b,c,d = face
            triangles_out.append((w[a], w[b], w[c]))
            triangles_out.append((w[a], w[c], w[d]))

    triangles = []
    for seg in segments:
        (sx, sy), (ex, ey) = seg
        length = math.hypot(ex - sx, ey - sy)
        cx = (sx + ex) / 2.0
        cy = (sy + ey) / 2.0
        ang = math.atan2(ey - sy, ex - sx)
        add_box_triangles(cx, cy, length, wall_thickness, wall_height, ang, triangles)

    # write ASCII STL
    with open(output_path, 'w') as f:
        f.write('solid maze_walls\n')
        for tri in triangles:
            # compute normal
            (x1,y1,z1),(x2,y2,z2),(x3,y3,z3) = tri
            ux,uy,uz = (x2-x1, y2-y1, z2-z1)
            vx,vy,vz = (x3-x1, y3-y1, z3-z1)
            nx = uy*vz - uz*vy
            ny = uz*vx - ux*vz
            nz = ux*vy - uy*vx
            # normalize if non-zero
            norm = math.hypot(nx, ny, nz)
            if norm > 1e-12:
                nx,ny,nz = nx/norm, ny/norm, nz/norm
            else:
                nx,ny,nz = 0.0,0.0,0.0
            f.write(f'  facet normal {nx} {ny} {nz}\n')
            f.write('    outer loop\n')
            f.write(f'      vertex {x1} {y1} {z1}\n')
            f.write(f'      vertex {x2} {y2} {z2}\n')
            f.write(f'      vertex {x3} {y3} {z3}\n')
            f.write('    endloop\n')
            f.write('  endfacet\n')
        f.write('endsolid maze_walls\n')


def create_world_sdf_with_mesh(segments, mesh_uri, ground_size_x, ground_size_y):
        # produce a world that uses the visual mesh for appearance but keeps
        # per-wall primitive box collisions (safer for physics engines)
        header = """<?xml version='1.0'?>\n<sdf version='1.7'>\n  <world name='maze_world'>\n"""
        footer = "\n  </world>\n</sdf>\n"
        parts = [header]
        parts.append("""
        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <direction>-0.5 0.1 -0.9</direction>
        </light>
        """)
        parts.append(f"  <model name='ground'>\n    <static>true</static>\n    <link name='link'>\n      <collision name='collision'>\n        <geometry>\n          <plane>\n            <normal>0 0 1</normal>\n            <size>{ground_size_x} {ground_size_y}</size>\n          </plane>\n        </geometry>\n      </collision>\n      <visual name='visual'>\n        <geometry>\n          <plane>\n            <normal>0 0 1</normal>\n            <size>{ground_size_x} {ground_size_y}</size>\n          </plane>\n        </geometry>\n      </visual>\n    </link>\n  </model>\n")

        # wall sizing constants
        wall_thickness = WALL_THICKNESS
        wall_height = WALL_HEIGHT

        # Model: visual uses the single mesh, collisions are per-wall boxes
        parts.append(f"  <model name='maze'>\n    <static>true</static>\n")
        # visual at model level
        parts.append(f"    <link name='visual_link'>\n      <visual name='visual'>\n        <geometry>\n          <mesh>\n            <uri>{mesh_uri}</uri>\n          </mesh>\n        </geometry>\n      </visual>\n    </link>\n")

        for i, seg in enumerate(segments):
                (sx, sy), (ex, ey) = seg
                length = math.hypot(ex - sx, ey - sy)
                cx = (sx + ex) / 2.0
                cy = (sy + ey) / 2.0
                ang = math.atan2(ey - sy, ex - sx)
                parts.append(f"    <link name='maze_wall_{i}_link'>\n      <pose>{cx} {cy} {wall_height/2.0} 0 0 {ang}</pose>\n      <collision name='collision_{i}'>\n        <geometry>\n          <box>\n            <size>{length} {wall_thickness} {wall_height}</size>\n          </box>\n        </geometry>\n      </collision>\n    </link>\n")

        parts.append("  </model>\n")

        # default GUI camera centered above the maze
        cam_z = max(1.0, wall_height * 4.0)
        parts.append(f"  <gui>\n    <camera name='camera'>\n      <pose>0 0 {cam_z} 0 -0.6 0</pose>\n    </camera>\n  </gui>\n")

        parts.append(footer)
        return ''.join(parts)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rows', type=int, default=10)
    parser.add_argument('--cols', type=int, default=10)
    parser.add_argument('--cell_size', type=float, default=0.4)
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--output', type=str, default='/tmp/maze_world_generated.sdf')
    parser.add_argument('--compact', action='store_true', help='Emit a single model with many links for walls (faster to load)')
    parser.add_argument('--single-mesh', dest='single_mesh', action='store_true', help='Export walls as a single mesh (STL) and reference it from the world SDF')
    args = parser.parse_args()

    walls = generate_maze(args.rows, args.cols, seed=args.seed)
    segments = walls_to_segments(walls, args.rows, args.cols, args.cell_size)
    ground_x = args.cols * args.cell_size * 2
    ground_y = args.rows * args.cell_size * 2
    # If single_mesh requested, write an STL and generate an SDF that references it
    if args.single_mesh:
        mesh_path = args.output + '.stl'
        write_boxes_to_stl(segments, WALL_THICKNESS, WALL_HEIGHT, mesh_path)
        # create an SDF that uses the mesh for visuals but primitive box collisions
        sdf = create_world_sdf_with_mesh(segments, 'file://' + mesh_path, ground_x, ground_y)
        with open(args.output, 'w') as f:
            f.write(sdf)
        print(f'Wrote single-mesh STL to {mesh_path} and world SDF to {args.output} (mesh referenced for visuals, primitive collisions)')
    else:
        sdf = create_world_sdf(segments, ground_x, ground_y, compact=args.compact)
        with open(args.output, 'w') as f:
            f.write(sdf)
    print(f'Wrote maze world to {args.output} with {len(segments)} wall segments (compact={args.compact})')


if __name__ == '__main__':
    main()
