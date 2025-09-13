#!/usr/bin/env python3
"""Generate a maze using the included generator and verify grouping best-practice.

Writes cached generated SDF into src/robot_maze_simulation/generated/ by default when a seed is
provided. Verifies that the SDF contains a top-level model named 'maze' and that an STL exists
when single-mesh was requested.
"""
import argparse
import os
import subprocess
import sys


def find_generator():
    # prefer workspace src generator
    ws_candidate = os.path.normpath(os.path.join(os.getcwd(), 'src', 'robot_maze_simulation', 'tools', 'generate_maze_sdf.py'))
    if os.path.exists(ws_candidate):
        return ws_candidate
    # fallback to installed path inside install/ if present
    inst_candidate = os.path.normpath(os.path.join(os.getcwd(), 'install', 'robot_maze_simulation', 'share', 'robot_maze_simulation', 'tools', 'generate_maze_sdf.py'))
    if os.path.exists(inst_candidate):
        return inst_candidate
    return ws_candidate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rows', type=int, default=10)
    parser.add_argument('--cols', type=int, default=10)
    parser.add_argument('--cell_size', type=float, default=0.4)
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--compact', action='store_true')
    parser.add_argument('--single-mesh', dest='single_mesh', action='store_true')
    parser.add_argument('--force', action='store_true', help='force regen even if cache exists')
    args = parser.parse_args()

    generator = find_generator()
    if not os.path.exists(generator):
        print('Generator not found at', generator, file=sys.stderr)
        return 2

    # compute cache dir (dev-friendly)
    cache_dir = os.path.normpath(os.path.join(os.getcwd(), 'src', 'robot_maze_simulation', 'generated'))
    os.makedirs(cache_dir, exist_ok=True)

    cs_str = f"{args.cell_size:.2f}".replace('.', 'p')
    mesh_suffix = '_mesh' if args.single_mesh else ''
    out_name = f'maze_seed{args.seed}_r{args.rows}_c{args.cols}_cs{cs_str}{mesh_suffix}.sdf' if args.seed is not None else 'maze_last.sdf'
    out_path = os.path.join(cache_dir, out_name)

    # run generator if needed
    if args.seed is not None and os.path.exists(out_path) and not args.force:
        print('Using cached world at', out_path)
    else:
        cmd = ['python3', generator, '--rows', str(args.rows), '--cols', str(args.cols), '--cell_size', str(args.cell_size), '--output', out_path]
        if args.seed is not None:
            cmd += ['--seed', str(args.seed)]
        if args.compact:
            cmd.append('--compact')
        if args.single_mesh:
            cmd.append('--single-mesh')
        print('Running generator:', ' '.join(cmd))
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            print('Generator failed:', e, file=sys.stderr)
            return 3

    # verify SDF contains top-level model 'maze'
    try:
        with open(out_path, 'r') as f:
            data = f.read()
    except Exception as e:
        print('Failed to read generated world:', out_path, e, file=sys.stderr)
        return 4

    if "<model name='maze'" in data or '<model name="maze"' in data:
        print('Verified: top-level model named "maze" present in', out_path)
    else:
        print('Verification failed: top-level model named "maze" not found in', out_path, file=sys.stderr)
        return 5

    # if single-mesh, ensure mesh file exists
    if args.single_mesh:
        mesh_path = out_path + '.stl'
        if os.path.exists(mesh_path):
            print('Verified: single-mesh STL exists at', mesh_path)
        else:
            print('Verification failed: expected STL at', mesh_path, 'not found', file=sys.stderr)
            return 6

    print('All checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
