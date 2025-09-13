## GUI troubleshooting and best-practices

If the Gazebo GUI doesn't appear on your machine but the headless simulation runs fine (the robot and planner are spawned), the most likely cause is missing or misconfigured 3D acceleration on the host/VM. Follow these steps in order:

- Quick (recommended for development): run headless and use the smoke tester and topic/Tf viewers. See `tools/smoke_headless_test.sh`.
- If you need a visible GUI on this machine, prefer enabling 3D acceleration for the VM or running on the host with a GPU — this is the most reliable fix.
- If you can't enable hardware acceleration, try a virtual X server (Xvfb) + software GL. This may work but is slower and sometimes unstable.

Common commands (no sudo):

Headless launch (works for CI / algorithm dev):
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_maze_simulation bringup_launch.py headless:=true planner:=astar
```

Attempt GUI with software GL (sets software GL and Qt plugin):
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=swrast
export QT_QPA_PLATFORM=xcb
ros2 launch robot_maze_simulation bringup_launch.py headless:=false planner:=astar
```

If the above crashes or your environment lacks an X server, try the `tools/run_gui_xvfb.sh` helper which will attempt to run a virtual X server on :99 using Xvfb (no package install performed by the helper):

```bash
tools/run_gui_xvfb.sh --timeout 30
```

If `run_gui_xvfb.sh` complains about missing `Xvfb` or `xwd` then install them with sudo:

```bash
sudo apt update
sudo apt install -y xvfb x11-apps imagemagick
```

Notes:
- On many VMs the fastest, most reliable fix is simply to enable 3D acceleration in the VM settings or run the GUI on the host machine.
- The project provides headless tooling (see `tools/smoke_headless_test.sh`) so you can continue development and CI without the GUI.

This package includes a small maze world generator used at launch.

Launch arguments provided by `bringup_launch.py` (maze generation and caching):

- `maze_rows` (int, default 10): number of rows in the maze grid.
- `maze_cols` (int, default 10): number of columns in the maze grid.
- `cell_size` (float, default 0.4): physical size (meters) of each maze cell.
- `maze_single_mesh` (bool, default false): export maze walls as a single STL and reference it from the SDF (reduces entity count).
- `maze_compact` (bool, default true): use compact per-link model output (fewer entities than separate models).
- `maze_seed` (int, optional): when provided, generation is deterministic and the generated world is cached in `install/.../share/robot_maze_simulation/generated/`.
- `maze_force_regen` (bool, default false): when true, forces regeneration even if a cached seeded world exists.

Examples:

# deterministic cached maze (seeded)
ros2 launch robot_maze_simulation bringup_launch.py \
  maze_seed:=42 \
  maze_rows:=8 \
  maze_cols:=8 \
  cell_size:=0.4 \
  maze_single_mesh:=true \
  headless:=true \
  planner:=astar

# random maze each run
ros2 launch robot_maze_simulation bringup_launch.py maze_rows:=10 maze_cols:=10 headless:=true planner:=astar

Notes:
- Seeded cached worlds are stored under the package share `generated/` directory to make them persistent between runs and visible to collaborators.
- Use `maze_force_regen:=true` to overwrite an existing cached seeded world.
