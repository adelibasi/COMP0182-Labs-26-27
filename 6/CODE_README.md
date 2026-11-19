# Lab 6 package reference

Follow the [Lab 6 worksheet](README.md) for setup, task order and checkpoints. This page is a quick reference to the supplied files and commands.

## Files and configuration

Work from your team's `labs/6` directory. Copy `.env.example` to `.env`, enter the allocated robot name and domain, then follow the worksheet's terminal setup. For direct `ros2` commands, run `source .env` to load the settings exported in the file.

Make's priority is **command-line arguments > `.env` > shell variables > Makefile defaults**. Use the plain commands below and check the values and sources printed by `make help`.

| File or folder | Purpose |
|---|---|
| `.env` | Robot, domain and simulation/physical settings; use the complete `tb_<name>` namespace |
| [exercises/planner.py](exercises/planner.py) | Implement and test your A* function |
| [config/nav_tuning.yaml](config/nav_tuning.yaml) | Parameters for the worksheet's simulation experiment |
| `config/static/` | Supplied Nav2, localisation and RViz configuration |
| `maps/` | Supplied house map for simulation |
| `launch/`, `scripts/`, `packages/` | Supplied ROS launches, adapter and planner bridge |
| `tests/` | Algorithm, adapter and infrastructure checks |

## Commands

Use separate prepared terminals for long-running commands. Stop teleop before Nav2; stop Nav2 before teleop.

| Command | Purpose |
|---|---|
| `make help` | Show commands and the effective ROS settings |
| `make build` | Build the supplied planner packages |
| `make sim` | Start Gazebo and the simulated robot |
| `make localise` | Load the house map and AMCL; simulation only |
| `make rviz` | Open the map, scans, costmaps and path view |
| `make nav` | Start Nav2 with your Student A* planner |
| `make teleop` | Manual control; keep Nav2 stopped |
| `make check-hardware` | Check physical scan, odometry, TF and Twist interfaces |
| `make mapping` | Check hardware and start physical RTAB-Map; staff-managed or supervised |

For simulation, `.env` uses `USE_SIM_TIME=true` and `ROS_LOCALHOST_ONLY=1`. For the physical task, staff allocate the robot/domain and the worksheet changes these values to `false` and `0`. Reload `.env` in each terminal after changing it. Keep one physical RTAB-Map source and one Nav2 stack in the assigned domain.

## Checks

- `make test-a-star` tests your algorithm. `NotImplementedError` is expected until you implement it.
- `make test-package` checks the supplied adapter and packaging.
- `make check` starts its own headless simulation and tests a goal and two waypoints using a supplied route fixture. Run it with other Lab 6 launches stopped and let it clean up. It does **not** test your A*.

Use the worksheet's [troubleshooting section](README.md#troubleshooting--use-the-failing-checkpoint) for failed checkpoints. Restart the ROS 2 daemon only for inconsistent CLI discovery after checking the terminal's settings; it cannot repair missing sensors or TF.
