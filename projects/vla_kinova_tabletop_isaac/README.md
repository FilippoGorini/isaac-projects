# Vla Kinova Tabletop

**Authors:** Filippo Gorini
**Isaac Sim version:** 5.1.0
**ROS 2 distro:** Humble

---

## Overview

This project provides an Isaac Sim simulation environment to test the use of VLA models in simple fixed-arm tabletop scenarios. The robot is a Kinova Gen3 6-DoF arm equipped with a Robotiq 2F-85 parallel-jaw gripper. The project ships Isaac Sim USD scenes and ROS 2 launch files that bring up the `ros2_control` stack using the `ros2_kortex` package published by Kinova Robotics.

---

### Project Structure

```
projects/vla_kinova_tabletop_isaac/
├── .env.example                  # Template environment file; copy to .env
├── .env                          # Local config (never committed)
├── setup.bash                    # Source in every new terminal
├── isaacsim/
│   ├── worlds/                   # Isaac Sim USD scenes (see below)
│   ├── rl_scenes/                # RL scene configs (empty for now)
│   └── startup_scenes/           # Lab startup scenes (empty for now)
└── ros2_ws/
    ├── bootstrap_kinova_ws.sh    # One-shot workspace bootstrap script
    ├── deps.repos                # vcstool manifest for ros2_kortex and deps
    └── src/
        └── vla_kinova_tabletop/  # Main project ROS 2 package
```

---

### Isaac Sim Scenes (`isaacsim/worlds/`)

| File | Description |
|------|-------------|
| `test.usda` | Not used anymore, soon to be deleted |
| `kinova_gen3_6dof_2f85/` | USD asset directory for the robot (base, physics, robot, sensor layers) |
| `kinova_gen3_6dof_2f85.usda` | Robot USDA imported from the original URDF. To better emulate the real hardware, only `robotiq_85_left_knuckle_joint` and `robotiq_85_right_knuckle_joint` were assigned a joint drive; all other joints were made passive. Two additional passive joints were added to close the parallel-gripper loop, connecting the `inner_knuckle` links to the `finger_tip` links, preventing the gripper from disassembling while grasping objects. |
| `kinova_gen3_6dof_2f85_ros2.usda` | References `kinova_gen3_6dof_2f85.usda` and adds the ActionGraph nodes that bridge to ROS 2 (joint states, joint commands, and camera feedback). |
| `kinova_tabletop.usda` | Simple tabletop scenario built from Isaac assets, referencing the ROS 2-ready `kinova_gen3_6dof_2f85_ros2.usda`. |

---

### ROS 2 Package: `vla_kinova_tabletop`

The main project package. It contains:

- **`urdf/`** — Xacro/URDF files describing the Kinova Gen3 + Robotiq 2F-85 for `ros2_control` with the Isaac Sim `TopicBasedSystem` hardware interface. The arm listens on `/isaac_arm_commands` and the gripper on `/isaac_gripper_commands`.
- **`config/ros2_controllers.yaml`** — Controller configuration for `joint_trajectory_controller`, `robotiq_gripper_controller`, and `joint_state_broadcaster`.
- **`config/moveit_controllers.yaml`** — MoveIt 2 controller config that delegates to the above.
- **`config/moveit.rviz`** — RViz preset for MoveIt 2 motion planning.
- **`src/joint_state_merger.cpp`** — Not needed anymore, soon to be deleted.
- **`launch/kinova_controllers.launch.py`** — Starts `robot_state_publisher`, `ros2_control_node`, and spawns `joint_state_broadcaster`, `joint_trajectory_controller`, and `robotiq_gripper_controller`. Use this for basic joint control without motion planning.
- **`launch/kinova_controllers_moveit.launch.py`** — Everything in the above launch file, plus MoveIt 2 `move_group` and RViz. Use this when you need motion planning through MoveIt 2.

---

### Bootstrap Procedure (fresh server)

Run this once on each new cloud server.

**1. Clone your fork and enter the repo:**

```bash
git clone https://github.com/FilippoGorini/isaac-projects.git ~/isaac-projects
cd ~/isaac-projects
```

**2. Copy and source the project specific environment:**

```bash
cp projects/vla_kinova_tabletop_isaac/.env.example projects/vla_kinova_tabletop_isaac/.env
source projects/vla_kinova_tabletop_isaac/.env
```

**3. Bootstrap the host (Docker, NVIDIA runtime, ROS 2, Isaac Sim image):**

```bash
./isaac_vmctl.sh bootstrap
```

**4. Bootstrap the Kinova ROS 2 workspace:**

`bootstrap` writes the ROS 2 setup to `~/.bashrc` but does not source it in the
current shell, so `ROS_DISTRO` is not yet set. Source ROS 2 manually before
running the workspace bootstrap script:

```bash
source /opt/ros/humble/setup.bash   # or jazzy, whichever was installed
cd ~/isaac-projects/projects/vla_kinova_tabletop_isaac/ros2_ws
./bootstrap_kinova_ws.sh
```

This imports `ros2_kortex` and all transitive dependencies via `vcstool`, installs
system packages with `rosdep`, and builds the workspace with `colcon`.

---

### Starting Isaac Sim

After bootstrap, in each new terminal source the project environment first:

```bash
source ~/isaac-projects/projects/vla_kinova_tabletop_isaac/setup.bash
```

Then start Isaac Sim from the repo root:

```bash
cd ~/isaac-projects
./isaac_vmctl.sh start isaacsim
```

Open the Isaac Sim WebRTC Streaming Client on your laptop and connect to the
server IP printed by `./isaac_vmctl.sh check`. From the Isaac Sim GUI, open the
desired scene from `projects/vla_kinova_tabletop_isaac/isaacsim/worlds/` (the
repo is mounted at `/workspace/isaac-projects` inside the container).

For a native GUI inside TigerVNC instead of WebRTC, run the start command from
the terminal inside the VNC desktop:

```bash
./isaac_vmctl.sh start isaacsim --gui
```

---

### Running the ROS 2 Launch Files

Source the project environment in every new terminal:

```bash
source ~/isaac-projects/projects/vla_kinova_tabletop_isaac/setup.bash
```

**Basic joint control (no motion planning):**

```bash
ros2 launch vla_kinova_tabletop kinova_controllers.launch.py
```

Starts `robot_state_publisher`, `ros2_control_node`, and spawns the
`joint_trajectory_controller`, `robotiq_gripper_controller`, and
`joint_state_broadcaster`.

**MoveIt 2 + RViz:**

```bash
ros2 launch vla_kinova_tabletop kinova_controllers_moveit.launch.py
```

Starts everything above plus MoveIt 2 `move_group` and RViz with the project
motion-planning preset.

---

## Access

- Ask your thesis supervisor which GPU resource to use. The supervisor
  coordinates with [Omotoye Shamsudeen Adekoya](https://github.com/Omotoye)
  ([omotoye.adekoya@edu.unige.it](mailto:omotoye.adekoya@edu.unige.it)) or
  Prof. Carmine Recchiuto
  ([carmine.recchiuto@unige.it](mailto:carmine.recchiuto@unige.it)), who set
  up the machine and send credentials.
- Follow the repo-level [setup](../../README.md#quick-start) to install Docker,
  NVIDIA Container Toolkit, and ROS 2 on the assigned machine.
- Copy `.env.example` to `.env` and fill in your values (especially
  `NGC_API_KEY` if required and `ALLOWED_CLIENT_IP` if on a SimplePod VM).

> [!IMPORTANT]
> Keep this project in your fork. Do not keep thesis code, scenes, or model
> progress only on a running GPU server. Cloud GPU servers are deleted after
> the allocated hours.

---

## How to Run

### 1. Start Isaac Sim

From the **repository root**:

```bash
cp projects/<your-project-name>/.env.example projects/<your-project-name>/.env
# edit projects/<your-project-name>/.env
source projects/<your-project-name>/.env
./isaac_vmctl.sh bootstrap        # once on a fresh server; add --verbose for live logs
./isaac_vmctl.sh start isaacsim
```

On SimplePod, if you sourced `configs/simplepod-tigervnc.env` and want the
native Isaac Sim UI inside the VNC desktop instead of WebRTC, run this from
the terminal inside TigerVNC:

```bash
./isaac_vmctl.sh start isaacsim --gui
```

If you are running an Isaac Lab script, keep the same arguments you would pass
to `./isaaclab.sh` and run them through `isaac_vmctl.sh` instead. For example,
headless training looks like:

```bash
source configs/isaac-sim-5.1.0.env
source configs/isaac-lab.env
./isaac_vmctl.sh bootstrap
./isaac_vmctl.sh run isaaclab \
  '-p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Ant-v0 --headless'
```

That bootstrap run manages the pinned Isaac Lab checkout and local Isaac Lab
image automatically.

To see the Isaac Lab GUI, omit `--headless` and run the command from the
terminal inside TigerVNC.

### 2. Source the project environment

In each new terminal:

```bash
source projects/<your-project-name>/setup.bash
```

### 3. Build and run your ROS 2 workspace

```bash
cd projects/<your-project-name>/ros2_ws
colcon build
source install/setup.bash
ros2 launch <your_package> <your_launch_file>.py
```

### 4. Connect to Isaac Sim

Run the connectivity check to get the IP and ports:

```bash
./isaac_vmctl.sh check
```

Open the Isaac Sim WebRTC client and connect to the IP printed above.

For Isaac Lab GUI runs, skip WebRTC and launch the command from the terminal
inside TigerVNC so the viewport opens there.

> [!NOTE]
> On Vast.ai headless jobs, skip WebRTC and use
> [Zenoh](../../zenoh/README.md) with the external port mapped to server
> port `7447`.

### 5. Run a training command

Use `run` for one-shot training jobs so the command exits with the training
process and writes artifacts into the mounted project folder:

```bash
./isaac_vmctl.sh run -- bash -lc 'cd projects/<your-project-name> && python train.py'
```

---

## ROS 2 Package Structure

```
ros2_ws/
└── src/
    └── <your_package>/
        ├── package.xml
        ├── setup.py          # (Python package) or CMakeLists.txt (C++)
        ├── <your_package>/
        │   ├── __init__.py
        │   └── ...
        └── launch/
            └── ...
```

---

## Isaac Sim and RL Scene Structure

```
isaacsim/
├── worlds/           # Isaac Sim world files, USD scenes, robots, environments
├── rl_scenes/        # RL scene assets, task configs, training scene files
└── startup_scenes/   # Lab-provided startup scenes to copy and adapt
```

Put thesis-specific scene files here instead of scattering them across the
repository. Startup scenes are provided as a base when they match your project.
Inside the container, this repository is mounted at `/workspace/isaac-projects`.

---

## Repository Workflow

Work from your own fork of the main repository. Sync your fork regularly so
you receive lab updates to scripts, configs, and startup templates.

If this repository setup breaks, or if the instructions are unclear, open an
[issue on the main repository](https://github.com/RICE-unige/isaac-projects/issues)
and inform [Omotoye](https://github.com/Omotoye).

---

## Saving Training Progress

Use `scripts/project_snapshot.sh` from the repo root to save the project
artifacts and matching repo code state before the cloud server is deleted.

Copy the optional snapshot defaults file if you want to pin include paths,
an rsync target, or a resume command:

```bash
cp projects/<your-project-name>/snapshot.env.example \
  projects/<your-project-name>/.snapshot.env
```

Recommended per-session save:

```bash
./scripts/project_snapshot.sh save --project <your-project-name>
```

Recommended end-of-day save when your git auth is already configured:

```bash
./scripts/project_snapshot.sh save \
  --project <your-project-name> \
  --git-push
```

Restore on a fresh server:

```bash
./scripts/project_snapshot.sh restore \
  --project <your-project-name> \
  --snapshot projects/<your-project-name>/artifacts/snapshots/<snapshot-id>.tar.gz
```

> [!TIP]
> Save a `metadata.yaml` next to each checkpoint with the git commit, command,
> seed, task name, Isaac Sim version, and short notes. Snapshot archives,
> manifests, and checksums live under `artifacts/snapshots/`. The helper uses
> existing SSH or HTTPS git auth only and does not manage tokens.

---

## Isaac Sim Configuration

<!-- Document any Isaac-Sim-specific settings, USD scene paths, or extensions
     your project depends on. -->

| Variable | Value |
|----------|-------|
| `ISAAC_IMAGE` | `nvcr.io/nvidia/isaac-sim:5.1.0` |
| `WEBRTC_SIGNAL_PORT` | `49100` |
| `WEBRTC_STREAM_PORT` | `47998` |
| `VSCODE_REMOTE_ENABLE` | `1` |
| `JUPYTER_ENABLE` | `1` |
| `STUDENT_EXTRA_TOOLS` | unset |
| `TIGERVNC_ENABLE` | `0` |
| `TIGERVNC_PORT` | `5901` |
| `TIGERVNC_GEOMETRY` | `1920x1080` |
| `TIGERVNC_DESKTOP` | `xfce` |
| `ISAAC_EXTRA_ARGS` | _(none)_ |

---

## Troubleshooting

<!-- Add project-specific troubleshooting notes here. -->

See also the repo-level [Troubleshooting](../../README.md#troubleshooting) section.
