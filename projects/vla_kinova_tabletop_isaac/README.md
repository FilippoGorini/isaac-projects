# Vla Kinova Tabletop Isaac

**Authors:** Filippo Gorini

![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-5.1.0-76B900?logo=nvidia&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)

---

## Overview

This project provides an Isaac Sim simulation environment to test the use of VLA models in simple fixed-arm tabletop scenarios. The robot is a Kinova Gen3 6-DoF arm equipped with a Robotiq 2F-85 parallel-jaw gripper. The project ships Isaac Sim USD scenes and ROS 2 launch files that bring up the `ros2_control` stack using the `ros2_kortex` package published by Kinova Robotics.

---

## Project Structure

```
projects/vla_kinova_tabletop_isaac/
├── .env.example                  # Template environment file; copy to .env
├── .env                          # Local config (never committed)
├── setup.bash                    # Source in every new terminal
├── isaacsim/
│   ├── worlds/                   # Isaac Sim USD scenes (see below)
│   ├── rl_scenes/                # RL scene configs (empty for now)
│   └── startup_scenes/           # Lab startup scenes (empty for now)
├── openpi_policy_server/
│   ├── bootstrap_openpi.sh       # One-shot openpi bootstrap script
│   └── src/                      # Policy server entry point and kinova policy definition
└── ros2_ws/
    ├── bootstrap_kinova_ws.sh    # One-shot workspace bootstrap script
    ├── deps.repos                # vcstool manifest for ros2_kortex and deps
    └── src/
        ├── vla_kinova_tabletop/  # Robot description and controller config
        └── vla_policy_client/    # ROS 2 WebSocket client for the VLA policy server
```

---

## Bootstrap Procedure (fresh server)

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

**4. Bootstrap the openpi policy server (you can skip this if you don't want to run the $\pi_0/\pi_{0.5}$ VLA models) :**

```bash
cd ~/isaac-projects/projects/vla_kinova_tabletop_isaac/openpi_policy_server
./bootstrap_openpi.sh
```

This clones the `openpi` repository into `external/openpi`, creates a `uv`-managed
virtual environment for the policy server, and installs `openpi-client` plus its
dependencies into system Python so the ROS 2 policy client node can import them.

**5. Bootstrap the Kinova ROS 2 workspace:**

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

## Starting Isaac Sim

After bootstrap, source the project environment in each new terminal:

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

## Isaac Sim Scenes (`isaacsim/worlds/`)

| File | Description |
|------|-------------|
| `kinova_gen3_6dof_2f85/` | Base USD asset directory for the robot (base, physics, robot, sensor layers). |
| `kinova_gen3_6dof_2f85.usda` | Robot USDA imported from `kinova_gen3_6dof_2f85.urdf`. Mirrors the original URDF: all 6 gripper joints are driven. |
| `kinova_gen3_6dof_2f85_ros2.usda` | References `kinova_gen3_6dof_2f85.usda` and adds the ActionGraph nodes that bridge to ROS 2 (joint states, joint commands, and camera feedback). To better emulate the real hardware, only `robotiq_85_left_knuckle_joint` and `robotiq_85_right_knuckle_joint` are assigned a joint drive; all other gripper joints are passive. Two additional passive joints close the parallel-gripper loop by connecting the `inner_knuckle` links to the `finger_tip` links, preventing the gripper from disassembling while grasping. To avoid articulation errors, `robotiq_85_left_inner_knuckle_joint` and `robotiq_85_right_inner_knuckle_joint` were excluded from the articulation and are therefore expected to show small position errors. |
| `kinova_tabletop.usda` | Simple tabletop scenario built from Isaac assets, referencing the ROS 2-ready `kinova_gen3_6dof_2f85_ros2.usda`. |

---

## ROS 2 Packages

| Package | Description |
|---------|-------------|
| [`vla_kinova_tabletop`](#vla_kinova_tabletop) | Custom wrapper over the upstream `kortex_description` and `robotiq_description` packages, providing a project-specific robot description and controller setup for Isaac Sim. |
| [`vla_policy_client`](#vla_policy_client) | ROS 2 node that connects to the $\pi_0$ VLA policy server over WebSocket, subscribes to joint states and camera images, and publishes joint trajectories and gripper commands at inference rate. |

---

## `vla_kinova_tabletop`

This package is a customized wrapper over the upstream `ros2_kortex` packages from Kinova Robotics. Rather than modifying those packages directly, it provides its own xacro files and configuration to expose separate `ros2_control` command topics for the arm (`/isaac_arm_commands`) and gripper (`/isaac_gripper_commands`), instead of the single `/isaac_joint_commands` topic used by the upstream packages. It also ships the controller and MoveIt 2 configuration specific to this project.

### Package Files

- **`urdf/`**: Project-specific xacro files wrapping the upstream robot description. See [URDF Files](#urdf-files-urdf) below.
- **`config/ros2_controllers.yaml`**: Controller configuration for `joint_trajectory_controller`, `robotiq_gripper_controller`, and `joint_state_broadcaster`.
- **`config/moveit_controllers.yaml`**: MoveIt 2 controller config.
- **`config/moveit.rviz`**: RViz preset for MoveIt 2 motion planning.

### URDF Files (`urdf/`)

The xacro files are layered: `gen3.xacro` is the entry point referenced by the launch files; each layer includes the next.

| File | Role |
|------|------|
| `gen3.xacro` | Top-level entry point. Declares all xacro arguments and instantiates the `load_robot` macro from `kortex_robot.xacro`. This is the file passed to `xacro` by the launch files. |
| `kortex_robot.xacro` | `load_robot` macro. Orchestrates the full robot assembly: loads the arm via `gen3_macro.xacro` and the gripper via `robotiq_2f_85_macro.xacro`, routing the arm and gripper command topics separately. |
| `gen3_macro.xacro` | `load_arm` macro. Defines all kinematic links and joints of the Gen3 arm (`base_link` through `end_effector_link`), with optional wrist-camera frames when `vision:=true`. Instantiates the arm `ros2_control` block via `kortex.ros2_control.xacro`. |
| `kortex.ros2_control.xacro` | `ros2_control` hardware block for the arm's 6 joints (`joint_1`–`joint_6`). Selects the hardware plugin via xacro conditionals: `topic_based_ros2_control/TopicBasedSystem` for Isaac Sim, `mock_components/GenericSystem` for fake hardware, or `kortex_driver/KortexMultiInterfaceHardware` for the real robot. Exposes position command interfaces and position/velocity/effort state interfaces for each joint. |
| `robotiq_2f_85_macro.xacro` | `load_gripper` macro. Includes `robotiq_2f_85_macro.urdf.xacro` and instantiates the `robotiq_gripper` macro, conditionally enabling the `ros2_control` block depending on the hardware mode. |
| `robotiq_2f_85_macro.urdf.xacro` | `robotiq_gripper` macro. Full kinematic model of the Robotiq 2F-85 (all links and joints for the parallel-gripper mechanism). Instantiates the gripper `ros2_control` block via `2f_85.ros2_control.xacro`. |
| `2f_85.ros2_control.xacro` | `ros2_control` hardware block for the gripper. The actuated joint is `robotiq_85_left_knuckle_joint`; in simulation all dependent mimic joints are registered as well. Uses `topic_based_ros2_control/TopicBasedSystem` for Isaac Sim or `robotiq_driver/RobotiqGripperHardwareInterface` for the real gripper. |
| `kinova_gen3_6dof_2f85.urdf` | Static URDF export of the full robot. Not used by the launch files, which process `gen3.xacro` directly: this is just the URDF which was imported into Isaac Sim. |

### Launch Files

Source the project environment in every new terminal before running any launch file:

```bash
source ~/isaac-projects/projects/vla_kinova_tabletop_isaac/setup.bash
```

Both launch files accept the same arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim` | `true` | Set to `true` for Isaac Sim, `false` for the real robot. Controls the hardware interface, `use_sim_time`, and whether `robot_ip` is forwarded to the driver. |
| `robot_ip` | `192.168.11.11` | IP address of the real Kinova arm. Ignored when `use_sim:=true`. |
| `auto_home` | `false` | Run the homing script after the controllers come up. Always enabled in simulation; opt-in on the real robot. |

---

*Basic joint control — Isaac Sim (no motion planning):*

```bash
ros2 launch vla_kinova_tabletop kinova_controllers.launch.py
```

*Basic joint control — real robot:*

```bash
ros2 launch vla_kinova_tabletop kinova_controllers.launch.py use_sim:=false robot_ip:=192.168.11.11
```

Starts `robot_state_publisher` and `ros2_control_node`, then spawns `joint_state_broadcaster`, `joint_trajectory_controller`, and `robotiq_gripper_controller`. Use this when you only need to send joint trajectories or test the `ros2_control` bridge without motion planning.

---

*MoveIt 2 + RViz — Isaac Sim:*

```bash
ros2 launch vla_kinova_tabletop kinova_controllers_moveit.launch.py
```

*MoveIt 2 + RViz — real robot:*

```bash
ros2 launch vla_kinova_tabletop kinova_controllers_moveit.launch.py use_sim:=false robot_ip:=192.168.11.11
```

Starts everything above, plus MoveIt 2 `move_group` and RViz with the project motion-planning preset. Use this when you need full motion planning through MoveIt 2.

To also run the homing script when connecting to the real robot:

```bash
ros2 launch vla_kinova_tabletop kinova_controllers_moveit.launch.py use_sim:=false robot_ip:=192.168.11.11 auto_home:=true
```

---

## `vla_policy_client`

This package implements the ROS 2 side of the VLA inference loop. It connects to the $\pi_0$ policy server (started via `openpi_policy_server/scripts/serve_kinova.sh`) over a WebSocket using `openpi-client`, and drives the robot at inference rate by publishing joint trajectories and gripper commands.

### Node: `policy_client`

**Subscriptions:**

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current arm and gripper joint positions. |
| `/isaac_external_camera/color/image_raw` | `sensor_msgs/Image` | Base/scene RGB camera feed. |
| `/isaac_wrist_camera/color/image_raw` | `sensor_msgs/Image` | Wrist RGB camera feed. |

**Publications:**

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 50-step arm trajectory chunk (absolute joint positions). |
| `/robotiq_gripper_controller/gripper_cmd` | `control_msgs/GripperCommand` (action) | Gripper position goal derived from the last action in the chunk. |

**Parameters** (set in `config/client.yaml`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `policy_host` | `localhost` | Hostname or IP of the policy server. |
| `policy_port` | `8000` | WebSocket port of the policy server. |
| `prompt` | `pick up the object` | Language instruction passed to the VLA model. |
| `control_hz` | `50.0` | Control frequency; determines per-step time delta. |
| `chunk_size` | `50` | Number of action steps per inference call. |

### Launch File

Make sure the policy server is already running (`serve_kinova.sh`) and Isaac Sim is streaming joint states and camera images before launching the client.

```bash
ros2 launch vla_policy_client policy_client.launch.py prompt:="Pick up the green cube"
```

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
