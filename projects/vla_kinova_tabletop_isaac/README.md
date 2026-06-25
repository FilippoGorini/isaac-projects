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
├── set_dds_udp_buffers.sh         # Re-run after every reboot: raises kernel UDP buffers for reliable camera topics
├── set_quest_adp_connection.sh    # Re-run after replugging the Quest's USB cable: tunnels the teleop link over USB instead of WiFi
├── isaacsim/
│   ├── worlds/                   # Isaac Sim USD scenes (see below)
│   ├── rl_scenes/                # RL scene configs (empty for now)
│   └── startup_scenes/           # Lab startup scenes (empty for now)
├── openpi_policy_server/
│   ├── bootstrap_openpi.sh       # One-shot openpi bootstrap script
│   └── src/                      # Policy server entry point and kinova policy definition
└── ros2_ws/
    ├── bootstrap_kinova_ws.sh         # Robot + cameras: ros2_kortex, kinova_vision, RealSense
    ├── bootstrap_quest_teleop.sh      # Quest 3 teleop stack (ROS-TCP-Endpoint + Quest2ROS2)
    ├── bootstrap_lerobot_ros.sh       # Adds lerobot_ros (data recording) to the workspace
    ├── kinova_deps.repos              # vcstool manifest for ros2_kortex + kinova_vision
    ├── teleop_deps.repos              # vcstool manifest for the Quest teleop packages
    └── src/
        ├── vla_kinova_description/         # Customized URDF/Xacro robot description
        ├── vla_kinova_bringup/              # ros2_control + MoveIt 2 launches and configs
        ├── vla_kinova_sensors/              # RealSense D435 + Kinova wrist camera bringup
        ├── vla_kinova_teleop/               # Quest 3 twist-based EE pose tracking
        ├── vla_kinova_data_collection/      # LeRobot dataset recording
        └── vla_policy_client/                # ROS 2 WebSocket client for the VLA policy server
```

---

## Bootstrap Procedure (fresh machine)

Run this once on each new machine, either a cloud GPU server or a local Ubuntu 22.04
desktop wired to the real robot. Not every step is needed for every use case:

- **Steps 1–3** set up the remote host with ROS2 Humble, Docker, IsaacSim etc (can be skipped otherwise).
- **Step 4** (openpi) is only for running the $\pi_0/\pi_{0.5}$ policy server.
- **Steps 5–7** are the real-robot stack (arm + cameras, Quest teleop, dataset
  recorder). They run natively and don't require Isaac Sim. For pure real-hardware
  use on a local desktop you still need ROS 2 Humble.

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

This imports `ros2_kortex` (arm + gripper) and the Kinova wrist camera
(`kinova_vision`) plus all transitive dependencies via `vcstool`, apt-installs
the Intel RealSense stack (`librealsense2` SDK + `realsense2` ROS 2 wrapper) and
the GStreamer runtime the wrist camera needs, resolves the rest with `rosdep`,
and builds the workspace with `colcon`. After this step the robot and both
cameras (RealSense external + Kinova wrist) are ready.

> [!NOTE]
> The ZED 2 is supported as an optional alternative external camera but is **not**
> installed by the bootstrap, because its `zed_wrapper` needs the proprietary ZED
> SDK (v5.2) and CUDA. Install those manually from
> [stereolabs.com](https://www.stereolabs.com/developers/release/latest/), then
> `git clone https://github.com/stereolabs/zed-ros2-wrapper.git` into
> `ros2_ws/src/` and rebuild. Only needed if you launch with `launch_zed:=true`.

**6. Bootstrap the Quest 3 teleoperation stack (skip if you don't teleoperate the real robot):**

```bash
source /opt/ros/humble/setup.bash   # if not already sourced
cd ~/isaac-projects/projects/vla_kinova_tabletop_isaac/ros2_ws
./bootstrap_quest_teleop.sh
```

This imports the Unity `ROS-TCP-Endpoint` and the `FilippoGorini/Quest2ROS2` fork
(pinned in `teleop_deps.repos`), generates the `quest2ros` message package the Quest
app expects (created locally from `Quest2ROS2/Files_for_msg_pkg`), installs
`tf_transformations`, and rebuilds the workspace. Required for the
`vla_kinova_teleop` launch files.

**7. Bootstrap the lerobot_ros data-recording layer (skip if you don't plan to record datasets):**

```bash
cd ~/isaac-projects/projects/vla_kinova_tabletop_isaac/ros2_ws
./bootstrap_lerobot_ros.sh
```

This clones the personal fork `FilippoGorini/lerobot_ros` on the `humble-patches`
branch (Python 3.10 compatibility patches + a few bugfixes), installs `uv` and a
Rust toolchain if missing, creates a `--system-site-packages` venv at
`ros2_ws/src/lerobot_ros/.venv` (inherits Humble's `rclpy`), installs `lerobot`
0.4.3 + `rust_py_timer`, builds `lerobot_interfaces` and `lerobot_ros` with
`colcon`, and rewrites the entry-point shebangs to point at the venv's Python.

Re-run the script after any `colcon build` that touches `lerobot_ros`, otherwise
the entry-point shebangs get reset to `/usr/bin/python3` (which can't import
`lerobot`) and `ros2 run lerobot_ros dataset_recorder` will crash on startup.

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
| [`vla_kinova_description`](#vla_kinova_description) | Project-specific description package wrapping the upstream `kortex_description` and `robotiq_description` packages, with separate `ros2_control` command topics for the arm and gripper. |
| [`vla_kinova_bringup`](#vla_kinova_bringup) | ros2_control + MoveIt 2 controller configuration and launch files. Brings up the robot in either Isaac Sim or on real hardware. |
| [`vla_kinova_sensors`](#vla_kinova_sensors) | Camera bringup for the Intel RealSense D435 (external) and the Kinova wrist camera via `kinova_vision` (RGB only); ZED 2 supported as an optional alternative external camera. |
| [`vla_kinova_teleop`](#vla_kinova_teleop) | Twist-based teleoperation that subscribes to a target EE pose (from the Quest right-arm controller) and drives the Kortex twist controller. |
| [`vla_kinova_data_collection`](#vla_kinova_data_collection) | TOML-driven dataset recording on top of `lerobot_ros`. Provides a launch file for the recorder node and a script to automate the data-collection process based on predefined TOML configuration files for each recording session (tasks, number of episodes etc) |
| [`vla_policy_client`](#vla_policy_client) | ROS 2 node that connects to the $\pi_0$ VLA policy server over WebSocket, subscribes to joint states and camera images, and publishes joint trajectories and gripper commands at inference rate. |

---

## `vla_kinova_description`

Hosts the URDF / xacro stack for the Kinova Gen3 6-DoF arm + Robotiq 2F-85 gripper. Wraps the upstream `kortex_description` and `robotiq_description` packages without modifying them, exposing separate `ros2_control` command topics for the arm (`/isaac_arm_commands`) and gripper (`/isaac_gripper_commands`) instead of the single `/isaac_joint_commands` used upstream.

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

---

## `vla_kinova_bringup`

Brings up `ros2_control`, the controllers, and optionally MoveIt 2. Depends on `vla_kinova_description` for the URDF. Also ships the `home_robot.py` script (auto-run after JTC spawn in sim, optional on real robot) and the abandoned MoveIt-Servo `pose_tracking_node` (not used anymore, the teleop now uses the Kortex twist controller via `vla_kinova_teleop`).

### Package Files

- **`config/ros2_controllers.yaml`**: Controller setup for `joint_trajectory_controller`, `robotiq_gripper_controller`, `joint_state_broadcaster`, plus the real-robot-only `twist_controller`, `gripper_velocity_controller`, `gripper_position_controller`, and `fault_controller`.
- **`config/moveit_controllers.yaml`**: MoveIt 2 controller config.
- **`config/moveit.rviz`**, **`config/servo.rviz`**: RViz presets for MoveIt motion planning and for Servo/Twist Teleop.
- **`config/joint_limits.yaml`**, **`config/servo.yaml`**, **`config/pose_tracking_settings.yaml`**: MoveIt Servo parameters.
- **`scripts/home_robot.py`**: Sends a single `JointTrajectory` to the home pose.
- **`src/pose_tracking_node.cpp`**: MoveIt-Servo PoseTracking wrapper (abandoned, kept as reference).

### Launch Files

Source the project environment in every new terminal before running any launch file:

```bash
source ~/isaac-projects/projects/vla_kinova_tabletop_isaac/setup.bash
```

The three bringup launchers share this set of arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim` | `true` | `true` for Isaac Sim, `false` for the real robot. Controls the hardware interface, `use_sim_time`, and whether `robot_ip` is forwarded to the driver. |
| `robot_ip` | `192.168.50.12` | IP address of the real Kinova arm. Ignored when `use_sim:=true`. |
| `auto_home` | `false` | Run the homing script after the controllers come up. Always enabled in sim; optional on real robot. |
| `gripper_max_velocity` | `100.0` | Gripper go-to speed [0–100%]. Lowering helps smoothing the discrete go-to stepping of the gripper when using the forward position controller. |
| `gripper_max_force` | `100.0` | Gripper grasp force [0-100%]. Lower it for delicate objects. Only applied in low-level cyclic mode; high-level twist teleop ignores it. |
| `tf_publish_rate` | `200.0` | `robot_state_publisher` /tf publish frequency [Hz]. |

The MoveIt, MoveIt-Servo and Twist Pose Tracking launchers additionally accept `launch_rviz` (default `true`).

---

*Basic joint control, Isaac Sim (no motion planning):*

```bash
ros2 launch vla_kinova_bringup kinova_controllers.launch.py
```

*Basic joint control, real robot:*

```bash
ros2 launch vla_kinova_bringup kinova_controllers.launch.py use_sim:=false robot_ip:=192.168.50.12
```

Starts `robot_state_publisher` and `ros2_control_node`, then spawns `joint_state_broadcaster`, `joint_trajectory_controller`, and `robotiq_gripper_controller`. On the real robot, the inactive `twist_controller`, `gripper_velocity_controller`, `gripper_position_controller`, and `fault_controller` are also pre-loaded so they can be switched in later (e.g. by the teleop stack).

---

*MoveIt 2 + RViz, Isaac Sim:*

```bash
ros2 launch vla_kinova_bringup kinova_controllers_moveit.launch.py
```

*MoveIt 2 + RViz, real robot:*

```bash
ros2 launch vla_kinova_bringup kinova_controllers_moveit.launch.py use_sim:=false robot_ip:=192.168.50.12
```

Same as the basic launcher plus MoveIt 2 `move_group` and RViz with the project motion-planning preset.

To also run the homing script when connecting to the real robot:

```bash
ros2 launch vla_kinova_bringup kinova_controllers_moveit.launch.py use_sim:=false robot_ip:=192.168.50.12 auto_home:=true
```

---

*Legacy MoveIt Servo path (abandoned for now due to jerky movement and singularity handling issues):*

```bash
ros2 launch vla_kinova_bringup kinova_controllers_moveit_servo.launch.py
```

Brings up MoveIt Servo and the C++ `pose_tracking_node`. **Not used anymore due to singularity and vibration issues**

---

## `vla_kinova_sensors`

Camera bringup for the RGB feeds consumed by the VLA policy and by the data-recording pipeline. By default it brings up two cameras, both RGB-only (depth disabled — the VLA doesn't need it) and both publishing **raw** (the compressed/theora transports are turned off to save CPU, since we only ever record raw):

- **External camera:** Intel RealSense D435, run directly via `realsense2_camera` on `/realsense/d435/color/image_raw`.
- **Wrist camera:** Kinova Vision module via our trimmed `wrist_camera.launch.py` (the upstream `kinova_vision` color path with the compressed/theora encoders disabled) on `/camera/color/image_raw`.

The ZED 2 is supported as an **optional alternative external camera** (`launch_zed:=true launch_realsense:=false`) but requires the manually-installed ZED SDK (see the bootstrap notes).

### Launch File

| Argument | Default | Description |
|----------|---------|-------------|
| `launch_wrist` | `true` | Bring up the Kinova wrist camera (RGB only, raw, depth disabled). |
| `launch_realsense` | `true` | Bring up the Intel RealSense D435 (RGB only; recorded as `observation.images.external`). |
| `realsense_color_profile` | `640x480x30` | RealSense color profile `WIDTHxHEIGHTxFPS`. Valid combos depend on the USB link (`rs-enumerate-devices -c`). |
| `launch_zed` | `false` | Bring up the ZED 2 instead of / alongside the RealSense (needs the ZED SDK). |
| `zed_camera_model` | `zed2` | ZED model passed to `zed_wrapper`. |
| `kinova_ip` | `192.168.50.12` | IPv4 address of the Kinova arm; forwarded to the wrist camera as `device`. |
| `zed_disable_depth` | `true` | Sets `depth.depth_mode:=NONE` on the ZED, which also disables point cloud, positional tracking and every other module that depends on depth extraction. |
| `zed_resolution` | `HD720` | ZED resolution. One of `HD2K`, `HD1080`, `HD720`, `VGA`, `AUTO`. |
| `zed_grab_fps` | `30` | ZED internal grab frame rate in Hz. Allowed values depend on the resolution: `HD2K@15`, `HD1080@15/30`, `HD720@15/30/60`, `VGA@15/30/60/100`. |
| `zed_pub_fps` | `0.0` | ZED image publish rate in Hz. `0` = no limit (matches the grab rate). |

The ZED-specific arguments above are bundled into a single `param_overrides` string and forwarded to the upstream `zed_camera.launch.py`.

*Default (RealSense external + Kinova wrist):*

```bash
ros2 launch vla_kinova_sensors cameras.launch.py
```

*Wrist only / external only:*

```bash
ros2 launch vla_kinova_sensors cameras.launch.py launch_realsense:=false
ros2 launch vla_kinova_sensors cameras.launch.py launch_wrist:=false
```

*Higher-resolution RealSense RGB:*

```bash
ros2 launch vla_kinova_sensors cameras.launch.py realsense_color_profile:=1280x720x30
```

*Use the ZED instead of the RealSense as the external camera:*

```bash
ros2 launch vla_kinova_sensors cameras.launch.py launch_zed:=true launch_realsense:=false
```

### Reliable camera topic rates (run after every reboot)

On machines without enlarged kernel UDP buffers, DDS drops fragments of large messages
like raw camera frames under load, causing low/unstable rates on `/realsense/d435/color/image_raw` (for example).
Fix by raising `net.core.rmem_max`/`wmem_max` for the session:

```bash
./set_dds_udp_buffers.sh
```

This is intentionally **not** persisted to `/etc/sysctl.d` as the lab's workstation is shared with others, remember to re-run after every reboot.

---

## `vla_kinova_teleop`

Twist-based end-effector pose tracking for the real Kinova Gen3, driven by a Meta Quest 3 (the `quest2ros2` right-arm controller publishes the desired EE pose on `/target_frame`). The `twist_pose_tracking_node` runs a per-axis PID on the 6D pose error (with respect to the control frame (`fingertips_frame`), which can be offset from the end effector one) in `base_link`, saturates the resulting twist, rotates it into the firmware TOOL frame, and publishes on the Kortex `twist_controller` command topic.

The Quest controller (in the `FilippoGorini/Quest2ROS2` fork) supports two gripper modes selected via the `gripper_mode` parameter:

- `binary`: each squeeze of the index/middle trigger sends a single discrete open/close goal to the `robotiq_gripper_controller` action server.
- `velocity`: hold-to-move continuous teleop via `gripper_velocity_controller`.

In either mode the Quest controller also publishes a latched `JointState` on `/gripper_command/state` with the most recently commanded position (0.0 open / 0.8 closed), which the recorder uses as the gripper dimension of the `action` column.

### Package Files

- **`scripts/twist_pose_tracking_node.py`**: The teleop tracking node.
- **`config/twist_pose_tracking.yaml`**: PID gains, saturation limits, stale-target handling.
- **`scripts/gripper_sine_test.py`**, **`scripts/servo_sine_test.py`**: helper diagnostic scripts.

### Launch Files

#### `kinova_pose_tracking_twist.launch.py`

Brings up the robot side of the teleop loop.

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_ip` | `192.168.50.12` | IP address of the real Kinova arm. |
| `launch_rviz` | `false` | Bring up RViz alongside the teleop stack. |
| `tf_publish_rate` | `200.0` | Forwarded to `kinova_controllers.launch.py`. |

```bash
ros2 launch vla_kinova_teleop kinova_pose_tracking_twist.launch.py robot_ip:=192.168.50.12
```

Includes `vla_kinova_bringup`'s `kinova_controllers.launch.py` (with `use_sim:=false`), switches the active controllers from `joint_trajectory_controller` + `robotiq_gripper_controller` to `twist_controller` + `gripper_velocity_controller`, and starts `twist_pose_tracking_node`. **Real robot only.**

#### `quest_bringup.launch.py`

Brings up the Quest side of the teleop loop: the ROS-TCP endpoint that the Quest VR app connects to, plus the `q2r2_bringup` right-arm controller node that converts Quest controller poses into the `/target_frame` topic consumed by `twist_pose_tracking_node`.

| Argument | Default | Description |
|----------|---------|-------------|
| `ros_ip` | `0.0.0.0` | Address the ROS-TCP endpoint binds to (`0.0.0.0` accepts any client). |
| `ros_tcp_port` | `10000` | TCP port the Quest VR app connects to. |

```bash
ros2 launch vla_kinova_teleop quest_bringup.launch.py
```

Replaces the manual two-terminal workflow (`ros2 launch ros_tcp_endpoint endpoint.py` + `ros2 run q2r2_bringup right_arm_controller`). Run alongside `kinova_pose_tracking_twist.launch.py` to get the full teleop stack.

### Reliable Quest connection (if WiFi causes hiccups)

Lab WiFi can be too unreliable for teleop (we measured 100-300ms+ delivery stalls), causing the arm to
freeze briefly then snap to the target. Fix by tethering the Quest over USB instead:

```bash
./set_quest_adp_connection.sh
```

Then, in the Quest2ROS app on the headset, set the server IP to `127.0.0.1`. Re-run the script every
time you unplug/replug the USB cable.

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

## `vla_kinova_data_collection`

Project-specific dataset-recording layer that sits on top of `lerobot_ros`. Holds the Kinova-specific recorder TOML (`config/kinova_pi05.toml`), one or more "session" TOMLs describing what to record (`sessions/*.toml`), a convenience launch file for the recorder, and a single-keystroke driver that automates the recorder's service calls during teleoperation.

The lerobot_ros recorder itself (the `dataset_recorder` node) lives in `ros2_ws/src/lerobot_ros/` and is installed by `bootstrap_lerobot_ros.sh`. This package is the thin Kinova wrapper around it.

### Package Files

- **`config/kinova_pi05.toml`**: Recorder configuration. Sets `fps=30`, `robot_type=kinova_gen3_6dof_2f85`, `dataset_root`, and topic subscriptions. Two `/joint_states` subscriptions (one tagged `observation`, one tagged `action` for the arm joints), plus `/gripper_command/state` as the gripper dimension of the action, plus the RealSense D435 RGB feed (640x480) and the Kinova wrist RGB feed (320x240) as image topics. The recorded image dimensions match the published topics exactly, so lerobot_ros records them verbatim (no resize, no JPEG round-trip).
- **`sessions/example.toml`**: Example recording session: `dataset_name`, plus an ordered list of `{prompt, episodes}` tasks. 
- **`launch/lerobot_recorder.launch.py`**: Wraps `ros2 run lerobot_ros dataset_recorder` so you don't have to expand the config path each time; also sets `HF_HUB_OFFLINE=1` so reopening an existing dataset doesn't try to fetch it from Hugging Face.
- **`vla_kinova_data_collection/record_session.py`**: Script to automate data-collection given a TOML plan for the recording session. Connects to the recorder's services (`/new_dataset`, `/start_episode`, `/end_episode`, `/store_episodes`, `/finalize_dataset`), walks through the slots in a session TOML, and handles keep/discard/skip/quit from the keyboard.

### Launch File

#### `lerobot_recorder.launch.py`

Starts the lerobot_ros recorder (kinova TOML is loaded by default).

| Argument | Default | Description |
|----------|---------|-------------|
| `config` | `kinova_pi05.toml` from this package's `share/config/` | Path to the recorder TOML. Override to point at a different config. |

```bash
ros2 launch vla_kinova_data_collection lerobot_recorder.launch.py
```

### Console Script

#### `record_session`

Interactive driver. Reads a session TOML, opens the recorder's services, and walks through `{task, episode}` slots while you teleoperate. Keys (no Enter needed): `SPACE`/`Enter` start, `k` keep, `d` discard (retries the slot), `s` skip, `q` quit. During an in-progress recording, `q` discards the in-flight episode and exits cleanly.

The `--session` value can be a bare name (looked up in CWD first, then in `share/vla_kinova_data_collection/sessions/`) or a relative/absolute path:

```bash
ros2 run vla_kinova_data_collection record_session --session example
ros2 run vla_kinova_data_collection record_session -s ~/my_thesis_session.toml
```

When the session ends (or you quit after keeping episodes), the script calls `/finalize_dataset`, which waits for the background encoding to finish and writes the dataset metadata before returning. You do **not** need to Ctrl-C the recorder to persist the dataset.

### Recording a Dataset (full terminal sequence)

Open each command in its own terminal; source the project setup in every new terminal first.

1. Robot + teleop (this already includes `kinova_controllers.launch.py`, so no separate bringup is needed):
   ```bash
   ros2 launch vla_kinova_teleop kinova_pose_tracking_twist.launch.py
   ```
2. Quest bridge (ROS-TCP endpoint + Quest controller):
   ```bash
   ros2 launch vla_kinova_teleop quest_bringup.launch.py
   ```
3. Cameras (RealSense D435 external + Kinova wrist):
   ```bash
   ros2 launch vla_kinova_sensors cameras.launch.py
   ```
4. Recorder:
   ```bash
   ros2 launch vla_kinova_data_collection lerobot_recorder.launch.py
   ```
5. Session driver (kicks off the actual recording loop):
   ```bash
   ros2 run vla_kinova_data_collection record_session --session example
   ```

Finalize: the driver calls `/finalize_dataset` automatically when it exits after keeping episodes, so it blocks until the encoding finishes and the metadata (`info.json` + per-episode stats) is written. Once it prints that the dataset is finalized, the recorder can be safely Ctrl-C'd.

---

## Dataset Workflow

Datasets land on disk in a layout that mirrors Hugging Face's `<owner>/<name>` repo IDs, so the same string is both the local path leaf and the HF repo URL:

```
~/datasets/
└── FilippoGorini/
    ├── kinova_pick_v1/                  # v3 recording, push to FilippoGorini/kinova_pick_v1 (main)
    └── kinova_pick_v1@v2.1/             # v2.1 conversion, push to same repo's v2.1 branch
```

Make sure `dataset_root = "/home/filippo/datasets"` is set in `config/kinova_pi05.toml`, and each session TOML uses `dataset_name = "FilippoGorini/<name>"` (note the `FilippoGorini/` prefix). The recorder writes to `<dataset_root>/<dataset_name>/` automatically.

### One-time setup: Hugging Face tokens

1. Create two access tokens at https://huggingface.co/settings/tokens:
   - **Write** token (`kinova-laptop-write`) for the recording laptop.
   - **Read** token (`kinova-training-server-read`) for the remote training machine.
2. Log in from a terminal with the lerobot_ros venv active:
   ```bash
   source ~/isaac-projects/projects/vla_kinova_tabletop_isaac/ros2_ws/src/lerobot_ros/.venv/bin/activate
   huggingface-cli login         # paste the write token
   huggingface-cli whoami        # should print FilippoGorini
   ```

The token is cached at `~/.cache/huggingface/stored_tokens` and is shared across venvs, so you only need to log in once per machine.

### Converting v3 to v2.1 (for openpi finetuning)

openpi consumes LeRobot **v2.1** datasets while the recorder produces **v3.0**. Conversion is done in lerobot studio:

1. Open lerobot studio and load `~/datasets/FilippoGorini/<name>/`.
2. `Export → v2.1 format`, output path `~/datasets/FilippoGorini/<name>@v2.1/`.
3. Sanity check that the converted dataset still points at the same HF repo:
   ```bash
   python3 -c "import json; d=json.load(open('/home/filippo/datasets/FilippoGorini/<name>@v2.1/meta/info.json')); print('repo_id:', d.get('repo_id')); print('codebase_version:', d.get('codebase_version'))"
   ```
   `repo_id` should equal `FilippoGorini/<name>` (same as the v3 dataset; both are branches of the same HF repo). `codebase_version` should be `v2.1`. If `repo_id` is wrong:
   ```bash
   python3 -c "import json; p='/home/filippo/datasets/FilippoGorini/<name>@v2.1/meta/info.json'; d=json.load(open(p)); d['repo_id']='FilippoGorini/<name>'; json.dump(d, open(p,'w'), indent=2); print('done')"
   ```

### Pushing to Hugging Face (v3 to main, v2.1 to a branch)

After recording (and before / after the v2.1 conversion):

```bash
# 1. Make sure the v3 dataset's repo_id is set correctly on disk
python3 -c "import json; p='/home/filippo/datasets/FilippoGorini/<name>/meta/info.json'; d=json.load(open(p)); d['repo_id']='FilippoGorini/<name>'; json.dump(d, open(p,'w'), indent=2); print('done')"

# 2. Create the HF repo (private; flip to public from the web UI when ready)
hf repo create FilippoGorini/<name> --repo-type dataset --private

# 3. Push v3 to the main branch
hf upload FilippoGorini/<name> ~/datasets/FilippoGorini/<name> --repo-type=dataset

# 4. Create the v2.1 branch on the same repo (no `hf branch` subcommand; use the Python API)
python -c "from huggingface_hub import create_branch; create_branch('FilippoGorini/<name>', branch='v2.1', repo_type='dataset')"

# 5. Push the v2.1 content to that branch
hf upload FilippoGorini/<name> ~/datasets/FilippoGorini/<name>@v2.1 --repo-type=dataset --revision v2.1
```

Verify by visiting `https://huggingface.co/datasets/FilippoGorini/<name>`: the branch selector should list both `main` and `v2.1`. Files differ in format between the two branches.

### Pulling on the training server

With the read-only token logged in on the training machine, load by `revision`:

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset("FilippoGorini/<name>", revision="v2.1")
```

`huggingface_hub` downloads only the v2.1 branch into the local HF cache (`~/.cache/huggingface/datasets/`). For repeated pulls / very large datasets, export `HF_HUB_ENABLE_HF_TRANSFER=1` (the `hf_transfer` Rust uploader is already installed in both venvs as a `lerobot` dep).

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

- **RealSense camera topic is slow/empty in ROS 2, even though `realsense-viewer` shows a smooth 30 fps.**
  Linux's default network buffers are too small for camera-sized messages, so data gets dropped. Fix:
  1. Run `./set_dds_udp_buffers.sh` once after every reboot, before launching the cameras.
  2. Check `.env` has `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` set.

- **Quest teleop is jerky, or the arm freezes briefly then snaps forward.** If WiFi is too unreliable to provide a smooth data stream, 
you can fix it by connecting the Quest with a USB-C cable instead:
  1. Run `./set_quest_adp_connection.sh` (re-run every time you unplug/replug the cable).
  2. In the Quest2ROS app on the headset, set the server IP to `127.0.0.1`.

<!-- Add project-specific troubleshooting notes here. -->

See also the repo-level [Troubleshooting](../../README.md#troubleshooting) section.
