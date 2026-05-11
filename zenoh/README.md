# Zenoh ROS 2 Bridge

![Zenoh](https://img.shields.io/badge/Zenoh-1.9.0-0082C8?logo=eclipseide&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-22314E?logo=ros&logoColor=white)
![Protocol](https://img.shields.io/badge/Protocol-TCP%207447-informational)

Use Zenoh when Isaac Sim runs on a lab GPU server and you want ROS 2 topics on
your laptop: `/tf`, `/joint_states`, sensors, commands, and RViz.

> [!IMPORTANT]
> Cloud GPU servers are deleted after the allocated hours. Start Zenoh manually
> for each session, and save code/checkpoints before the server is removed.
> Do not depend on system services or other host-level persistence.

## Connection Model

| Machine | Port Rule | Laptop Command |
|---|---|---|
| [SimplePod](https://simplepod.ai/) RTX 5090 | Open TCP `7447` directly when needed. | `./zenoh/connect_zenoh_bridge.sh <SIMPLEPOD_PUBLIC_IP> 7447` |
| [Vast.ai](https://vast.ai/) RTX 6000 Pro | Expose server TCP `7447`; Vast.ai may assign a different external port. | `./zenoh/connect_zenoh_bridge.sh <VAST_PUBLIC_IP> <EXTERNAL_MAPPED_PORT>` |

WebRTC and Zenoh are independent. WebRTC is for the Isaac Sim viewport.
Zenoh is for ROS 2 topics and only needs TCP.

```
GPU server: Isaac Sim -> ROS 2 -> zenoh router :7447
Laptop:     zenoh client -> local ROS 2 tools / RViz
```

## Install

Run this once on the GPU server and once on your laptop:

```bash
./isaac_vmctl.sh bootstrap zenoh
```

This downloads `zenoh-bridge-ros2dds` v1.9.0 into `zenoh/`. Keep the same
Zenoh version on both sides. The direct helper remains available as
`./zenoh/setup.sh`.

## Start the Server Bridge

On the assigned GPU server:

```bash
cd ~/isaac-projects
./isaac_vmctl.sh bootstrap zenoh
./isaac_vmctl.sh start zenoh 7447
```

When Isaac Sim is already running, `isaac_vmctl.sh start zenoh` starts the
server bridge inside the Isaac Sim container so Zenoh uses the same ROS 2
runtime as Isaac's ROS bridge. This is important on Ubuntu 22.04 hosts, where
the host ROS 2 runtime is Humble but the Isaac Sim 5.1 container runtime is
Jazzy. Set `ZENOH_BRIDGE_CONTEXT=host` only when you intentionally want the
host ROS 2 bridge runtime, or `ZENOH_BRIDGE_CONTEXT=isaac` when the bridge must
fail instead of falling back to the host.

With a ROS domain:

```bash
./isaac_vmctl.sh start zenoh 7447 --domain <id>
```

Stop the server bridge:

```bash
./isaac_vmctl.sh stop zenoh
```

> [!WARNING]
> `ROS_DOMAIN_ID` must match on the GPU server and laptop. Bootstrap persists
> the GPU server default in `/etc/isaac-projects/ros.env`; use `--domain <id>`
> only when you need to override that default for one bridge process.

## Connect From Your Laptop

SimplePod direct port:

```bash
./isaac_vmctl.sh bootstrap zenoh
./zenoh/connect_zenoh_bridge.sh <SIMPLEPOD_PUBLIC_IP> 7447
```

Vast.ai mapped port:

```bash
./zenoh/connect_zenoh_bridge.sh <VAST_PUBLIC_IP> <EXTERNAL_MAPPED_PORT>
```

Then verify:

```bash
source /opt/ros/<distro>/setup.bash
ros2 topic list
ros2 topic hz /tf
```

Useful variants:

```bash
./zenoh/connect_zenoh_bridge.sh <GPU_PUBLIC_IP> 7447 --domain <id>
./zenoh/connect_zenoh_bridge.sh <GPU_PUBLIC_IP> 7447 --namespace /sim
./zenoh/connect_zenoh_bridge.sh <GPU_PUBLIC_IP> 7447 --config zenoh/configs/example_filter.json5
```

## Isaac Sim ROS 2 Bridge

Isaac Sim must publish ROS 2 topics before Zenoh can forward them. Enable the
ROS 2 bridge extension with:

```bash
ISAAC_EXTRA_ARGS="--/app/enableExtensions/0=omni.isaac.ros2_bridge"
```

Or source the Isaac Lab overlay:

```bash
source configs/isaac-sim-5.1.0.env
source configs/isaac-lab.env
./isaac_vmctl.sh start isaacsim
```

On mixed Humble/Jazzy machines, the container-side bridge can still log a
warning about older ROS discovery GIDs if a host Humble `ros2 daemon` is active
in the same `ROS_DOMAIN_ID`. That warning is from the host CLI daemon, not from
Isaac's publishers, and the setup is healthy when the bridge log also shows
routes for Isaac topics such as `/clock`, `/isaac_joint_states`, and camera
topics. Stop the host daemon with `ros2 daemon stop` if you only want to quiet
that warning while using Zenoh from a laptop.

## Topic Filtering

WAN links should not carry every topic. Start with one of the supplied configs:

| Config | Use |
|---|---|
| [isaac_control_only.json5](configs/isaac_control_only.json5) | Clock, TF, joint state, and Isaac command topics only. No camera stream. |
| [isaac_camera_throttled.json5](configs/isaac_camera_throttled.json5) | Control/state topics plus camera image topics capped to `5 Hz`. |
| [example_filter.json5](configs/example_filter.json5) | Commented template for custom allowlists, frequency caps, and namespaces. |

Apply filtering on the GPU server. Restart the bridge when changing configs:

```bash
./isaac_vmctl.sh stop zenoh
./isaac_vmctl.sh start zenoh 7447 --config zenoh/configs/isaac_control_only.json5
```

Use the camera config when a student needs images:

```bash
./isaac_vmctl.sh stop zenoh
./isaac_vmctl.sh start zenoh 7447 --config zenoh/configs/isaac_camera_throttled.json5
```

The laptop can optionally use the same config to restrict local ROS publishers
and subscribers too:

```bash
./zenoh/connect_zenoh_bridge.sh <GPU_PUBLIC_IP> 7447 --config zenoh/configs/isaac_control_only.json5
```

| Topic Type | Practical WAN Limit |
|---|---|
| `/tf`, `/joint_states` | 30-50 Hz |
| Camera images | 5 Hz or compressed transport |
| Point clouds | 2 Hz |
| 2D lidar | 10 Hz |

## Files

| File | Purpose |
|---|---|
| [setup.sh](setup.sh) | Downloads the bridge binary. |
| [start_zenoh_bridge.sh](start_zenoh_bridge.sh) | Starts the GPU-server router on TCP `7447` in the current ROS 2 shell. Prefer `../isaac_vmctl.sh start zenoh` for Isaac Sim sessions so the container runtime is used when available. |
| [connect_zenoh_bridge.sh](connect_zenoh_bridge.sh) | Connects the laptop to the GPU-server router. |
| [configs/](configs) | Ready-to-use and template Zenoh filtering configs. |

## Troubleshooting

| Problem | First Check |
|---|---|
| Cannot connect | Is the server bridge running? Is the SimplePod port open or the Vast.ai external port mapped to server `7447`? |
| `ros2 topic list` is empty | Enable `omni.isaac.ros2_bridge` and check `ROS_DOMAIN_ID`. |
| Topics appear, then stop | Restart both bridges; cloud network mappings can reset. |
| High latency | Use `configs/example_filter.json5` and reduce camera/point-cloud traffic. |

References: [Zenoh](https://zenoh.io),
[zenoh-plugin-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds),
[Isaac Sim ROS 2 bridge](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/index.html).
