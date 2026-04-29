# Isaac Sim / Isaac Lab for RICE Lab Thesis Projects

![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-5.1.0%20%7C%206.0-76B900?logo=nvidia&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20%7C%2024.04-E95420?logo=ubuntu&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-22314E?logo=ros&logoColor=white)
![GPU](https://img.shields.io/badge/GPU-RTX%205060%20%7C%205090%20%7C%206000%20Pro-76B900?logo=nvidia&logoColor=white)
![Zenoh](https://img.shields.io/badge/Zenoh-1.9.0-0082C8)

This repository helps RICE lab thesis students start reproducible
**Isaac Sim**, **Isaac Lab**, and **ROS 2** work on the Cloud/lab GPU machines.

The main tool is [isaac_vmctl.sh](isaac_vmctl.sh). It bootstraps Docker,
NVIDIA Container Toolkit, ROS 2, default student tooling such as VS Code
remote support and Jupyter, pulls the Isaac Sim container, mounts this repo
into the container, and starts WebRTC, a native GUI on the current X display
such as TigerVNC, or headless sessions.

**What To Do First**

1. Ask your thesis supervisor which GPU machine to use.
2. Fork this repository into your own GitHub account.
3. Clone your fork on the assigned machine.
4. Copy [projects/template](projects/template/) into your own project folder.
5. Keep your code, scenes, configs, and notes in your fork.
6. Open an [issue](https://github.com/RICE-unige/isaac-projects/issues) and
   inform [Omotoye](https://github.com/Omotoye) if the setup or docs break.

> [!IMPORTANT]
> Cloud GPU servers are deleted at the end of the allocated hours, not stopped.
> Push code and scenes to GitHub, and download checkpoints/logs before time
> runs out. Required setup should be reproducible from your fork, not one-off
> host changes. Contact Omotoye if you need extra time to save progress.

## GPU Access

GPU access is handled through your thesis supervisor. The supervisor coordinates
with [Omotoye Shamsudeen Adekoya](https://github.com/Omotoye) or Prof. Carmine
Recchiuto, who set up the machine and send credentials.

| Contact | Email | Teams |
|---|---|---|
| [Omotoye Shamsudeen Adekoya](https://github.com/Omotoye) | [omotoye.adekoya@edu.unige.it](mailto:omotoye.adekoya@edu.unige.it) | Search `Omotoye Adekoya` |
| Prof. Carmine Recchiuto | [carmine.recchiuto@unige.it](mailto:carmine.recchiuto@unige.it) | Search `Carmine Recchiuto` |

| Machine | Use For |
|---|---|
| Lab workstation, RTX 5060 | Setup, ROS integration, simple single-robot simulation. Not available yet. |
| [SimplePod](https://simplepod.ai/), RTX 5090 | Isaac Sim WebRTC, Zenoh, external ports, VPN, remote interactive work. |
| [Vast.ai](https://vast.ai/), RTX 6000 Pro | Headless training, and headless Isaac Sim/ROS 2 jobs when the environment is already set up. |

> [!NOTE]
> Until the RTX 5060 lab workstation arrives, use the cloud machines for all
> use cases.

## Quick Start

Fork `https://github.com/RICE-unige/isaac-projects`, then clone your fork:

```bash
git clone https://github.com/<your-github-user>/isaac-projects.git ~/isaac-projects
cd ~/isaac-projects
cp .env.example .env
```

Edit `.env` if your supervisor gives you specific values. Then start the path
that matches your machine. `isaac_vmctl.sh` only uses environment variables
that are already sourced in the current shell. Use `.env` for the default
WebRTC or headless path, and use the files under `configs/` when the workflow
below calls for a pinned Isaac Sim version, the Isaac Lab overlay, or the
TigerVNC desktop overlay.

### Fresh Server Bootstrap

```bash
source .env
./isaac_vmctl.sh bootstrap
```

Run this once on each new cloud server. After that, use `start` or `run`
commands without reinstalling host packages. Add `--verbose` only when you
want the full installer output in the terminal. If you are using a pinned
config or TigerVNC, source those files before `bootstrap`.

### Workflow Summary

- SimplePod + TigerVNC desktop: bootstrap the TigerVNC desktop once, connect
  with TigerVNC Viewer, then run `./isaac_vmctl.sh start isaacsim --gui` from
  the terminal inside that desktop.
- SimplePod + WebRTC: run `./isaac_vmctl.sh start isaacsim` on the server and
  connect with the NVIDIA Isaac Sim WebRTC Streaming Client.
- Vast.ai headless: run `./isaac_vmctl.sh start isaacsim --headless` when no
  viewport is needed.
- Isaac Lab headless: run the same `isaaclab.sh` tutorial arguments through
  `./isaac_vmctl.sh run isaaclab '...'` from the repo root.
- Isaac Lab GUI on SimplePod: omit `--headless` and run the Isaac Lab command
  from the terminal inside TigerVNC so the viewport opens there.

### SimplePod with TigerVNC Desktop

Use this when you want a full remote Linux desktop instead of the WebRTC
client. SimplePod must allow inbound TCP `5901`, or the custom port set in
`TIGERVNC_PORT`.

First, bootstrap the desktop on the server:

```bash
source configs/isaac-sim-5.1.0.env
source configs/simplepod-tigervnc.env
./isaac_vmctl.sh bootstrap
./isaac_vmctl.sh check
```

Bootstrap installs TigerVNC and starts an XFCE desktop with GNOME Terminal,
Ubuntu Yaru theming, and a Full HD `1920x1080` display on `:1`. If
`TIGERVNC_PASSWORD` is empty, the helper generates one and saves it on the VM:

```bash
cat ~/.vnc/isaac-projects-vnc-password.txt
```

Open TigerVNC Viewer on your laptop and connect to:

```text
<SIMPLEPOD_PUBLIC_IP>:5901
```

If `ufw` is active and `ALLOWED_CLIENT_IP` is set, bootstrap restricts the VNC
port to that IP. This does not configure SimplePod provider-side port rules;
open TCP `5901` in SimplePod before connecting.

Then, from the terminal inside that VNC desktop, start Isaac Sim in GUI mode:

```bash
cd ~/isaac-projects
source configs/isaac-sim-5.1.0.env
source configs/simplepod-tigervnc.env
./isaac_vmctl.sh start isaacsim --gui
./isaac_vmctl.sh check
```

This starts Isaac Sim on the current X display from that VNC terminal. Use the
TigerVNC window as the UI; WebRTC is not part of this workflow.

Run non-headless Isaac Lab commands from that same VNC terminal. If
`run isaaclab` omits `--headless`, the viewport opens on that display.

For standalone Isaac Sim `python.sh` examples, use:

```bash
./isaac_vmctl.sh run --py --gui \
  'standalone_examples/api/isaacsim.robot.policy.examples/anymal_standalone.py'
```

The WebRTC-based workflows below use NVIDIA's WebRTC client instead of
TigerVNC.

### WebRTC Client on Your Laptop

You need the NVIDIA Isaac Sim WebRTC Streaming Client for the SimplePod WebRTC
workflow.

Download the client from NVIDIA's official
[Isaac Sim 5.1.0 Latest Release page](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)
under **Isaac Sim WebRTC Streaming Client**. NVIDIA's livestream client guide
is here:
[Livestream Clients](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html).

On Linux, make the AppImage executable:

```bash
chmod +x ~/Downloads/isaacsim-webrtc-streaming-client-*.AppImage
~/Downloads/isaacsim-webrtc-streaming-client-*.AppImage
```

If it complains about FUSE, install the FUSE 2 runtime first:

```bash
sudo add-apt-repository universe
sudo apt update
sudo apt install libfuse2 || sudo apt install libfuse2t64
```

Then launch it again:

```bash
~/Downloads/isaacsim-webrtc-streaming-client-*.AppImage --no-sandbox
```

When the client opens, enter the server IP and click **Connect**.

### SimplePod with WebRTC

```bash
source .env
./isaac_vmctl.sh start isaacsim
./isaac_vmctl.sh check
```

Open the Isaac Sim WebRTC client and connect to the public IP printed by
`./isaac_vmctl.sh check`.

### Vast.ai Headless Training or Simulation

```bash
source .env
./isaac_vmctl.sh start isaacsim --headless
```

Use Vast.ai when no viewport is needed. Zenoh can still be used for headless
ROS 2 work, but Vast.ai may map server port `7447` to a different external
port. Use the external mapped port from Vast.ai when connecting from your
laptop.

For training commands, use a one-shot container so logs and exit codes stay in
your terminal:

```bash
./isaac_vmctl.sh run -- bash -lc 'cd projects/my-project && python train.py'
```

### Isaac Sim python.sh Commands

Use `run --py` when you want to translate a normal Isaac Sim `./python.sh ...`
command into this repo's Docker workflow.

This:

```bash
./python.sh standalone_examples/api/isaacsim.robot.policy.examples/anymal_standalone.py
```

becomes:

```bash
./isaac_vmctl.sh run --py \
  'standalone_examples/api/isaacsim.robot.policy.examples/anymal_standalone.py'
```

To open the app inside TigerVNC, run the same command from the TigerVNC
terminal and add `--gui`:

```bash
./isaac_vmctl.sh run --py --gui \
  'standalone_examples/api/isaacsim.robot.policy.examples/anymal_standalone.py'
```

To launch the same kind of standalone example through WebRTC, use
`--livestream`:

```bash
./isaac_vmctl.sh run --py --livestream public \
  'standalone_examples/api/isaacsim.robot.policy.examples/anymal_standalone.py'
```

`run --py` wraps `/isaac-sim/python.sh` directly. `--gui` adds X/VNC wiring.
`--livestream` wraps a standalone script entrypoint with Isaac Sim's streaming
configuration, so it is meant for normal script-path examples such as
`standalone_examples/.../*.py`, not Python options like `-c` or `-m`.

For Isaac Sim `5.1.x`, this standalone Python streaming path uses NVIDIA's
default WebRTC ports `49100/tcp` and `47998/udp`.

### Isaac Lab Workflow

`run isaaclab` keeps the same arguments you would pass to `./isaaclab.sh`.
Only the prefix changes:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Ant-v0 --headless
```

becomes:

```bash
./isaac_vmctl.sh run isaaclab '-p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Ant-v0 --headless'
```

When `configs/isaac-lab.env` is sourced, bootstrap manages a pinned Isaac Lab
checkout and image for you.

| Location | Path |
|---|---|
| Host VM | `~/isaac-projects/external/IsaacLab` |
| Inside the container | `/workspace/isaac-projects/external/IsaacLab` |
| Config knob | `ISAACLAB_PATH=external/IsaacLab` |

The whole repo is mounted into the container:

| Location | Path |
|---|---|
| Host repo | `~/isaac-projects` |
| Container repo | `/workspace/isaac-projects` |

Bootstrap uses `ISAACLAB_REF`, `ISAACLAB_FRAMEWORKS`, and `ISAACLAB_PATH`
from `configs/isaac-lab.env`. After bootstrap:

```text
Host checkout:   ~/isaac-projects/external/IsaacLab
Container path:  /workspace/isaac-projects/external/IsaacLab
Managed image:   rice/isaac-lab:<isaac-sim-tag>-<isaaclab-ref>-<frameworks>
```

Override the checkout location persistently:

```bash
export ISAACLAB_PATH=projects/my-project/vendor/IsaacLab
```

Or for a one-off run:

```bash
./isaac_vmctl.sh run isaaclab --path projects/my-project/vendor/IsaacLab '--help'
```

#### First-Time Setup

```bash
cd ~/isaac-projects
source configs/isaac-sim-5.1.0.env
source configs/isaac-lab.env
./isaac_vmctl.sh bootstrap
```

This clones or updates the pinned Isaac Lab checkout and builds the managed
Isaac Lab image. If you change `ISAACLAB_REF`, `ISAACLAB_FRAMEWORKS`, or
`ISAACLAB_PATH`, rerun `./isaac_vmctl.sh bootstrap`.

#### Headless Isaac Lab

Use this for normal training:

```bash
cd ~/isaac-projects
source configs/isaac-sim-5.1.0.env
source configs/isaac-lab.env
./isaac_vmctl.sh run isaaclab \
  '-p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-Anymal-C-v0 --headless'
```

Another common example:

```bash
./isaac_vmctl.sh run isaaclab \
  '-p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Ant-v0 --headless'
```

#### Isaac Lab GUI Through TigerVNC

When you want the viewport, open TigerVNC first and run the same command
without `--headless` from the terminal inside that desktop:

```bash
cd ~/isaac-projects
source configs/isaac-sim-5.1.0.env
source configs/simplepod-tigervnc.env
source configs/isaac-lab.env
./isaac_vmctl.sh run isaaclab \
  '-p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-Anymal-C-v0'
```

That command does not use WebRTC. It opens on the current X display, which in
this workflow is the TigerVNC desktop.

> [!IMPORTANT]
> Do not start a separate `./isaac_vmctl.sh start isaacsim` session before
> `run isaaclab`. Isaac Lab should launch its own simulator process. Use
> `--headless` for non-interactive runs, or omit it only from the TigerVNC
> terminal when you want the GUI.

> [!TIP]
> `Ctrl+C` stops the Isaac Lab container started by `run isaaclab`. If you
> launched a GUI run from TigerVNC, the Isaac Sim window opened by that command
> should close as part of the same stop.

Keep Isaac Lab code in your fork or workspace, pin the Isaac Lab tag/commit in
your project README, and put generated outputs under
`projects/<name>/artifacts/`. The managed Isaac Lab image is built through
[containers/isaac-lab.Dockerfile](containers/isaac-lab.Dockerfile) during
bootstrap.

## Project Layout

Start from the template:

```bash
cp -r projects/template projects/my-project
cp projects/my-project/.env.example projects/my-project/.env
```

Use these folders:

| Path | Purpose | Commit? |
|---|---|---|
| `external/IsaacLab/` | Bootstrap-managed pinned Isaac Lab checkout used by `run isaaclab` by default | Usually no; keep it as its own git checkout |
| `projects/<name>/isaacsim/worlds/` | Isaac Sim worlds, USD scenes, robot scenes | Yes, if small |
| `projects/<name>/isaacsim/rl_scenes/` | RL scene configs, tasks, curricula | Yes |
| `projects/<name>/isaacsim/startup_scenes/` | Lab startup scenes copied for your project | Yes |
| `projects/<name>/ros2_ws/src/` | ROS 2 packages, launch files, messages | Yes |
| `projects/<name>/artifacts/` | Checkpoints, logs, videos, generated output | No |

More detail: [projects/README.md](projects/README.md).

Inside the container, the repository is available at:

```text
/workspace/isaac-projects
```

## Saving Progress

For normal interactive work, GPU access is usually allocated for a fixed daily
window such as 8-12 hours. Training jobs can run longer when agreed with the
lab. When the allocation ends, the cloud server is deleted to stop charges.

Use [`scripts/project_snapshot.sh`](scripts/project_snapshot.sh) to save both
your repo state and the heavy project artifacts that do not belong in git.
The helper always creates a local archive under:

```text
projects/<name>/artifacts/snapshots/
```

Copy the template config once per project if you want local defaults for
includes, resume commands, or an rsync target:

```bash
cp projects/my-project/snapshot.env.example projects/my-project/.snapshot.env
```

Save the current project locally before the server is deleted:

```bash
./scripts/project_snapshot.sh save --project my-project
```

Save and also create or update a repo commit, then push with your existing git
auth:

```bash
./scripts/project_snapshot.sh save \
  --project my-project \
  --git-push \
  --resume-command "./isaac_vmctl.sh run -- bash -lc 'cd projects/my-project && python train.py'"
```

Save locally and upload the archive, manifest, and checksum to another SSH host:

```bash
./scripts/project_snapshot.sh save \
  --project my-project \
  --rsync-target user@backup-host:/absolute/path/isaac-snapshots/
```

Restore on a fresh server from a local archive:

```bash
git clone https://github.com/<your-github-user>/isaac-projects.git ~/isaac-projects
cd ~/isaac-projects
./scripts/project_snapshot.sh restore \
  --project my-project \
  --snapshot projects/my-project/artifacts/snapshots/<snapshot-id>.tar.gz
```

Restore directly from an rsync source:

```bash
./scripts/project_snapshot.sh restore \
  --project my-project \
  --snapshot user@backup-host:/absolute/path/isaac-snapshots/<snapshot-id>.tar.gz
```

> [!TIP]
> Keep the `.tar.gz`, `.manifest.json`, and `.sha256` files together. Git push
> uses your existing SSH or HTTPS auth only; the helper does not manage tokens
> or store credentials. Local archive creation is always the primary save path,
> even when push or rsync upload fails.

## Student Tooling

Bootstrap also installs:

- VS Code remote support
- JupyterLab and Notebook

Defaults live in `.env`:

```bash
VSCODE_REMOTE_ENABLE=1
JUPYTER_ENABLE=1
# export STUDENT_EXTRA_TOOLS="tmux htop ncdu nvtop ripgrep git-lfs"
```

Optional extras: `tmux`, `htop`, `btop`, `ncdu`, `nvtop`, `nvitop`,
`ripgrep`, `git-lfs`, `ffmpeg`, `mosh`.

If `code` or `jupyter` is not found immediately after bootstrap, open a new
terminal or run:

```bash
source ~/.bashrc
```

Recommended VS Code workflow:
[Remote - SSH](https://code.visualstudio.com/docs/remote/ssh).

To run Jupyter manually on the server:

```bash
jupyter lab --no-browser --ip 127.0.0.1 --port 8888
```

## Common Commands

| Command | Use |
|---|---|
| `./isaac_vmctl.sh start isaacsim` | Start Isaac Sim with WebRTC |
| `./isaac_vmctl.sh start isaacsim --gui` | Start native Isaac Sim UI on the current X display |
| `./isaac_vmctl.sh start isaacsim --vnc` | Alias for `--gui`; useful from TigerVNC terminal |
| `./isaac_vmctl.sh start isaacsim --headless` | Start Isaac Sim without WebRTC |
| `./isaac_vmctl.sh run --py [--gui\|--livestream public\|private [--public-ip <ip>]] '<python.sh args>'` | Run an Isaac Sim `python.sh` command in a one-shot container |
| `./isaac_vmctl.sh run isaaclab '<args>'` | Run Isaac Lab with the same arguments you would pass to `./isaaclab.sh` |
| `./isaac_vmctl.sh run -- <command>` | Run a one-shot command inside the Isaac Sim image |
| `./isaac_vmctl.sh stop isaacsim` | Stop the container |
| `./isaac_vmctl.sh start tigervnc` | Install/start the TigerVNC desktop |
| `./isaac_vmctl.sh stop tigervnc` | Stop the TigerVNC desktop |
| `./isaac_vmctl.sh restart isaacsim` | Restart the container |
| `./isaac_vmctl.sh status` | Check host, GPU, Docker, ROS 2, container |
| `./isaac_vmctl.sh logs` | Follow Isaac Sim logs |
| `./isaac_vmctl.sh shell` | Open a shell in the running container |
| `./isaac_vmctl.sh check` | Print IP, port checks, client commands |
| `./isaac_vmctl.sh bootstrap` | Install Docker, NVIDIA runtime, ROS 2, default student tools, image; also manages Isaac Lab when `ISAACLAB_ENABLE=1` and starts TigerVNC when `TIGERVNC_ENABLE=1` |
| `./isaac_vmctl.sh bootstrap zenoh` | Download the Zenoh bridge binary under `zenoh/` |
| `./isaac_vmctl.sh start zenoh` | Start the server-side Zenoh ROS 2 bridge on TCP `7447` |

## WebRTC and ROS 2 Topics

Use **WebRTC** when you need the Isaac Sim viewport through NVIDIA's streaming
client. Use **TigerVNC** with `./isaac_vmctl.sh start isaacsim --gui` when you
want the native Isaac Sim UI inside the remote Linux desktop and do not want to
manage WebRTC separately. Run that command from the terminal inside the VNC
desktop so it can reuse the current X display. On SimplePod, make sure the
matching inbound ports are available:

| Port | Purpose |
|---|---|
| TCP `49100` | WebRTC signaling |
| UDP `47998` | WebRTC video stream |
| TCP `5901` | Optional TigerVNC XFCE desktop |

For Isaac Lab, use `./isaac_vmctl.sh run isaaclab '...'`. Keep `--headless`
for non-interactive runs. When you want to see the viewport, omit
`--headless` and launch the command from the terminal inside TigerVNC so the
GUI lands on that desktop instead of on WebRTC.

Use **Zenoh** when you need ROS 2 topics on your laptop:

```bash
# assigned GPU server
./isaac_vmctl.sh bootstrap zenoh
./isaac_vmctl.sh start zenoh

# Laptop
./isaac_vmctl.sh bootstrap zenoh
./zenoh/connect_zenoh_bridge.sh <GPU_PUBLIC_IP>

# Laptop with a Vast.ai mapped port
./zenoh/connect_zenoh_bridge.sh <VAST_PUBLIC_IP> <EXTERNAL_MAPPED_PORT>
```

Full guide: [zenoh/README.md](zenoh/README.md).

## Keeping Your Fork Updated

```bash
git remote add upstream https://github.com/RICE-unige/isaac-projects.git  # once
git fetch upstream
git checkout main
git merge upstream/main
git push origin main
```

## Useful References

| Topic | Link |
|---|---|
| Version configs | [configs/README.md](configs/README.md) |
| Project workspaces | [projects/README.md](projects/README.md) |
| Zenoh ROS 2 bridge | [zenoh/README.md](zenoh/README.md) |
| Isaac Lab | [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/) |
| Isaac Sim docs | [NVIDIA Isaac Sim documentation](https://docs.isaacsim.omniverse.nvidia.com/) |

## Troubleshooting

| Problem | First Check |
|---|---|
| WebRTC cannot connect | Run `./isaac_vmctl.sh check`; confirm SimplePod ports `49100/tcp` and `47998/udp`. |
| Container exits | Run `./isaac_vmctl.sh logs`. |
| No ROS 2 topics | Enable `omni.isaac.ros2_bridge`; check Zenoh and `ROS_DOMAIN_ID`. |
| GPU not visible | Contact your thesis supervisor, Omotoye, or Prof. Carmine Recchiuto. |

For setup or documentation issues, open an
[issue](https://github.com/RICE-unige/isaac-projects/issues) and inform
[Omotoye](https://github.com/Omotoye).
