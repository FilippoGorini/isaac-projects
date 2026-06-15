# lerobot_ros — In‑Depth Reference

A working reference for the `ros2_ws/src/lerobot_ros` package: a ROS 2 (Jazzy) bridge that
**records LeRobot datasets from live ROS topics**, replays them, and runs trained LeRobot
policies back onto the robot. Developed for the FHNW Rover project
(upstream: `github.com/sacovo/lerobot_ros`).

Paths below are relative to `ros2_ws/src/lerobot_ros/`.

> Companion doc: `../external/OPENPI_GUIDE.md` explains the openpi/π₀ side. The dataset format
> produced here (LeRobot v2/v3) is the *same* format openpi consumes — so this package is one
> way to generate training data for π₀/π₀.₅ as well as for native LeRobot policies (ACT, etc.).

---

## 1. What the package is and how it's laid out

This directory is itself a **nested colcon workspace** (`src/` inside it) containing four ROS
packages plus configs:

```
lerobot_ros/
  README.md                      # upstream quick-start
  pyproject.toml                 # Python deps (installed via uv into a --system-site-packages venv)
  config/
    so101/so101.toml             # example setup config (topics + policies)
    so101/params.yml             # so101 driver params
    test_recorder.toml
  src/
    lerobot_ros/                 # ** the main Python package (recorder/replay/policy nodes) **
      lerobot_ros/
        recorder.py              # dataset_recorder node — RECORDING entry point
        subscriber.py            # Ros2Feature: subscribes to topics, builds frames at fixed fps
        config.py                # TOML -> ROSFeatureConfig / PolicyConfig; QoS parsing
        convert/                 # ROS msg <-> torch.Tensor converters (the schema layer)
          base.py                #   BaseTopic + registry + layout-topic factory
          std.py                 #   std_msgs (Bool/Int*/Float*/MultiArray) — auto-generated
          sensor.py              #   sensor_msgs (JointState + Imu/Range/NavSatFix/... layouts)
          image.py               #   Image / CompressedImage -> uint8 HxWxC tensor
          geometry.py / fhnw.py  #   geometry_msgs / project-specific msgs
        policy_controller.py     # runs a trained policy, publishes actions (inference)
        replay.py                # replays a recorded dataset's actions onto action topics
        ros_torch_utils.py       # tensor->ROS (action publishing), prepare_frame for inference
        episode_tracker.py / _node.py
      setup.py                   # ros2 entry_points (console_scripts)
    lerobot_interfaces/          # custom srv/msg (NewDataset, StartEpisode, EndEpisode, ...)
    rust_py_timer/               # Rust extension: precise fixed-rate frame collector
    so101/                       # SO-101 arm leader/follower driver nodes
```

ROS entry points (`setup.py`):
| `ros2 run lerobot_ros ...` | node | role |
|---|---|---|
| `dataset_recorder` | `recorder.py:main` | **record a dataset from topics** |
| `policy_controller` | `policy_controller.py:main` | run a trained policy → publish actions |
| `replay` | `replay.py:main` | replay recorded actions to topics |
| `episode_tracker` | `episode_tracker_node.py` | track episode/task progress |
| `so101_leader` / `so101_follower` | so101 driver | SO-101 teleop arm |

---

## 2. The recording pipeline (the core of this package)

### 2.1 Data flow, end to end

```
ROS topics ──subscribe──> Ros2Feature (subscriber.py)
   │   (raw msgs handed to a Rust FrameCollector, or a Python timer fallback)
   ▼
fixed-rate "frame" @ fps: {topic_name: [latest msg(s) since last tick]}
   │   _process_loop: msg -> tensor via each topic's BaseTopic.to_tensor()
   ▼
_convert_frame(): assemble a LeRobot frame dict:
   { "observation.images.<key>": uint8[H,W,C],
     "observation.state":  float32[Σ state dims],   # all non-action, non-image, non-meta topics concatenated
     "action":             float32[Σ action dims],  # all tag="action" topics concatenated
     "meta.<key>":         <tensor> }               # tag="meta" topics, kept separate
   │   frame_callback (= Recorder._timer_callback)
   ▼
Recorder (recorder.py): if recording, buffer (frame, task, t) in RAM per episode
   │   service calls drive the lifecycle (new_dataset / start / end / store)
   ▼
LeRobotDataset.add_frame(frame) + save_episode()  -> on-disk LeRobot dataset under dataset_root
```

Key design point: **recording happens entirely in RAM first.** `start_episode` clears a buffer,
the fixed-rate callback appends `(frame, task, timestamp)` tuples (`recorder.py:81`), `end_episode`
moves the buffer into `self.episodes` (or discards it), and only `store_episodes` writes to disk
in a **background thread** (`store_thread`, `recorder.py:203`) via `dataset.add_frame` +
`dataset.save_episode`. This keeps the high-rate capture loop from blocking on disk I/O.

### 2.2 Frame timing — the Rust collector

`Ros2Feature` (`subscriber.py:36`) prefers a Rust `FrameCollector` (`rust_py_timer`) that holds
the latest message per topic and fires a callback at exactly `fps` Hz, giving precise, drift‑free
frame intervals. If the Rust extension isn't importable it falls back to a pure‑Python timer
thread (`_timer_loop`). Either way, each tick produces the set of messages received per topic
since the last tick; `_convert_frame` keeps **only the most recent** (`tensors[-1]`) of each
(comment says "average out high‑frequency measurements", but the code takes the last sample).
Missing topics for a tick are zero‑filled to their declared shape/dtype (`subscriber.py:307`).

### 2.3 Recording lifecycle (services)

From `recorder.py` (services registered at `:45`). Typical session:

```bash
# Start the recorder with your setup config:
ros2 run lerobot_ros dataset_recorder --ros-args -p config:=config/so101/so101.toml

# 1) Create / open a dataset (resume=true to append to an existing one)
ros2 service call /new_dataset      lerobot_interfaces/srv/NewDataset 'repo_id: "user/ds-name"'
# 2) Begin an episode, giving it the language task string
ros2 service call /start_episode    lerobot_interfaces/srv/StartEpisode 'task: "pick up the ball"'
#    ... perform the task (teleoperate the robot) ...
# 3) End it (or discard if it went wrong)
ros2 service call /end_episode      lerobot_interfaces/srv/EndEpisode               # keep
ros2 service call /end_episode      lerobot_interfaces/srv/EndEpisode 'discard: true' # drop
#    repeat 2–3 for more episodes
# 4) Flush buffered episodes to disk (background)
ros2 service call /store_episodes   std_srvs/srv/Trigger
# 5) Finalize (write metadata) and/or push to HF Hub
ros2 service call /finalize_dataset std_srvs/srv/Trigger
ros2 service call /push_to_hub      std_srvs/srv/Trigger
```

`new_dataset` (`recorder.py:276`) either resumes, opens an existing dataset (and **validates the
feature set matches** the configured topics, erroring on mismatch), or creates a fresh
`LeRobotDataset.create(...)` whose `features` come from `Ros2Feature.get_feature_description()`.
The node also publishes `frame` and `episode` counters (`Int32`) for monitoring.

Custom interfaces (`lerobot_interfaces/`): `NewDataset(repo_id, resume)`,
`StartEpisode(task)→episode_id`, `EndEpisode(discard)→frames`, plus policy‑side
`ListPolicies`, `SetActivePolicy`, `Calibrate`, and the `TaskProgress` msg.

---

## 3. Configuration (`*.toml`) — how you declare what to record

Everything is driven by a single TOML file (parsed in `config.py:parse_config`). Top‑level keys:

| Key | Meaning |
|-----|---------|
| `fps` | dataset frame rate; also the frame‑collector tick rate (default 20) |
| `dataset_root` | where datasets are written (default `./datasets`; so101 example uses `./data`) |
| `tolerance_s` | LeRobot timestamp tolerance (default 0.01) |
| `visualize` / `rerun_remote` | stream live data to a Rerun viewer |
| `robot_type` | string stored for policy inference |
| `[topics]` | the recorded/served topics (see below) |
| `[policies]` | trained policies for `policy_controller` (see §5) |

### 3.1 The `[topics]` section — the heart of the schema

Each `[topics."/ros/topic/name"]` table declares one topic. Common fields:

```toml
[topics."/so101_leader/joint_states"]
msg_type = "JointState"      # selects the converter class (BaseTopic.MAPPINGS key)
tag      = "action"          # "action" | "observation" (default) | "meta"
key      = "leader"          # short name used in the dataset feature key
qos      = { history = "keep_last", depth = 10, reliability = "reliable", durability = "volatile" }
# msg-type-specific fields follow...
joints   = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
position = true              # JointState: which sub-fields to include
velocity = false
effort   = false
```

The **`tag`** field decides which dataset column a topic feeds into
(`subscriber.py:key_for_topic`):
- `tag = "action"` → concatenated into the `action` vector (the policy's *output* target).
- `tag = "observation"` (default) → concatenated into `observation.state` (the policy's *input*),
  unless it's an image.
- image topics (`msg_type="Image"`/`"CompressedImage"`) → their own
  `observation.images.<key>` video feature (never merged into `state`).
- `tag = "meta"` → kept as a separate `meta.<key>` feature: available for training/analysis but
  **excluded from what a policy sees at inference**. (Example: `/human_intervention` Bool.)

`msg_type` is matched against `BaseTopic.MAPPINGS`, a registry every converter subclass
auto‑registers into (`base.py:__init_subclass__`). `key` defaults to the cleaned topic name.

### 3.2 SO‑101 example (`config/so101/so101.toml`)
- `/so101_leader/joint_states` (JointState, `tag=action`) → the **leader** arm a human moves =
  the action to imitate.
- `/so101_follower/joint_states` (JointState, default tag) → the **follower** arm state =
  `observation.state`.
- `/base/image_raw`, `/gripper/image_raw` (Image, resized to 360×640×3) → two camera views.
- `/human_intervention` (Bool, `tag=meta`) → recorded but hidden from policies.

So a recorded SO‑101 frame is: 2 images + 6‑D follower state + 6‑D leader action + 1 meta flag,
with the task string attached per episode.

---

## 4. The converter layer (`convert/`) — ROS message ⇄ tensor & dataset features

This is where ROS message semantics become LeRobot dataset features. Every converter is a
`BaseTopic` subclass exposing three things (`base.py:15`):
- `msg_type()` — the ROS message class (also its registry key).
- `feature_description()` — the LeRobot feature dict: `{dtype, shape, names}`.
- `to_tensor(msg)` / `from_tensor(tensor)` — encode/decode for recording vs. publishing.

`feature_description()` is what builds the dataset schema. `Ros2Feature.get_feature_description`
(`subscriber.py:350`) sums the shapes of all `action` topics into one `action` feature and all
state topics into one `observation.state` feature, concatenating their `names` so each scalar
dimension keeps a human‑readable label (e.g. `so101_follower.joint_states.shoulder_pan.position`).
Empty `action`/`state` features are dropped.

Converter modules:
- **`std.py`** — auto‑generates topic classes for every `std_msgs` type with a `data` field:
  scalars (`Bool`, `Int32`, `Float64`, …) → shape `(1,)`; `*MultiArray` → shape `(len(names),)`
  (you must supply `names` in TOML). Dtype is parsed from the class name; torch has no unsigned
  >8‑bit, so `uint16/32/64` are widened.
- **`sensor.py`** — `JointStateTopic` (configurable `position`/`velocity`/`effort` × `joints`,
  flattened) plus auto‑generated fixed‑layout classes for `Imu`, `Range`, `NavSatFix`,
  `BatteryState`, `MagneticField`, etc. Variable‑length msgs (PointCloud2, LaserScan, Joy) are
  intentionally unsupported.
- **`image.py`** — `ImageTopic` decodes `Image` (handles rgb8/bgr8/rgba8/mono8/mono16 + row
  padding/step), optional rotate, resizes to the configured `height×width` with cv2, returns
  `uint8[H,W,C]`; feature dtype is `"video"`. `ImageCompressedTopic` does the same for
  `CompressedImage` via PIL. `key` becomes `observation.images.<key>`.
- **`base.py`** — `make_layout_topic`/`generate_topic_classes_from_layouts` build converters from
  a list of (possibly nested, dot‑indexed) field paths; `get/set_nested_attr` support paths like
  `orientation.x` or `position_covariance.5`.

To support a **new message type**: add a `BaseTopic` subclass (or a layout entry) in `convert/`,
implement the three methods, and reference its `msg_type().__name__` as `msg_type` in TOML.

---

## 5. Inference & replay (closing the loop)

### 5.1 `policy_controller.py` — run a trained policy
Subscribes to the same observation topics, batches each frame (`prepare_frame`: images →
`float32/255`, `CHW`, add batch dim), and runs a LeRobot policy at `fps`. It maintains an
**action queue** filled by `predict_action_chunk` and blends newly predicted chunks with the
queued ones using a configurable smoothing weight (`action_smoothing_beta`,
`calculate_action_weights`) so overlapping chunks transition smoothly. Actions are decoded back
to ROS messages by `TensorToRosConverter` (`ros_torch_utils.py`), which **slices the flat action
vector back into per‑topic messages** in declared order using each action topic's `size()`, and
published on the original action topic names.

`[policies]` config (`PolicyConfig`, `config.py:22`): `pretrained_name_or_path` (HF repo or local
path), `ds_repo_id` (training dataset — needed to load normalization stats/metadata), `device`,
`policy_config` overrides (e.g. `n_action_steps`), `action_queue_size`, `action_smoothing_beta`.
Controlled by services: `/list_policies`, `/set_active_policy`, `/set_policy_running` (bool),
`/toggle_policy_running`, `/calibrate` (optional test‑time training if the policy supports it).
Progress can auto‑stop a task via the `episode_progress` `TaskProgress` topic.

### 5.2 `replay.py` — sanity‑check recorded actions
Loads a dataset and republishes each frame's recorded `action` to the action topics at the
dataset fps (`ros2 run lerobot_ros replay --ros-args -p repo_id:=... -p episodes:=[2,3] -p
repetitions:=3 -p config:=...`). **Warning:** this drives the real robot — it's how you verify the
action encoding round‑trips correctly through `from_tensor`.

---

## 6. How this relates to the dataset format openpi/LeRobot expects

The recorder writes a standard **LeRobot dataset** (via `lerobot.datasets.LeRobotDataset`), the
exact format described in `../external/OPENPI_GUIDE.md` §2/§5:
- `action` — one vector **per frame** (the leader/commanded values). LeRobot chunks it into the
  policy's action horizon at load time; you store single steps here.
- `observation.state` — proprioception.
- `observation.images.<key>` — camera views (stored as video).
- `task` — the language instruction, attached per episode at store time (`recorder.py:217`).

So to train π₀/π₀.₅ on data captured here you'd point a LeRobot/openpi config at the recorded
`repo_id`; the key remaps in openpi's `*DataConfig` map `observation.images.<key>` / `action` to
the model's expected keys. **Caveat:** openpi expects the action column named `actions` (plural)
while LeRobot/this recorder use `action` (singular) — handled by a `RepackTransform` on the
openpi side (the ALOHA config does exactly this remap). Native LeRobot policies (ACT, diffusion)
consume `action` directly via `lerobot-train`.

---

## 7. Cheat‑sheet / mental model

- **One TOML = one setup.** `[topics]` declares every signal; `tag` routes each into
  `action` / `observation.state` / `observation.images.*` / `meta.*`.
- **`msg_type` picks the converter**; the converter's `feature_description()` defines the dataset
  schema, `to_tensor` records, `from_tensor` publishes (replay/inference).
- **Recording is RAM‑buffered**: `new_dataset → start_episode(task) → [move robot] → end_episode
  → store_episodes (writes to disk in background) → finalize/push`.
- **fps** governs both the dataset rate and the Rust fixed‑rate frame collector; only the latest
  sample per topic per tick is kept, missing topics zero‑filled.
- **Same loop, three modes**: record (recorder), drive from policy (policy_controller),
  replay recorded actions (replay) — all share `Ros2Feature` + the `convert/` layer.
- **Output = a standard LeRobot dataset** → trainable with `lerobot-train` or (with a key remap)
  with openpi/π₀.
