"""Script to automate ros2 service calls to lerobot_ros

Reads a TOML session file (dataset name + list of {prompt, episodes})
and walks through the recording slots with single-keystroke control while teleoperating

Usage (after sourcing the workspace):
    ros2 run vla_kinova_data_collection record_session --session example
    ros2 run vla_kinova_data_collection record_session -s ~/my_session.toml

The `--session` value can be:
    - a bare name (looks for `<name>.toml` in the current directory first,
      then in the package's share/sessions/ dir).
    - a relative or absolute path to a TOML file.

The lerobot_ros recorder must already be running. When the session completes
(or you quit after keeping episodes) this script calls /finalize_dataset, which
consolidates the parquet + writes meta/info before returning. You do NOT need to
Ctrl-C the recorder to persist the dataset.

Keybindings (no Enter required):
    SPACE / Enter   start the current episode
    k               end + keep   (queues store, advances to next slot)
    d               end + discard (re-tries the same episode)
    s               skip the current slot without recording
    q               quit (during recording: discards the in-progress episode too)
"""

from __future__ import annotations

import argparse
import contextlib
import sys
import termios
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from lerobot_interfaces.srv import EndEpisode, NewDataset, StartEpisode
from std_srvs.srv import Trigger

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:
    import toml as _toml  # `pip install toml` or `apt install python3-toml`

    def _load_toml(path: Path) -> dict:
        with open(path) as f:
            return _toml.load(f)
else:
    def _load_toml(path: Path) -> dict:
        with open(path, "rb") as f:
            return tomllib.load(f)


@dataclass
class Slot:
    task_idx: int
    task_total: int
    ep_idx: int
    ep_total: int
    prompt: str

    def header(self) -> str:
        return (
            f"[task {self.task_idx + 1}/{self.task_total}]"
            f"[ep {self.ep_idx + 1}/{self.ep_total}] "
            f'"{self.prompt}"'
        )


@contextlib.contextmanager
def cbreak_stdin():
    """Put stdin in cbreak mode so single keypresses are read without Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class SessionDriver(Node):
    def __init__(self):
        super().__init__("record_session")
        self.new_ds_cli = self.create_client(NewDataset, "/new_dataset")
        self.start_cli = self.create_client(StartEpisode, "/start_episode")
        self.end_cli = self.create_client(EndEpisode, "/end_episode")
        self.store_cli = self.create_client(Trigger, "/store_episodes")
        self.finalize_cli = self.create_client(Trigger, "/finalize_dataset")
        self._cli_table = [
            ("/new_dataset", self.new_ds_cli),
            ("/start_episode", self.start_cli),
            ("/end_episode", self.end_cli),
            ("/store_episodes", self.store_cli),
            ("/finalize_dataset", self.finalize_cli),
        ]

    def wait_for_services(self, timeout_per_service: float = 10.0):
        for name, cli in self._cli_table:
            print(f"Waiting for {name} ...", flush=True)
            if not cli.wait_for_service(timeout_sec=timeout_per_service):
                raise RuntimeError(
                    f"Service {name} not available after {timeout_per_service}s. "
                    "Is the lerobot_ros recorder running?"
                )

    def _call(self, cli, request, timeout_sec: float = 30.0):
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError(f"Service call to {cli.srv_name} timed out")
        return future.result()

    def new_dataset(self, repo_id: str):
        req = NewDataset.Request()
        req.repo_id = repo_id
        res = self._call(self.new_ds_cli, req)
        if not res.success:
            raise RuntimeError(f"/new_dataset failed: {res.msg}")

    def start_episode(self, task: str) -> int:
        req = StartEpisode.Request()
        req.task = task
        res = self._call(self.start_cli, req)
        return int(res.episode_id)

    def end_episode(self, discard: bool) -> int:
        req = EndEpisode.Request()
        req.discard = discard
        res = self._call(self.end_cli, req)
        return int(res.frames)

    def store_episodes(self):
        res = self._call(self.store_cli, Trigger.Request())
        if not res.success:
            raise RuntimeError(f"/store_episodes failed: {res.message}")

    def finalize_dataset(self):
        # Joins pending encode threads + consolidates the parquet/meta, so we give it a long timeout for safety
        res = self._call(self.finalize_cli, Trigger.Request(), timeout_sec=600.0)
        if not res.success:
            raise RuntimeError(f"/finalize_dataset failed: {res.message}")


def build_slots(tasks: list) -> List[Slot]:
    slots: List[Slot] = []
    n_tasks = len(tasks)
    for ti, task in enumerate(tasks):
        prompt = task["prompt"]
        n_eps = int(task["episodes"])
        for ei in range(n_eps):
            slots.append(Slot(ti, n_tasks, ei, n_eps, prompt))
    return slots


def resolve_session(value: str) -> Optional[Path]:
    """Resolve a --session argument into an actual file path.

    The argument may be:
      1. An absolute or relative path to a `.toml` file that exists.
      2. A bare name (with or without `.toml` suffix) found in CWD.
      3. A bare name found in the package's share/sessions/ dir.

    Returns the first hit, or None if nothing matches.
    """
    candidate = Path(value).expanduser()
    if candidate.is_file():
        return candidate.resolve()

    name = value if value.endswith(".toml") else f"{value}.toml"

    cwd_candidate = Path.cwd() / name
    if cwd_candidate.is_file():
        return cwd_candidate.resolve()

    try:
        share_dir = Path(get_package_share_directory("vla_kinova_data_collection"))
    except Exception:
        return None
    share_candidate = share_dir / "sessions" / name
    if share_candidate.is_file():
        return share_candidate.resolve()

    return None


def parse_args(argv: list) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--session", "-s", required=True,
        help="Session TOML: bare name (looked up in CWD, then share/sessions/) "
             "or relative/absolute path to a .toml file.",
    )
    return p.parse_args(argv[1:])


def _ignore_other(_ch: str) -> None:
    pass


def run_session(driver: SessionDriver, dataset_name: str, slots: List[Slot]) -> int:
    """Walk the slot queue. Returns 0 on completion, 1 if quit early."""
    print(f"Creating / opening dataset: {dataset_name}", flush=True)
    driver.new_dataset(dataset_name)

    print()
    print(f"{len(slots)} episode slots queued.")
    print("Keys: SPACE/Enter=start  k=keep  d=discard  s=skip  q=quit")
    print()

    quit_early = False
    stored = 0
    discarded = 0
    skipped = 0

    with cbreak_stdin():
        i = 0
        while i < len(slots):
            slot = slots[i]
            print(f"{slot.header()}  (SPACE start, s skip, q quit) ", flush=True)

            # Idle: wait for start / skip / quit
            while True:
                ch = sys.stdin.read(1)
                if ch in (" ", "\r", "\n"):
                    ep_id = driver.start_episode(slot.prompt)
                    print(f"  recording ep #{ep_id}  (k keep, d discard, q discard+quit)", flush=True)
                    # Active: wait for keep / discard / quit
                    action = None  # one of: "keep", "discard", "quit"
                    while action is None:
                        ch2 = sys.stdin.read(1)
                        if ch2 in ("k", "K"):
                            action = "keep"
                        elif ch2 in ("d", "D"):
                            action = "discard"
                        elif ch2 in ("q", "Q"):
                            action = "quit"
                        else:
                            _ignore_other(ch2)
                    frames = driver.end_episode(discard=(action != "keep"))
                    if action == "keep":
                        driver.store_episodes()
                        stored += 1
                        print(f"  kept {frames} frames (queued for encoding).", flush=True)
                        i += 1
                    elif action == "discard":
                        discarded += 1
                        print(f"  discarded {frames} frames; retrying slot.", flush=True)
                    else:  # quit
                        discarded += 1
                        print(f"  discarded {frames} frames; quitting.", flush=True)
                        quit_early = True
                    break
                elif ch in ("s", "S"):
                    skipped += 1
                    print("  skipped.", flush=True)
                    i += 1
                    break
                elif ch in ("q", "Q"):
                    print("  quitting.", flush=True)
                    quit_early = True
                    break
                else:
                    _ignore_other(ch)
            if quit_early:
                break

    print()
    print(f"Done. kept={stored}  discarded={discarded}  skipped={skipped}")

    if stored > 0:
        print("Finalizing dataset: waiting for encoding + writing parquet/meta...", flush=True)
        driver.finalize_dataset()
        print("Dataset finalized and safe to use. The recorder can now be Ctrl-C'd.", flush=True)
    else:
        print("No episodes kept; nothing to finalize.", flush=True)

    return 1 if quit_early else 0


def main(args=None):
    rclpy.init(args=args)
    cli_args = remove_ros_args(args=sys.argv)
    parsed = parse_args(cli_args)

    session_path = resolve_session(parsed.session)
    if session_path is None:
        print(f"Session file not found: {parsed.session!r}", file=sys.stderr)
        print(
            "Looked in: cwd, then share/vla_kinova_data_collection/sessions/.",
            file=sys.stderr,
        )
        rclpy.shutdown()
        return 2
    print(f"Loading session: {session_path}", flush=True)

    session = _load_toml(session_path)
    dataset_name = session["dataset_name"]
    tasks = session["tasks"]
    if not tasks:
        print("Session has no tasks; nothing to record.", file=sys.stderr)
        rclpy.shutdown()
        return 2
    slots = build_slots(tasks)

    driver = SessionDriver()
    try:
        driver.wait_for_services()
        rc = run_session(driver, dataset_name, slots)
    except KeyboardInterrupt:
        print("\nInterrupted.", flush=True)
        rc = 130
    finally:
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
