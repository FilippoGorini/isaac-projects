#!/usr/bin/env python3
"""Records one image topic to an MP4 on demand, one file per start/stop call.

Frames are piped to a separate low-priority ffmpeg process, which does all the encoding. The
subscription is best effort with depth 1, so the camera driver isn't held back: if this node
falls behind it loses frames. Output is always 30 fps at the original resolution, using arrival
times, so lost frames are filled by repeats and the video keeps its real duration.

Usage (after sourcing the workspace):
    ros2 run vela_data_collection video_recorder --ros-args \
        -p image_topic:=<topic> -p output_dir:=<dir>

    ros2 service call /video_recorder/start lerobot_interfaces/srv/NewDataset \
        "{repo_id: trial_0123/external}"
    ros2 service call /video_recorder/stop std_srvs/srv/Trigger

The name is relative to output_dir and may include subfolders. ".mp4" is added if missing, and an
existing file is never overwritten.
"""

import subprocess
from pathlib import Path

import rclpy
from lerobot_interfaces.srv import NewDataset
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

FPS = 30
PIX_FMTS = {"rgb8": "rgb24", "bgr8": "bgr24"}


class VideoRecorder(Node):
    def __init__(self):
        super().__init__("video_recorder")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("output_dir", ".")
        topic = self.get_parameter("image_topic").get_parameter_value().string_value
        if not topic:
            raise SystemExit("image_topic must be set.")
        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value).expanduser()

        self._path = None       # set while recording
        self._ffmpeg = None     # started on the first frame, which gives the size and encoding
        self._frames = 0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, topic, self._cb_image, qos)
        # ponytail: NewDataset is reused only because it is a string -> success + msg service that
        # is already a dependency. Add a dedicated srv if this node grows.
        self.create_service(NewDataset, "~/start", self._cb_start)
        self.create_service(Trigger, "~/stop", self._cb_stop)
        self.get_logger().info(f"Ready to record '{topic}' into '{self._output_dir}'.")

    def _cb_start(self, request, response):
        if self._path is not None:
            response.success, response.msg = False, f"already recording {self._path}"
            return response
        if not request.repo_id:
            response.success, response.msg = False, "empty name"
            return response
        path = self._output_dir / request.repo_id
        if path.suffix != ".mp4":
            path = path.with_name(path.name + ".mp4")
        if path.exists():
            response.success, response.msg = False, f"{path} already exists"
            return response
        path.parent.mkdir(parents=True, exist_ok=True)
        self._path, self._frames = path, 0
        response.success, response.msg = True, str(path)
        return response

    def _cb_image(self, msg: Image) -> None:
        if self._path is None:
            return
        try:
            if self._ffmpeg is None:
                self._ffmpeg = self._start_ffmpeg(msg)
            self._ffmpeg.stdin.write(msg.data)
            self._frames += 1
        except (OSError, ValueError) as exc:
            # Not recording any more, so the next stop call reports the failure
            self.get_logger().error(f"Recording {self._path} failed: {exc}")
            self._path, self._ffmpeg = None, None

    def _start_ffmpeg(self, msg: Image) -> subprocess.Popen:
        if msg.encoding not in PIX_FMTS or msg.step != msg.width * 3:
            raise ValueError(f"unsupported image: {msg.encoding}, step {msg.step}")
        return subprocess.Popen(
            ["nice", "-n", "19", "ffmpeg", "-loglevel", "error",
             "-use_wallclock_as_timestamps", "1",
             "-f", "rawvideo", "-pixel_format", PIX_FMTS[msg.encoding],
             "-video_size", f"{msg.width}x{msg.height}", "-i", "-",
             "-r", str(FPS), "-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p",
             str(self._path)],
            stdin=subprocess.PIPE,
            start_new_session=True,     # Ctrl-C reaches this node only, which then closes the file
        )

    def _cb_stop(self, request, response):
        if self._path is None:
            response.success, response.message = False, "not recording"
            return response
        path, ffmpeg, frames = self._path, self._ffmpeg, self._frames
        self._path, self._ffmpeg = None, None
        if ffmpeg is None:
            response.success, response.message = False, f"no frames received for {path}"
            return response
        ffmpeg.stdin.close()    # end of input: ffmpeg finishes the file and exits
        code = ffmpeg.wait()
        response.success = code == 0
        response.message = f"{path}: {frames} frames" if code == 0 else f"ffmpeg exit code {code}"
        return response

    def close(self) -> None:
        """Finish the file if the node is stopped while recording."""
        if self._ffmpeg is not None:
            self._ffmpeg.stdin.close()
            self._ffmpeg.wait()
            self._path, self._ffmpeg = None, None


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # Ctrl-C can surface as RCLError ("context is not valid") from the executor
        if rclpy.ok():
            raise
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
