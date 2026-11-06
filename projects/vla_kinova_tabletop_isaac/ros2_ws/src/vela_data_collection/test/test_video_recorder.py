"""End-to-end check of video_recorder with fake frames. Needs ffmpeg and a sourced workspace."""

import json
import shutil
import subprocess
import time

import pytest
import rclpy
from lerobot_interfaces.srv import NewDataset
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from vela_data_collection.video_recorder import VideoRecorder

W, H = 640, 480


def call(executor, client, request):
    assert client.wait_for_service(timeout_sec=5.0)
    future = client.call_async(request)
    executor.spin_until_future_complete(future, timeout_sec=30.0)
    return future.result()


def publish(executor, pub, seconds, hz):
    msg = Image(width=W, height=H, encoding="rgb8", step=W * 3, data=bytes(W * H * 3))
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        pub.publish(msg)
        next_frame = time.monotonic() + 1.0 / hz
        while time.monotonic() < next_frame:
            executor.spin_once(timeout_sec=0.001)


def probe(path):
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
         "stream=width,height,avg_frame_rate:format=duration", "-of", "json", str(path)],
        check=True, capture_output=True, text=True).stdout
    data = json.loads(out)
    return data["streams"][0], float(data["format"]["duration"])


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="needs ffmpeg")
def test_records_on_demand(tmp_path):
    rclpy.init(args=["--ros-args", "-p", "image_topic:=/test_video/image",
                     "-p", f"output_dir:={tmp_path}"])
    node = VideoRecorder()
    helper = rclpy.create_node("test_video_helper")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    pub = helper.create_publisher(Image, "/test_video/image", 10)
    start = helper.create_client(NewDataset, "/video_recorder/start")
    stop = helper.create_client(Trigger, "/video_recorder/stop")
    try:
        assert not call(executor, stop, Trigger.Request()).success      # not recording yet
        assert not call(executor, start, NewDataset.Request(repo_id="")).success

        # No frames yet: stop reports it and leaves no file behind
        assert call(executor, start, NewDataset.Request(repo_id="empty")).success
        assert not call(executor, stop, Trigger.Request()).success
        assert not (tmp_path / "empty.mp4").exists()

        # Frames before start are ignored. At 15 Hz, the video still lasts as long as the trial
        publish(executor, pub, 1.0, 30)
        assert call(executor, start, NewDataset.Request(repo_id="trial_1/external")).success
        assert not call(executor, start, NewDataset.Request(repo_id="other")).success
        publish(executor, pub, 3.0, 15)
        assert call(executor, stop, Trigger.Request()).success

        path = tmp_path / "trial_1" / "external.mp4"
        stream, duration = probe(path)
        assert (stream["width"], stream["height"]) == (W, H)
        assert stream["avg_frame_rate"] == "30/1"
        assert duration == pytest.approx(3.0, abs=0.5)

        # An existing file is never overwritten
        assert not call(executor, start, NewDataset.Request(repo_id="trial_1/external")).success
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        helper.destroy_node()
        rclpy.shutdown()
