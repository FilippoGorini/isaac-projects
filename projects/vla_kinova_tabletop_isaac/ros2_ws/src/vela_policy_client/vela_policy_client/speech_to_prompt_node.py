#!/usr/bin/env python3
"""Node which uses faster-whisper (OpenAI) to transcribe user voice prompts and publishes them to a ros2 topic for the client to pick up"""

import re
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String

SAMPLE_RATE = 16000

# Whisper decoder bias toward the dataset vocabulary, this helps when the model is uncertain between similar words and we want to make sure we output the correct one to not trip the VLA
INITIAL_PROMPT = (
    "grey, lunchbox, plate, cup, eraser, highlighter, marker, masking tape, "
    "correction tape, glue stick, screwdriver, pliers, scissors, usb cable, dock, gum box, "
    "tissue, towel, spray bottle, rope, cube, cylinder, block, spill, "
    "rightmost, leftmost")

# Largeky unnecessary as the INITIAL_PROMPT helps a lot in getting the right words, anywyas if whisper gets it wrong we can manually substitute the word, for example launch box -> lunchbox
_SUBSTITUTIONS = [
    (re.compile(r"\bgray\b"), "grey"),
    (re.compile(r"\b(launch ?box|lunch box)\b"), "lunchbox"),
    (re.compile(r"\bpen ?holder\b"), "pen-holder"),
    (re.compile(r"\blight blue\b"), "light-blue"),
]


def preload_cuda_libs():
    import ctypes
    import glob
    import os

    try:
        import nvidia
    except ImportError:
        return
    for base in nvidia.__path__:
        for pattern in ("cublas/lib/libcublas.so.12", "cudnn/lib/libcudnn.so.9"):
            for so in glob.glob(os.path.join(base, pattern)):
                try:
                    ctypes.CDLL(so, mode=ctypes.RTLD_GLOBAL)
                except OSError:
                    pass


def process_prompt(text: str) -> str:
    """Match the training task strings: lowercase, no punctuation and manually fix known words which often gets misheard"""

    text = re.sub(r"[^a-z0-9\s-]", " ", text.lower())   # lowercase and no punctuation
    text = " ".join(text.split())
    
    for pattern, replacement in _SUBSTITUTIONS:
        # manually replace the known words which whisper often gets wrong to match the dataset tasks exact spelling
        text = pattern.sub(replacement, text)
    return text


class SpeechToPromptNode(Node):
    def __init__(self):
        super().__init__("speech_to_prompt")
        self.declare_parameter("model", "small.en")                 # we can also use medium.en and large.en
        self.declare_parameter("vad", False)                        # vad filter helps with filtering out silence and noise segments and leaving only the ones containing voice
        self.declare_parameter("prompt_topic", "/vla_prompt")   
        self._vad = self.get_parameter("vad").get_parameter_value().bool_value
        topic = self.get_parameter("prompt_topic").get_parameter_value().string_value
        model = self.get_parameter("model").get_parameter_value().string_value

        # Latched topic so that we publish on change only. DDS keeps the last sample so that if we start the client after this node it still gets the prompt sent last
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._prompt = ""       
        self._pub = self.create_publisher(String, topic, qos)
        self._set_prompt("")    # latch the initial idle state

        preload_cuda_libs()     # load cuda stuff
        self.get_logger().info(f"Loading {model}...")
        from faster_whisper import WhisperModel
        self._model = WhisperModel(model, device="cuda", compute_type="float16")
        list(self._model.transcribe(np.zeros(SAMPLE_RATE, np.float32), language="en")[0])  # warmup
        self.get_logger().info(f"Ready, publishing prompts on '{topic}'")

    def _set_prompt(self, prompt: str) -> None:
        """Update the desired task and latch it on the topic (published on change only)."""
        self._prompt = prompt
        self._pub.publish(String(data=prompt))

    def record_loop(self):      # loop to record audio when the user wants to publish a new prompt for the model
        import sounddevice as sd

        chunks = []
        recording = False

        def callback(indata, frames, time_info, status):
            if recording:
                chunks.append(indata.copy())

        stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype="float32",
                                callback=callback)
        stream.start()
        print("\nSPACE = start/stop recording | ESC = cancel recording / stop robot | q = quit\n")
        old_term = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                if not select.select([sys.stdin], [], [], 0.05)[0]:
                    continue
                key = sys.stdin.read(1)
                if key == "q":
                    self._set_prompt("")            # stop the robot before quitting
                    time.sleep(0.2)                      
                    break
                if key == "\x1b":                   # if ESC cancel recording, or stop robot when idle
                    if recording:
                        recording = False
                        chunks.clear()
                        print("CANCELLED, press SPACE to record again\n")
                    else:
                        self._set_prompt("")        # send empty string to make the robot go idle (the client only runs inference if it has a valid non-empty prompt, otherwise it stops the arm)
                        print("STOPPED (arm idle), press SPACE to task again\n")
                    continue
                if key != " ":                      # SPACE = record toggle
                    continue
                if not recording:
                    chunks.clear()
                    recording = True
                    print("recording... (SPACE to send, ESC to cancel)")
                    continue
                recording = False
                audio = np.concatenate(chunks, axis=0)[:, 0] if chunks else np.zeros(0, np.float32)
                if len(audio) < SAMPLE_RATE // 4:
                    continue
                t0 = time.monotonic()
                # transcribe audio segment with whisper mdoel
                segments, _ = self._model.transcribe(audio, language="en", beam_size=5, vad_filter=self._vad, initial_prompt=INITIAL_PROMPT)
                raw = " ".join(s.text.strip() for s in segments).strip()    # raw output before processing the prompt
                prompt = process_prompt(raw)                                # process the prompt to remove punctuation, make everything lowercase and substitute manually the words we know are often misheard
                if not prompt:
                    continue
                self._set_prompt(prompt)
                print(f'heard ({time.monotonic() - t0:.2f}s): "{raw}"')
                print(f'prompt: "{prompt}"\n')

        except KeyboardInterrupt:
            pass

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
            stream.stop()
            stream.close()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechToPromptNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.record_loop()
    finally:
        # stop the executor and join before destroying the context, or rclpy aborts
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
