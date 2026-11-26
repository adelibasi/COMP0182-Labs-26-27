"""Decode a robot's ROS H.264 packets to a localhost-only raw image topic."""

import argparse
from contextlib import ExitStack
import os
import re
import sys
import time


class Decoder:
    def __init__(self):
        import av  # rusty-linter: ignore

        self.av = av
        self.codec = None
        self.next_sequence = None
        self.headers = {}

    def decode(self, message):
        if message.encoding.partition(";")[0] != "h264" or message.pts >= 2**63:
            return []

        if message.pts != self.next_sequence:
            self.codec = None
            self.headers.clear()

        self.next_sequence = message.pts + 1
        if self.codec is None:
            if not message.flags & 1:
                return []

            self.codec = self.av.CodecContext.create("h264", "r")
            self.codec.thread_count = 1

        packet = self.av.Packet(bytes(message.data))
        packet.pts = message.pts
        self.headers[message.pts] = message.header
        # bound metadata even when damaged packets yield no frames
        if len(self.headers) > 32:
            del self.headers[next(iter(self.headers))]

        try:
            frames = self.codec.decode(packet)
        except self.av.error.InvalidDataError:
            self.codec = None
            self.headers.clear()

            return []

        decoded = []
        for frame in frames:
            header = self.headers.pop(frame.pts, None)
            if header is not None:
                decoded.append((frame, header))

        return decoded


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", required=True)
    parser.add_argument("--input-topic", default="camera/video_h264")
    parser.add_argument(
        "--input-localhost-only", default="false", choices=("true", "false")
    )
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", args.robot_name):
        parser.error("robot-name must be the robot name without a leading slash")

    # custom transports could add a network interface to the localhost participant
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["SKIP_DEFAULT_XML"] = "1"
    for key in (
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        "FASTDDS_DEFAULT_PROFILES_FILE",
        "FASTDDS_ENVIRONMENT_FILE",
    ):
        os.environ.pop(key, None)

    # load ROS bindings after setting transport defaults
    from cv_bridge import CvBridge  # rusty-linter: ignore
    from ffmpeg_image_transport_msgs.msg import FFMPEGPacket  # rusty-linter: ignore
    import rclpy  # rusty-linter: ignore
    from rclpy.context import Context  # rusty-linter: ignore
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor  # rusty-linter: ignore
    from rclpy.qos import QoSProfile, ReliabilityPolicy  # rusty-linter: ignore
    from rclpy.signals import SignalHandlerOptions, uninstall_signal_handlers  # rusty-linter: ignore
    from sensor_msgs.msg import Image  # rusty-linter: ignore

    decoder, bridge = Decoder(), CvBridge()
    local_context, input_context = Context(), Context()
    namespace = f"/{args.robot_name}"
    with ExitStack() as cleanup:
        # humble captures localhost settings independently at context initialisation
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init(args=[], context=local_context)
        cleanup.callback(local_context.try_shutdown)
        os.environ["ROS_LOCALHOST_ONLY"] = (
            "1" if args.input_localhost_only == "true" else "0"
        )
        rclpy.init(
            args=[],
            context=input_context,
            signal_handler_options=SignalHandlerOptions.ALL,
        )
        cleanup.callback(uninstall_signal_handlers)
        cleanup.callback(input_context.try_shutdown)

        local_node = rclpy.create_node(
            "decoded_camera",
            namespace=namespace,
            context=local_context,
        )
        cleanup.callback(local_node.destroy_node)
        publisher = local_node.create_publisher(Image, "camera/video_raw", 1)
        camera_ready = False

        def receive(message):
            nonlocal camera_ready
            for frame, header in decoder.decode(message):
                image = bridge.cv2_to_imgmsg(
                    frame.to_ndarray(format="bgr24"), encoding="bgr8"
                )
                image.header = header
                publisher.publish(image)
                camera_ready = True

        input_node = rclpy.create_node(
            "camera_h264_receiver",
            namespace=namespace,
            context=input_context,
        )
        cleanup.callback(input_node.destroy_node)
        subscription = input_node.create_subscription(
            FFMPEGPacket,
            args.input_topic,
            receive,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        input_node.get_logger().info(
            f"Waiting up to 60s for {subscription.topic_name}. "
            f"Decoded images will be on {publisher.topic_name} locally."
        )
        executor = SingleThreadedExecutor(context=input_context)
        cleanup.callback(executor.shutdown)
        try:
            deadline = time.monotonic() + 60
            while input_context.ok() and not camera_ready:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    source = (
                        "Launch sim.launch.py with camera_transport:=h264."
                        if args.input_localhost_only == "true"
                        else "Start the robot camera and check its ROS domain."
                    )
                    raise SystemExit(
                        f"FAIL: no decodable H.264 frames on {subscription.topic_name} "
                        f"after 60s. {source}"
                    )

                rclpy.spin_once(input_node, executor=executor, timeout_sec=remaining)

            if input_context.ok():
                rclpy.spin(input_node, executor=executor)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == "__main__":
    try:
        main()
    except ImportError as error:
        sys.exit(
            f"FAIL: camera decoder dependency unavailable ({error}). "
            "Check ROS Humble, python3-av, ros-humble-cv-bridge and "
            "ros-humble-ffmpeg-image-transport-msgs."
        )
