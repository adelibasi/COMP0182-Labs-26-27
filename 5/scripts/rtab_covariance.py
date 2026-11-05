"""Plot pose covariance: python3 scripts/rtab_covariance.py [/localization_pose]."""

import sys
import time
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/localization_pose"
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("rtab_covariance_viewer")
    history = deque(maxlen=120)
    start = time.monotonic()
    figure, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 6), constrained_layout=True)
    lines = [axis.plot([], [])[0] for axis in axes]
    for axis, label in zip(axes, ("X variance (m²)", "Y variance (m²)", "Yaw variance (rad²)")):
        axis.set_ylabel(label)
        axis.grid(alpha=0.3)
    axes[-1].set_xlabel("Seconds since viewer started")
    figure.suptitle(f"Waiting for {topic}")

    def receive(message):
        covariance = message.pose.covariance
        history.append((time.monotonic() - start, covariance[0], covariance[7], covariance[35]))
        columns = list(zip(*history))
        for index, (axis, line) in enumerate(zip(axes, lines), start=1):
            line.set_data(columns[0], columns[index])
            axis.relim()
            axis.autoscale_view()
            axis.set_ylim(bottom=0, auto=True)
            axis.set_title(f"{columns[index][-1]:.6g}", loc="right")
        figure.suptitle(f"{topic} — last {len(history)} samples")
        figure.canvas.draw_idle()

    subscription = node.create_subscription(
        PoseWithCovarianceStamped, topic, receive, qos_profile_sensor_data
    )
    try:
        plt.show(block=False)
        while rclpy.ok() and plt.fignum_exists(figure.number):
            rclpy.spin_once(node, timeout_sec=0.05)
            plt.pause(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        plt.close(figure)


if __name__ == "__main__":
    main()
