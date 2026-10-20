#!/usr/bin/env python3
"""Lab 3 student model — MATH ONLY; NO ROS or robot commands.

Predict an ideal path before inspecting the supplied ROS controller.
Compare unsigned distance with signed progress after overshoot.

This script does not import ROS, publish /cmd_vel, or operate a physical robot.
Run FROM THIS FOLDER: python3 -B relative_move_math_walkthrough.py
"""
import argparse
import math


def local_to_world(x_local, y_local, yaw_radians):
    """Rotate a robot-frame displacement into a world-frame displacement."""
    return (
        math.cos(yaw_radians) * x_local - math.sin(yaw_radians) * y_local,
        math.sin(yaw_radians) * x_local + math.cos(yaw_radians) * y_local,
    )


def demo_square(side_m=1.0, turn_deg=90.0):
    """Calculate four ideal relative moves for a chosen side and turn."""
    x, y, heading = 0.0, 0.0, 0.0
    print("IDEAL RELATIVE PATH — no sensor noise or motor error")
    print(f"start: position=({x:.3f}, {y:.3f}) m, heading=0 deg")
    print(f"each relative move: forward={side_m:g} m, turn={turn_deg:+g} deg")
    for leg in range(1, 5):
        dx, dy = local_to_world(side_m, 0.0, heading)
        x += dx
        y += dy
        heading += math.radians(turn_deg)
        print(
            f"leg {leg}: position=({x:.3f}, {y:.3f}) m, "
            f"heading={math.degrees(heading):.0f} deg"
        )
    print(f"ideal position closure: {math.hypot(x, y):.6f} m")


def demo_overshoot():
    # Historical *illustrative* values from staff's original-controller audit.
    # Robot has already progressed beyond the intended 1.000 m endpoint.
    planned_length = 1.000
    along_track_progress = 1.013
    sideways_offset = 0.027
    euclidean_distance_to_goal = math.hypot(
        planned_length - along_track_progress, sideways_offset
    )
    signed_distance_remaining = planned_length - along_track_progress
    print("\nWHY THE STOPPING CONDITIONS DIFFER")
    print(f"unsigned distance to goal = {euclidean_distance_to_goal:.3f} m")
    print(f"signed distance along path = {signed_distance_remaining:+.3f} m")
    print("The first can be >0.010 m after overshooting the target.")
    print("The second becomes negative once the planned progress is exceeded.")
    print("Neither measurement alone guarantees collision avoidance or accuracy.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Safe ROS-free local-coordinate exploration")
    parser.add_argument("--side", type=float, default=1.0,
                        help="ideal forward distance in metres (default 1.0)")
    parser.add_argument("--turn", type=float, default=90.0,
                        help="ideal turn angle in degrees (default +90)")
    args = parser.parse_args()
    if not (math.isfinite(args.side) and math.isfinite(args.turn)
            and 0 < args.side <= 5 and -180 <= args.turn <= 180):
        parser.error("--side must be >0 and <=5 m; --turn must be from -180 to 180 degrees")
    demo_square(side_m=args.side, turn_deg=args.turn)
    demo_overshoot()
