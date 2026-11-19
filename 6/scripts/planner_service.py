"""ROS service used by the Nav2 bridge; edit exercises/planner.py, then restart Nav2."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from lab6_planner_interfaces.srv import PlanPath
from nav_msgs.msg import Path
from path_adapter import poses_to_cells, validate_path
from planner import a_star_impl


def main():
    rclpy.init()
    node = rclpy.create_node("student_planner")

    def plan(request, response):
        try:
            start_cell, goal_cell = poses_to_cells(request.costmap, request.start, request.goal)
            cells = a_star_impl(
                nav2_cost_map=request.costmap,
                start_point=start_cell,
                goal_point=goal_cell,
            )
            validate_path(request.costmap, start_cell, goal_cell, cells)
            meta = request.costmap.metadata
            path = Path()
            path.header.frame_id = request.costmap.header.frame_id
            path.header.stamp = node.get_clock().now().to_msg()
            # use cell centres so the controller follows the grid path
            for index, (column, row) in enumerate(cells):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = meta.origin.position.x + (column + 0.5) * meta.resolution
                pose.pose.position.y = meta.origin.position.y + (row + 0.5) * meta.resolution
                if index + 1 < len(cells):
                    next_column, next_row = cells[index + 1]
                    angle = math.atan2(next_row - row, next_column - column)
                    pose.pose.orientation.z = math.sin(angle / 2)
                    pose.pose.orientation.w = math.cos(angle / 2)
                else:
                    pose.pose.orientation = request.goal.pose.orientation

                path.poses.append(pose)

            path.poses[0].pose.position = request.start.pose.position
            if len(path.poses) == 1:
                # keep both poses when start and goal share one cell
                first = PoseStamped()
                first.header = path.header
                first.pose = request.start.pose
                path.poses.insert(0, first)

            path.poses[-1].pose.position = request.goal.pose.position
            response.path = path
            response.success = True
        except (NotImplementedError, ValueError, TypeError, OverflowError) as error:
            response.error = str(error)
            node.get_logger().error(response.error)

        return response

    node.create_service(PlanPath, "/student_plan", plan)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
