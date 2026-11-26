"""Your known-location ArUco mission for Lab 7."""

from mission.node import MapPose, MissionNode


class AutonomousSearch(MissionNode):
    def navigate_to(self, pose: MapPose) -> None:
        """Send a goal and return only after verifying arrival.

        ``pose`` gives the map x, y and heading in metres and radians. Use
        ``self.action`` and the ROS clock. Bound the action wait by
        ``self.goal_timeout_s`` and request cancellation if an accepted goal
        times out. Raise RuntimeError on failure or TimeoutError on a timeout.

        After action success, use ``self.tf_buffer`` to verify the pose in
        ``map`` for ``self.base_frame``. Allow up to ``self.arrival_timeout_s``
        for TF to settle. Compare both position and wrapped heading against
        the supplied tolerances and raise RuntimeError if arrival cannot be verified.
        """
        # todo: build a stamped map goal and send it with its heading

        # todo: check acceptance and completion, and cancel on timeout

        # todo: compare the measured TF position and heading with the goal
        raise NotImplementedError("Implement navigate_to()")

    def run_mission(self) -> str:
        """Read each room's sign in ``self.visit_order``, then park.

        Use ``self.view_poses`` and ``self.goal_poses``, both keyed by room.
        After each verified view stop, call ``self.marker_reader.read_marker()``
        for the ID, label and source timestamp. Raise RuntimeError on an
        unreadable sign or if the three readings fail to identify exactly one
        room with ``self.target_fruit``.

        Return the selected room key after verified arrival at its parking goal.
        """
        # todo: visit each view pose, verify arrival and record its fresh reading

        # todo: find the unique matching sign after reading all three

        # todo: navigate to the selected room's parking goal and verify arrival
        raise NotImplementedError("Implement run_mission()")
