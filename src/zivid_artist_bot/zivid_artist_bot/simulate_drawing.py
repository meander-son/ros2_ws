#!/usr/bin/env python3
import os
import sys

from scipy.spatial.transform import Rotation as R

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from svgpathtools import svg2paths2


class SvgRobotDrawingNode(Node):
    def __init__(self, svg_filepath):
        super().__init__("svg_robot_drawing_node")

        self.svg_filepath = svg_filepath
        self.drawHeight = 0.0
        self.safeHeight = 0.005
        self.offset_x = 0.25 
        self.offset_y = 0.1 # more reliable at these offsets for some reason 
        self.scale_factor = 0.0005

        self.latest_joint_state = None
        self.create_subscription(
            JointState,
            "/lbr/joint_states",
            self._joint_state_callback,
            10,
        )

        self.move_group_client = ActionClient(self, MoveGroup, "/lbr/move_action")
        self.get_logger().info("Connecting to MoveIt Action Server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("MoveGroup connection active.")

        self.optimized_paths = self.parse_and_optimize_svg(self.svg_filepath)

    def _joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def parse_and_optimize_svg(self, filepath):
        self.get_logger().info(f"Loading vector image from file: {filepath}")
        if not os.path.exists(filepath):
            self.get_logger().error(f"SVG file path does not exist: {filepath}")
            sys.exit(1)

        paths, _, _ = svg2paths2(filepath)
        unvisited_paths = list(paths)
        optimized_paths = []

        current_pen_position = 0.0 + 0.0j
        self.get_logger().info(f"Optimizing order for {len(unvisited_paths)} paths...")

        while unvisited_paths:
            best_idx = None
            shortest_distance = float("inf")
            should_flip = False

            for i, path in enumerate(unvisited_paths):
                dist_to_start = abs(current_pen_position - path.start)
                dist_to_end = abs(current_pen_position - path.end)

                if dist_to_start < shortest_distance:
                    shortest_distance = dist_to_start
                    best_idx = i
                    should_flip = False

                if dist_to_end < shortest_distance:
                    shortest_distance = dist_to_end
                    best_idx = i
                    should_flip = True

            chosen_path = unvisited_paths.pop(best_idx)
            if should_flip:
                chosen_path = chosen_path.reversed()

            current_pen_position = chosen_path.end
            optimized_paths.append(chosen_path)

        return optimized_paths

    def _build_move_goal(self, target_x, target_y, target_z):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.pipeline_id = "pilz_industrial_motion_planner"
        goal_msg.request.planner_id = "LIN"
        goal_msg.request.num_planning_attempts = 100
        goal_msg.request.allowed_planning_time = 15.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        if self.latest_joint_state is not None:
            goal_msg.request.start_state.joint_state = self.latest_joint_state
            goal_msg.request.start_state.is_diff = False

        target_pose = PoseStamped()
        target_pose.header.frame_id = "lbr_link_0"
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.position.z = target_z

        r = R.from_euler("xyz", [0.0, 3.14159, 0.0])
        q = r.as_quat()
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        constraints = Constraints()
        constraints.name = "drawing_coordinate"

        pos_c = PositionConstraint()
        pos_c.header = target_pose.header
        pos_c.link_name = "lbr_link_ee"
        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.BOX
        bounding_box.dimensions = [0.005, 0.005, 0.005]
        pos_c.constraint_region.primitives.append(bounding_box)
        pos_c.constraint_region.primitive_poses.append(target_pose.pose)
        pos_c.weight = 1.0
        constraints.position_constraints.append(pos_c)

        ori_c = OrientationConstraint()
        ori_c.header = target_pose.header
        ori_c.link_name = "lbr_link_ee"
        ori_c.orientation.x = q[0]
        ori_c.orientation.y = q[1]
        ori_c.orientation.z = q[2]
        ori_c.orientation.w = q[3]
        ori_c.absolute_x_axis_tolerance = 0.05
        ori_c.absolute_y_axis_tolerance = 0.05
        ori_c.absolute_z_axis_tolerance = 0.05
        ori_c.weight = 1.0
        constraints.orientation_constraints.append(ori_c)

        goal_msg.request.goal_constraints.append(constraints)
        return goal_msg

    def move_linear(self, target_x, target_y, target_z):
        goal_msg = self._build_move_goal(target_x, target_y, target_z)

        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(
                f"Pilz LIN goal rejected for x={target_x:.4f}, y={target_y:.4f}, z={target_z:.4f}"
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        action_result = result_future.result()
        if action_result and action_result.status == 4:
            return True

        self.get_logger().error(
            f"Pilz LIN failed for x={target_x:.4f}, y={target_y:.4f}, z={target_z:.4f}"
        )
        return False

    def execute_drawing_sequence(self):
        self.get_logger().info("Beginning automated straight-line stroke playback sequences...")

        for path_idx, path in enumerate(self.optimized_paths):
            self.get_logger().info(f"Processing Stroke Path #{path_idx}")

            start_vector = path.point(0.0)
            start_x = (start_vector.real * self.scale_factor) + self.offset_x
            start_y = (start_vector.imag * self.scale_factor) + self.offset_y

            self.get_logger().info(f"Hovering over start point of Path #{path_idx}")
            if not self.move_linear(start_x, start_y, self.safeHeight):
                self.get_logger().warn(f"Skipping Path #{path_idx} because approach failed.")
                continue

            self.get_logger().info("Lowering pen tip to drawing board surface...")
            if not self.move_linear(start_x, start_y, self.drawHeight):
                self.get_logger().warn(f"Skipping Path #{path_idx} because lowering failed.")
                continue

            end_vector = path.point(1.0)
            end_x = (end_vector.real * self.scale_factor) + self.offset_x
            end_y = (end_vector.imag * self.scale_factor) + self.offset_y

            self.get_logger().info(f"Drawing linear stroke for Path #{path_idx} using Pilz LIN...")
            if not self.move_linear(end_x, end_y, self.drawHeight):
                self.get_logger().warn(f"Skipping Path #{path_idx} because Pilz LIN failed.")
                continue

            self.get_logger().info("Lifting pen clear from drawing surface.")
            self.move_linear(end_x, end_y, self.safeHeight)

        self.get_logger().info("Finished rendering image sequence completely.")


def main(args=None):
    rclpy.init(args=args)

    svg_path = None
    clean_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    for arg in clean_args:
        if arg.startswith("--svg="):
            svg_path = arg.split("=", 1)[1]

    if not svg_path:
        print("\nCRITICAL ERROR: Please define target path using command execution arguments.")
        print("Usage pattern: ros2 run zivid_artist_bot simulate_drawing --svg=/absolute/path/to/image.svg\n")
        sys.exit(1)

    node = SvgRobotDrawingNode(svg_filepath=svg_path)

    try:
        node.execute_drawing_sequence()
    except KeyboardInterrupt:
        node.get_logger().warn("Drawing sequence halted mid-execution by user input request.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
