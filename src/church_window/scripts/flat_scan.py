#!/usr/bin/env python3
import os
import sys
import time
import cv2
import numpy as np
from stl import mesh
from scipy.spatial.transform import Rotation as R

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint, JointConstraint
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


# NOTE: This node speaks directly to MoveIt 2 Services. I couldn't get teh python API to work.
# Ensure that the /compute_ik,/check_state_validity, and /move_action interfaces are active

class FlatScanNode(Node):
    def __init__(self, stl_filepath):
        super().__init__("flat_scan_node")
        # Get STL file path
        self.stl_filepath = stl_filepath
        
        # --- WORKSPACE POSITION & ORIENTATION CONFIGURATION ---
        # xyz
        self.offset_x = 0.7
        self.offset_y = 0.3
        self.table_height = 0.251
        
        # rpy - roll pitch yaw
        self.part_roll = 3.14159
        self.part_pitch = 0.0
        self.part_yaw = 1.5708

        # Determine the tool yaw to align the sensor footprint with the part's yaw,
        # choosing between self.part_yaw and self.part_yaw - pi to point the camera
        # away from the part center (preventing camera-part collisions).
        cam_angle1 = self.part_yaw - 1.5708
        cam_angle2 = self.part_yaw - 3.14159 - 1.5708
        dot1 = np.cos(cam_angle1) * (-self.offset_x) + np.sin(cam_angle1) * (-self.offset_y)
        dot2 = np.cos(cam_angle2) * (-self.offset_x) + np.sin(cam_angle2) * (-self.offset_y)
        if dot2 > dot1:
            self.tool_yaw = self.part_yaw - 3.14159
        else:
            self.tool_yaw = self.part_yaw

        self.get_logger().info(f"Selected tool yaw for collision avoidance: {self.tool_yaw:.4f} rad (part yaw: {self.part_yaw:.4f} rad)")

        # Load mesh and calculate lowest rotated Z coordinate for automatic Z offset alignment
        your_mesh = mesh.Mesh.from_file(self.stl_filepath)
        all_vertices = your_mesh.vectors.reshape(-1, 3)
        
        # Build 3D rotation matrix from RPY configuration 
        self.rpy_rot = R.from_euler('zyx', [self.part_yaw, self.part_pitch, self.part_roll])
        self.R_matrix = self.rpy_rot.as_matrix()
        
        # Find the Z coordinate of the lowest point when rotated (which must rest on the table)
        rotated_vertices = all_vertices @ self.R_matrix.T
        z_lowest_relative = np.min(rotated_vertices[:, 2])
        self.offset_z = self.table_height - z_lowest_relative
        
        self.get_logger().info(f"Automatic Z alignment: table={self.table_height:.4f}m, lowest_rel={z_lowest_relative:.4f}m -> offset_z={self.offset_z:.4f}m")

        # Define probe dimensions (in meters)
        self.probe_x = 0.020  # 20 mm
        self.probe_y = 0.025  # 25 mm
        self.overlap = 0.002  # 2 mm
        
        # Calculate step size ensuring 2 mm overlap
        self.step_size = min(self.probe_x, self.probe_y) - self.overlap # 0.018 m

        # Define velocity and acceleration scaling factors to avoid violating joint limits
        self.vel_scaling = 0.05
        self.accel_scaling = 0.05

        self.latest_joint_state = None
        self.create_subscription(
            JointState,
            "/joint_states", 
            self._joint_state_callback,
            10,
        )

        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.get_logger().info("Connecting to MoveIt Action Server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("MoveGroup connection active.")

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.ik_client.wait_for_service()
        self.val_client = self.create_client(GetStateValidity, "/check_state_validity")
        self.val_client.wait_for_service()

        # Generate the adaptive concentric contour tracking path
        structured_grid, self.local_surface_z = self.generate_adaptive_contour_raster(
            self.stl_filepath, step_size=self.step_size
        )

        self.get_logger().info(f"Optimizing path sequence with Loop-Aligned Concentric Contour algorithm...")
        self.local_grid = self.optimize_structured_path(structured_grid)
        self.get_logger().info(f"Path optimization complete. Total waypoints: {len(self.local_grid)}")

    def _joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def optimize_structured_path(self, structured_grid):
        if not structured_grid:
            return []
        
        # Calculate world coordinates to find distance from base (0, 0)
        def robot_base_dist_sq(pt):
            cad_x, cad_y, _ = pt
            cad_pt = np.array([cad_x, cad_y, self.local_surface_z])
            world_pt = self.R_matrix @ cad_pt + np.array([self.offset_x, self.offset_y, self.offset_z])
            return world_pt[0]**2 + world_pt[1]**2 + world_pt[2]**2

        def dist_sq(pt1, pt2):
            return (pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2

        unvisited_islands = list(structured_grid)
        current_pos = None
        optimized_waypoints = []
        
        while unvisited_islands:
            # 1. Choose the next closest island
            if current_pos is None:
                # Find the island closest to the robot base
                best_island = None
                best_dist = float('inf')
                for island in unvisited_islands:
                    if island and island[0]:
                        for pt in island[0]:
                            d = robot_base_dist_sq(pt)
                            if d < best_dist:
                                best_dist = d
                                best_island = island
            else:
                # Find the island closest to current_pos
                best_island = None
                best_dist = float('inf')
                for island in unvisited_islands:
                    if island and island[0]:
                        for pt in island[0]:
                            d = dist_sq(current_pos, pt)
                            if d < best_dist:
                                best_dist = d
                                best_island = island
            
            if best_island is None:
                break
                
            unvisited_islands.remove(best_island)
            
            # 2. Trace nested contours of this island from outer to inner.
            # Align the start of each contour loop with the end of the previous one.
            for contour in best_island:
                if not contour:
                    continue
                
                if current_pos is None:
                    start_idx = min(range(len(contour)), key=lambda idx: robot_base_dist_sq(contour[idx]))
                else:
                    start_idx = min(range(len(contour)), key=lambda idx: dist_sq(current_pos, contour[idx]))
                
                # Shift the loop to start at the closest point
                shifted_contour = contour[start_idx:] + contour[:start_idx]
                optimized_waypoints.extend(shifted_contour)
                current_pos = shifted_contour[-1]
                
        return optimized_waypoints

    def generate_adaptive_contour_raster(self, stl_path, step_size=0.01):
        self.get_logger().info(f"Processing CAD file (Bottom Surface Mode): {stl_path}")
        if not os.path.exists(stl_path):
            self.get_logger().error(f"STL file path does not exist: {stl_path}")
            sys.exit(1)

        your_mesh = mesh.Mesh.from_file(stl_path)
        all_vertices = your_mesh.vectors.reshape(-1, 3)
        min_z = np.min(all_vertices[:, 2])
        max_z = np.max(all_vertices[:, 2])
        
        # Extract facets on the bottom surface of the STL (Z = min_z)
        bottom_surface_mask = np.any(np.abs(your_mesh.vectors[:, :, 2] - min_z) < 0.0005, axis=1)
        bottom_facets = your_mesh.vectors[bottom_surface_mask]
        
        if len(bottom_facets) == 0:
            self.get_logger().error("Could not find a flat bottom plane on STL.")
            sys.exit(1)

        bottom_vertices = bottom_facets.reshape(-1, 3)
        x_min, x_max = np.min(bottom_vertices[:, 0]), np.max(bottom_vertices[:, 0])
        y_min, y_max = np.min(bottom_vertices[:, 1]), np.max(bottom_vertices[:, 1])

        scale = 1000.0  
        img_w = int(np.ceil((x_max - x_min) * scale)) + 20
        img_h = int(np.ceil((y_max - y_min) * scale)) + 20
        mask = np.zeros((img_h, img_w), dtype=np.uint8)

        for triangle in bottom_facets:
            pts = np.array([
                [(triangle[0, 0] - x_min) * scale, (triangle[0, 1] - y_min) * scale],
                [(triangle[1, 0] - x_min) * scale, (triangle[1, 1] - y_min) * scale],
                [(triangle[2, 0] - x_min) * scale, (triangle[2, 1] - y_min) * scale]
            ], dtype=np.int32)
            cv2.fillPoly(mask, [pts], 255)

        num_labels, labels_im = cv2.connectedComponents(mask)
        
        structured_grid = []
        step_pixels = int(step_size * scale)
        if step_pixels < 1:
            step_pixels = 1

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (step_pixels * 2 + 1, step_pixels * 2 + 1))

        for label_id in range(1, num_labels):
            island_mask = np.zeros_like(mask)
            island_mask[labels_im == label_id] = 255
            current_layer = island_mask.copy()
            
            island_contours = []
            
            while cv2.countNonZero(current_layer) > 0:
                contours, _ = cv2.findContours(current_layer, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                for contour in contours:
                    subsampled_contour = contour[::step_pixels]
                    contour_pts = []
                    for pt in subsampled_contour:
                        px_x, px_y = pt[0][0], pt[0][1]
                        cad_x = (px_x / scale) + x_min
                        cad_y = (px_y / scale) + y_min
                        contour_pts.append((cad_x, cad_y, label_id))
                    if contour_pts:
                        island_contours.append(contour_pts)
                
                current_layer = cv2.erode(current_layer, kernel)
            if island_contours:
                structured_grid.append(island_contours)

        return structured_grid, min_z

    def _build_move_goal(self, target_x, target_y, target_z, current_yaw=0.0, planner_id="LIN"):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "manipulator" 
        goal_msg.request.pipeline_id = "pilz_industrial_motion_planner"
        goal_msg.request.planner_id = planner_id
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.allowed_planning_time = 0.5
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2

        if self.latest_joint_state is not None:
            goal_msg.request.start_state.joint_state = self.latest_joint_state
            goal_msg.request.start_state.is_diff = False

        target_pose = PoseStamped()
        target_pose.header.frame_id = "lbr_link_0" 
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.position.z = target_z

        r = R.from_euler("xyz", [0.0, 3.14159, current_yaw])
        q = r.as_quat()
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        constraints = Constraints()
        constraints.name = "scan_coordinate"

        pos_c = PositionConstraint()
        pos_c.header = target_pose.header
        pos_c.link_name = "sensor_tip" 
        
        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.BOX
        bounding_box.dimensions = [0.005, 0.005, 0.005]
        pos_c.constraint_region.primitives.append(bounding_box)
        pos_c.constraint_region.primitive_poses.append(target_pose.pose)
        pos_c.weight = 1.0
        constraints.position_constraints.append(pos_c)

        ori_c = OrientationConstraint()
        ori_c.header = target_pose.header
        ori_c.link_name = "sensor_tip"
        ori_c.orientation = target_pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = 0.05
        ori_c.absolute_y_axis_tolerance = 0.05
        if planner_id == "PTP":
            ori_c.absolute_z_axis_tolerance = 0.05
        else:
            ori_c.absolute_z_axis_tolerance = 6.28  
        ori_c.weight = 1.0
        constraints.orientation_constraints.append(ori_c)

        goal_msg.request.goal_constraints.append(constraints)
        return goal_msg

    def solve_ik(self, x, y, z, yaw, check_discontinuity=True):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "manipulator"
        req.ik_request.ik_link_name = "sensor_tip"
        req.ik_request.avoid_collisions = False
        
        if self.latest_joint_state is not None:
            req.ik_request.robot_state.joint_state = self.latest_joint_state
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "lbr_link_0"
        pose_stamped.pose.position.x = float(x)
        pose_stamped.pose.position.y = float(y)
        pose_stamped.pose.position.z = float(z)
        
        r = R.from_euler("xyz", [0.0, 3.14159, yaw])
        q = r.as_quat()
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]
        
        req.ik_request.pose_stamped = pose_stamped
        
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res and res.error_code.val == 1:
            val_req = GetStateValidity.Request()
            val_req.robot_state = res.solution
            val_req.group_name = "manipulator"
            
            val_future = self.val_client.call_async(val_req)
            rclpy.spin_until_future_complete(self, val_future)
            val_res = val_future.result()
            
            if val_res and val_res.valid:
                if check_discontinuity and self.latest_joint_state is not None:
                    curr_joints = {name: pos for name, pos in zip(self.latest_joint_state.name, self.latest_joint_state.position)}
                    sol_joints = {name: pos for name, pos in zip(res.solution.joint_state.name, res.solution.joint_state.position)}
                    
                    discontinuous = False
                    for name in sol_joints:
                        if name in curr_joints and "lbr" in name:
                            diff = abs(sol_joints[name] - curr_joints[name])
                            if diff > 1.0: # 1.0 radian discontinuity limit
                                discontinuous = True
                                break
                    if discontinuous:
                        self.get_logger().warn("IK solution rejected due to joint configuration jump/discontinuity.")
                        return None
                return res.solution.joint_state
        return None

    def _build_joint_move_goal(self, joint_state, planner_id="LIN"):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "manipulator" 
        goal_msg.request.pipeline_id = "pilz_industrial_motion_planner"
        goal_msg.request.planner_id = planner_id
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.allowed_planning_time = 0.5
        goal_msg.request.max_velocity_scaling_factor = self.vel_scaling
        goal_msg.request.max_acceleration_scaling_factor = self.accel_scaling

        if self.latest_joint_state is not None:
            goal_msg.request.start_state.joint_state = self.latest_joint_state
            goal_msg.request.start_state.is_diff = False

        constraints = Constraints()
        constraints.name = "scan_joint_goal"

        for name, position in zip(joint_state.name, joint_state.position):
            if "lbr" in name:
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = position
                jc.tolerance_above = 0.001
                jc.tolerance_below = 0.001
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        return goal_msg

    def move_linear_joint(self, joint_state):
        goal_msg = self._build_joint_move_goal(joint_state, planner_id="LIN")
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        action_result = result_future.result()
        if action_result and action_result.status == 4:
            return True
        return False

    def move_ptp_joint(self, joint_state):
        goal_msg = self._build_joint_move_goal(joint_state, planner_id="PTP")
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        action_result = result_future.result()
        if action_result and action_result.status == 4:
            return True
        return False

    def execute_scan_sequence(self):
        self.get_logger().info("====================================================")
        self.get_logger().info("Beginning Oriented Adaptive Contour Scan Pipeline...")
        self.get_logger().info(f"Part 3D Orientation (RPY): Roll={self.part_roll:.4f}, Pitch={self.part_pitch:.4f}, Yaw={self.part_yaw:.4f} rad")
        self.get_logger().info(f"Selected Tool Yaw: {self.tool_yaw:.4f} rad")
        self.get_logger().info("====================================================")

        prev_x, prev_y = None, None

        for i, (cad_x, cad_y, island_id) in enumerate(self.local_grid):
            
            # Apply full 3D rotation and translation
            cad_pt = np.array([cad_x, cad_y, self.local_surface_z])
            world_pt = self.R_matrix @ cad_pt + np.array([self.offset_x, self.offset_y, self.offset_z])

            target_world_x = world_pt[0]
            target_world_y = world_pt[1]
            scan_height = world_pt[2]
            
            # Safe height is directly above the target coordinate in world space
            safe_height = scan_height + 0.03

            # Geometric feasibility pre-check (workspace spherical shell and inner cylinder check)
            dist_from_base = np.sqrt(target_world_x**2 + target_world_y**2 + scan_height**2)
            dist_2d = np.sqrt(target_world_x**2 + target_world_y**2)
            if dist_from_base > 0.80 or dist_2d < 0.18:
                self.get_logger().warn(f"Scan target {i+1}/{len(self.local_grid)} at x={target_world_x:.4f}, y={target_world_y:.4f} is geometrically unreachable. Skipping.")
                continue

            # Skip jumps exceeding 5cm
            if prev_x is not None:
                jump_dist = np.sqrt((target_world_x - prev_x)**2 + (target_world_y - prev_y)**2)
                if jump_dist > 0.05:
                    self.get_logger().warn(f"Skipping large world jump of {jump_dist:.4f}m.")
                    prev_x, prev_y = target_world_x, target_world_y
                    continue

            # Solve IK for safe height and scan height to ensure feasibility and bypass MoveIt solver bugs
            safe_joint_state = self.solve_ik(target_world_x, target_world_y, safe_height, self.tool_yaw, check_discontinuity=False)
            scan_joint_state = self.solve_ik(target_world_x, target_world_y, scan_height, self.tool_yaw, check_discontinuity=(prev_x is not None))

            if scan_joint_state is None:
                self.get_logger().warn(f"Scan target {i+1}/{len(self.local_grid)} at x={target_world_x:.4f}, y={target_world_y:.4f} has no valid IK solution. Skipping.")
                continue

            # Consolidated Logging Output
            self.get_logger().info(f"--- TRACKING ELEMENT {i+1}/{len(self.local_grid)} ---")
            self.get_logger().info(f"Local CAD: X={cad_x:.4f}, Y={cad_y:.4f} (Island {island_id}) -> Executing World Target: x={target_world_x:.4f}, y={target_world_y:.4f}, z={scan_height:.4f}")

            if prev_x is None:
                if safe_joint_state is None:
                    self.get_logger().warn("Safe height approach has no valid IK solution. Skipping.")
                    continue
                # Initial approach: PTP to safe height, then lower linearly
                self.move_ptp_joint(safe_joint_state)
                success = self.move_linear_joint(scan_joint_state)
            else:
                # Direct linear move at scan height (no lifting over gaps)
                success = self.move_linear_joint(scan_joint_state)

            if not success:
                self.get_logger().warn(f"Scan target {i+1}/{len(self.local_grid)} at x={target_world_x:.4f}, y={target_world_y:.4f} failed execution. Skipping point.")
                continue

            prev_x, prev_y = target_world_x, target_world_y
            time.sleep(0.02)

        self.get_logger().info("--- Automated Oriented Scan Execution Completed Successfully ---")

def main(args=None):
    rclpy.init(args=args)


    script_dir = os.path.dirname(os.path.abspath(__file__))
    cad_filename = os.path.join(script_dir, "..", "meshes", "CHURCH_WINDOW_NDT_TEST.stl")
    cad_filename = os.path.normpath(cad_filename)
    node = FlatScanNode(stl_filepath=cad_filename)

    try:
        node.execute_scan_sequence()
    except KeyboardInterrupt:
        node.get_logger().warn("Scan sequence halted mid-execution by user break request.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()