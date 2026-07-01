#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient

class SingleJointMover(Node):
    def __init__(self):
        super().__init__('single_joint_mover')
        self._action_client = ActionClient(self, MoveGroup, '/lbr/move_action')
        self.get_logger().info("Joint mover initialized. Waiting for MoveGroup...")
        self._action_client.wait_for_server()
        
        # Test targets for Joint A1 (in radians). 
        # 0.5 rad is roughly 28 degrees; -0.5 rad is roughly -28 degrees.
        self.target_angles = [0.5, -0.5, 0.0]
        
        self.run_joint_loop()

    def run_joint_loop(self):
        for index, angle in enumerate(self.target_angles):
            self.get_logger().info(f"Sending request: Rotate joint lbr_A1 to {angle} rad")
            
            # 1. Initialize the root Goal Message
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = "arm"
            goal_msg.request.num_planning_attempts = 5
            goal_msg.request.allowed_planning_time = 5.0
            
            # 2. Build the specific Joint Constraint
            joint_c = JointConstraint()
            joint_c.joint_name = "lbr_A1"  # Matches the KUKA frame name
            joint_c.position = angle
            joint_c.tolerance_above = 0.01
            joint_c.tolerance_below = 0.01
            joint_c.weight = 1.0
            
            # 3. CRITICAL: Pack the constraint directly into the Goal payload
            goal_constraints = Constraints()
            goal_constraints.name = f"move_a1_step_{index+1}"
            goal_constraints.joint_constraints.append(joint_c)
            
            # This line bridges the local variables into the active ROS network payload
            goal_msg.request.goal_constraints.append(goal_constraints)

            # 4. Send the goal and wait for acceptance
            send_goal_future = self._action_client.send_goal_async(goal_msg)    
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f"Step {index+1} was rejected by the server.")
                continue

            self.get_logger().info(f"Step {index+1} accepted. Planning trajectory...")

            # 5. Wait for the trajectory execution to complete
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            
            action_result = get_result_future.result()
            
            # Status code 4 means successful completion in ROS 2 Actions
            if action_result.status == 4:
                self.get_logger().info(f"--> Success! Joint lbr_A1 reached {angle} rad.")
            else:
                self.get_logger().error(f"--> Failed to reach target. Status: {action_result.status}")

def main(args=None):
    rclpy.init(args=args)
    node = SingleJointMover()
    rclpy.shutdown()

if __name__ == '__main__':
    main()