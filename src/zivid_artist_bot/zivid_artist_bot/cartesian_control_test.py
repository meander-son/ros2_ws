import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint

from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

from scipy.spatial.transform import Rotation as R
class CartesianController(Node):
    def __init__(self):
        super().__init__('cartesian_controller')
        self._action_client = ActionClient(self, MoveGroup, '/lbr/move_action')
        self.get_logger().info("Cartesian controller initialized. Waiting for MoveGroup...")
        self._action_client.wait_for_server()
        
        # Target position [x, y, z]
        self.target_position = [0.5, 0.0, 0.5]

    def run_cartesian_loop(self):
        self.get_logger().info(f"Sending request: Move to position {self.target_position} m")
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        
        constraints = Constraints()
        constraints.name = "move_to_target_pose"
        
        # Define a target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "lbr_link_0" 
        target_pose.pose.position.x = self.target_position[0]
        target_pose.pose.position.y = self.target_position[1]
        target_pose.pose.position.z = self.target_position[2]

        # Define your exact target Roll, Pitch, Yaw (in radians)
        # Note: Avoid using raw values like -10 unless you specifically want multiple rotations.
        target_roll = 0.0    # 0 degrees
        target_pitch = 0.0  # 90 degrees (Tilted forward from its default 'up' state)
        target_yaw = 0.0     # 0 degrees
        
        r = R.from_euler('xyz', [target_roll, target_pitch, target_yaw])
        q = r.as_quat() # [x, y, z, w]

        # Assign orientation to target pose
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
        
        # --- POSITION CONSTRAINT ---
        pos_c = PositionConstraint()
        pos_c.header = target_pose.header
        pos_c.link_name = "lbr_link_ee" 
        
        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.BOX
        bounding_box.dimensions = [0.001, 0.001, 0.001] 
        
        pos_c.constraint_region.primitives.append(bounding_box)
        pos_c.constraint_region.primitive_poses.append(target_pose.pose)
        pos_c.weight = 1.0
        constraints.position_constraints.append(pos_c)

        # --- CORRECTED ORIENTATION CONSTRAINT ---
        ori_c = OrientationConstraint()
        ori_c.header = target_pose.header
        ori_c.link_name = "lbr_link_ee"
        
        # FIX: Link the constraint reference directly to your calculated target quaternion
        ori_c.orientation.x = q[0]
        ori_c.orientation.y = q[1]
        ori_c.orientation.z = q[2]
        ori_c.orientation.w = q[3]
        
        # Tight tolerances (0.01 rad ≈ 0.5 degrees) on all axes force 
        # the end-effector to match your exact specified angles.
        ori_c.absolute_x_axis_tolerance = 0.01 
        ori_c.absolute_y_axis_tolerance = 0.01
        ori_c.absolute_z_axis_tolerance = 0.01 
        ori_c.weight = 1.0
        constraints.orientation_constraints.append(ori_c)
        
        goal_msg.request.goal_constraints.append(constraints)

        # Send goal
        send_goal_future = self._action_client.send_goal_async(goal_msg)    
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("The request was rejected by the server.")
            return

        self.get_logger().info("Request accepted. Planning trajectory...")

        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        action_result = get_result_future.result()
        
        if action_result.status == 4:
            self.get_logger().info("--> Success! Exact Cartesian target reached.")
        else:
            self.get_logger().error(f"--> Failed. MoveIt Action Status: {action_result.status}")
def main(args=None):
    rclpy.init(args=args)
    node = CartesianController()
    node.run_cartesian_loop()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
