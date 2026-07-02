#!/usr/bin/env python3
import sys
import os
import tty
import termios

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class LaptopManualJogger(Node):
    def __init__(self):
        super().__init__('laptop_manual_jogger')
        
        # 1. Configuration Settings
        # Map to your exact joint names format (lbr_A1 to lbr_A7)
        self.joint_names = [
            'lbr_A1',
            'lbr_A2',
            'lbr_A3',
            'lbr_A4',
            'lbr_A5',
            'lbr_A6',
            'lbr_A7'
        ]
        
        # Seed initial internal joint state positions (all zero radians)
        self.current_joints = [0.0] * 7
        self.selected_joint = 0  # 0-6 index maps directly to A1-A7
        self.step_size = 0.05    # Radians (~2.8 degrees) step changes

        # 2. Setup Publisher
        # RViz's robot_state_publisher node subscribes directly to this topic
        self.joint_pub = self.create_publisher(JointState, '/lbr/joint_states', 10)
        
        # Print Control Map Information Layout
        self.print_user_ui()

        # Publish the first neutral pose so the robot appears in RViz immediately
        self.publish_updated_joints()

    def print_user_ui(self):
        print("\n" + "="*40)
        print("    ROS 2 KUKA iiwa Manual Jogger")
        print("="*40)
        print("Controls:")
        print("  [1-7] : Select Active Joint (A1 - A7)")
        print("  [w]   : Step Joint Angle Positive (+)")
        print("  [s]   : Step Joint Angle Negative (-)")
        print("  [q]   : Clean Exit Shutdown Sequence")
        print("-"*40)
        print(f"Initial State -> Active Selected Joint: lbr_A{self.selected_joint + 1}")

    def get_keyboard_key(self):
        """Captures a single raw keystroke from standard input without blocking threads."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
            if ch == '\x03':  # Catch standard Ctrl+C combinations
                raise KeyboardInterrupt
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def publish_updated_joints(self):
        """Constructs the JointState structure message payload and ships to RViz."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_joints
        
        # Velocity and Effort are optional; keeping them empty array lists is standard
        msg.velocity = []
        msg.effort = []
        
        self.joint_pub.publish(msg)

    def run_control_loop(self):
        """Main manual input driver loop."""
        while rclpy.ok():
            try:
                key = self.get_keyboard_key()
                
                if key == 'q':
                    self.get_logger().info("Quit key received. Shutting down manual jogger.")
                    break
                
                # Active selection indexing adjustments
                elif key in ['1', '2', '3', '4', '5', '6', '7']:
                    self.selected_joint = int(key) - 1
                    print(f"Selected Target: lbr_A{self.selected_joint + 1} (Current Angle: {self.current_joints[self.selected_joint]:.3f} rad)")
                    continue
                
                # Dynamic coordinate stepping logic
                elif key == 'w':
                    self.current_joints[self.selected_joint] += self.step_size
                    print(f"Moving lbr_A{self.selected_joint + 1} [+] to {self.current_joints[self.selected_joint]:.3f} rad")
                    
                elif key == 's':
                    self.current_joints[self.selected_joint] -= self.step_size
                    print(f"Moving lbr_A{self.selected_joint + 1} [-] to {self.current_joints[self.selected_joint]:.3f} rad")
                
                else:
                    continue # Bypass unmapped random terminal input noise safely

                # Fire data downstream to the RViz display interface pipeline
                self.publish_updated_joints()
                
            except KeyboardInterrupt:
                break


def main(args=None):
    rclpy.init(args=args)
    node = LaptopManualJogger()
    
    try:
        # Run loop sequence tracking
        node.run_control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\nExiting cleanly and stopping publisher node.")

if __name__ == '__main__':
    main()