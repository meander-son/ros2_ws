import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('zivid_artist_bot')
    urdf_xacro = os.path.join(pkg_share, 'urdf', 'artist_bot.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'artist_bot.rviz')
        
    # Process the xacro into a string
    robot_description = os.popen(
        f"xacro {urdf_xacro} robot_name:=lbr mode:=true"
    ).read()

    # Declare our GUI argument
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description="Start the joint state publisher GUI for interactive visualization."
    )

    def create_nodes(context):
        # Read the argument at runtime
        use_gui = LaunchConfiguration('use_gui').perform(context).lower() == 'true'
        nodes = []

        # ── RVIZ ──────────────────────────────────────────────────────────────
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config], 
            output='screen'
        ))

        # ── ROBOT STATE PUBLISHER ─────────────────────────────────────────────
        nodes.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
            ],
        ))

        # ── JOINT STATE PUBLISHER GUI ─────────────────────────────────────────
        if use_gui:
            nodes.append(Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ))

        return nodes

    # Return the description and run the node creation function
    return LaunchDescription([
        use_gui_arg,
        OpaqueFunction(function=create_nodes)
    ])