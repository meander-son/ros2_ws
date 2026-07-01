import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory('zivid_artist_bot')
    urdf_xacro = os.path.join(description_share, 'urdf', 'artist_bot.xacro')

    rviz_config = os.path.join(
        get_package_share_directory('iiwa14_moveit_config'),
        'config',
        'moveit.rviz',
    )

    # IMPORTANT: keep this aligned with the live iiwa_joint_* joint naming used by the URDF.
    ros2_controllers_path = os.path.join(
        get_package_share_directory('iiwa14_description'),
        'ros2_control',
        'lbr_controllers.yaml',
    )

    srdf_path = os.path.join(
        get_package_share_directory('iiwa14_bringup'),
        'config',
        'iiwa14_semantic.srdf',
    )

    robot_namespace = 'lbr'

    moveit_config = (
        MoveItConfigsBuilder("iiwa14", package_name="iiwa14_moveit_config")
        .robot_description(
            file_path=urdf_xacro,
            # Force the internal macro to evaluate without a custom string prefix
            mappings={"robot_name": "", "mode": "mock"}, 
        )
        .robot_description_semantic(file_path=srdf_path)
        .to_moveit_configs()
    )


    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description="Start the joint state publisher GUI."
    )

    def create_nodes(context):
        use_gui = LaunchConfiguration('use_gui').perform(context).lower() == 'true'
        nodes = []

        move_group_runtime = {
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "monitor_dynamics": False,
        }

        # ros2_control controller manager
        nodes.append(Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=robot_namespace,
            parameters=[
                moveit_config.robot_description,
                ros2_controllers_path,
            ],
            output="screen",
        ))

        # spawn controllers
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", f"/{robot_namespace}/controller_manager",
            ],
            output="screen",
        ))

        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_trajectory_controller",
                "--controller-manager", f"/{robot_namespace}/controller_manager",
            ],
            output="screen",
        ))

        nodes.append(Node(
            package="moveit_ros_move_group",
            executable="move_group",
            namespace=robot_namespace,
            output="screen",
            parameters=[moveit_config.to_dict(), move_group_runtime],
        ))

        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
            ],
        ))

        nodes.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot_namespace,
            output="screen",
            parameters=[moveit_config.robot_description],
        ))

        if use_gui:
            nodes.append(Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                namespace=robot_namespace,
            ))

        return nodes

    return LaunchDescription([
        use_gui_arg,
        OpaqueFunction(function=create_nodes),
    ])