import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 1. Gather relevant package paths
    description_share = get_package_share_directory('zivid_artist_bot')
    urdf_xacro = os.path.join(description_share, 'urdf', 'artist_bot.xacro')
    rviz_config = os.path.join(
        get_package_share_directory('iiwa14_moveit_config'),
        'config',
        'moveit.rviz',
    )
    robot_namespace = 'lbr'
    
    # add near other paths
    srdf_path = os.path.join(
    get_package_share_directory('iiwa14_bringup'),
    'config',
    'iiwa14_semantic.srdf',
    )

    moveit_config = (
    MoveItConfigsBuilder("iiwa14", package_name="iiwa14_moveit_config")
    .robot_description(file_path=urdf_xacro, mappings={"robot_name": robot_namespace, "mode": "mock"})
    .robot_description_semantic(file_path=srdf_path)
    .to_moveit_configs()
    )   

    # Declare GUI argument for fallback tracking
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',  # Set default to false since MoveIt will track joint state publishers instead
        description="Start the joint state publisher GUI."
    )

    def create_nodes(context):
        use_gui = LaunchConfiguration('use_gui').perform(context).lower() == 'true'
        nodes = []

        move_group_runtime = {
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": True,
            "capabilities": "",
            "disable_capabilities": "",
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "monitor_dynamics": False,
        }

        # ── MOVE GROUP (The Core MoveIt Brain) ────────────────────────────────
        nodes.append(Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            namespace=robot_namespace,
            parameters=[moveit_config.to_dict(), move_group_runtime],
        ))

        # ── RVIZ2 (With MoveIt parameters fully mapped) ───────────────────────
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
            ],
            output='screen'
        ))

        # ── ROBOT STATE PUBLISHER ─────────────────────────────────────────────
        nodes.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            namespace=robot_namespace,
            parameters=[moveit_config.robot_description],
        ))

        # ── CONTROLLERS / JOINT STATE TRACKING ────────────────────────────────
        # MoveIt relies on incoming joint states from your controllers.
        # For mock/simulation, we track using the moveit joint state broadcaster:
        nodes.append(Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            namespace=robot_namespace,
            parameters=[
                moveit_config.robot_description,
                {"source_list": ["move_group/joint_states"]},
            ],
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
        OpaqueFunction(function=create_nodes)
    ])