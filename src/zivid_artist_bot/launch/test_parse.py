import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

urdf = os.path.join(get_package_share_directory('iiwa14_description'), 'urdf', 'iiwa14', 'iiwa14.xacro')
configs = (
    MoveItConfigsBuilder("iiwa14", package_name="iiwa14_moveit_config")
    .robot_description(file_path=urdf, mappings={"robot_name": "lbr", "mode": "mock"})
    .robot_description_semantic(mappings={"robot_name": "lbr"})
    .to_moveit_configs()
)

print("--- SRDF (SEMANTIC) OUTPUT ---")
print(configs.robot_description_semantic["robot_description_semantic"])