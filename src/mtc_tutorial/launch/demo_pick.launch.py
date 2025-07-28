from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
#moveit_resources_panda
""" def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

   
    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="novi_mtc_node",
        output="screen",
        parameters=[
            moveit_config,
             {"use_sim_time": True},
        ],
    )

    return LaunchDescription([pick_place_demo])  """
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    urdf_path = "/home/developer/panda_ws/install/panda_description/share/panda_description/urdf/panda.urdf"
    srdf_path = "/home/developer/panda_ws/install/panda_moveit_config/share/panda_moveit_config/srdf/panda.srdf"
    kinematics_yaml = "/home/developer/panda_ws/install/panda_moveit_config/share/panda_moveit_config/config/kinematics.yaml"

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    with open(srdf_path, 'r') as infp:
        robot_description_semantic = infp.read()

    mtc_node = Node(
        package="mtc_tutorial",
        executable="novi_mtc_node",
        output="screen",
        parameters=[ {"use_sim_time": True},
            {"robot_description": robot_description,
             "robot_description_semantic": robot_description_semantic,
             "robot_description_kinematics": yaml.safe_load(open(kinematics_yaml))},
            {"planning_plugin": "ompl_interface/OMPLPlanner"}
        ],
    )

    return LaunchDescription([mtc_node])