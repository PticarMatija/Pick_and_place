import os
import yaml
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    baxter_description_pkg = get_package_share_directory("baxter_description")
    urdf_xacro_path = os.path.join(baxter_description_pkg, "urdf", "baxter2_include.urdf.xacro")
    robot_description_content = subprocess.check_output(['xacro', urdf_xacro_path, 'gazebo:=0']).decode()
    robot_description = {"robot_description": robot_description_content}

    baxter_moveit_pkg = get_package_share_directory("baxter_moveit_config")
    srdf_xacro_path = os.path.join(baxter_moveit_pkg, "config", "baxter.srdf.xacro")
    robot_description_semantic_content = subprocess.check_output(['xacro', srdf_xacro_path]).decode()
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

   
    kinematics_yaml_path = os.path.join(baxter_moveit_pkg, "config", "kinematics.yaml")
    with open(kinematics_yaml_path, "r") as f:
        kinematics_yaml_content = yaml.safe_load(f)
    kinematics_params = {"robot_description_kinematics": kinematics_yaml_content}

    ompl_yaml_path = os.path.join(baxter_moveit_pkg, "config", "ompl_planning.yaml")
    with open(ompl_yaml_path, "r") as f:
        ompl_yaml_content = yaml.safe_load(f)
    ompl_params = {"ompl": ompl_yaml_content}

    baxter_mtc_node = Node(
        package="baxter_mtc",
        executable="baxter_mtc_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_params,
            ompl_params,
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([baxter_mtc_node])
