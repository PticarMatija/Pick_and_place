import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
import subprocess

def load_xacro(xacro_path, mappings=None):
    # mappings: dict of xacro args
    cmd = ['xacro', str(xacro_path)]
    if mappings:
        for k, v in mappings.items():
            cmd.append(f'{k}:={v}')
    return subprocess.check_output(cmd).decode()

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulation clock"
        ),
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="True",
            description="Whether to use Gazebo for simulation"
        ),
        DeclareLaunchArgument(
            "is_robot_state_publisher_launched",
            default_value="True",
            description="If rsp is launched in other node, don't launch it again"
        ),
    ]

    panda_description_pkg = get_package_share_directory("moveit_resources_panda_description")
    panda_moveit_pkg = get_package_share_directory("moveit_resources_panda_moveit_config")

    panda_urdf_xacro = os.path.join(panda_moveit_pkg, "config", "panda.urdf.xacro")
    robot_description = {
        "robot_description": load_xacro(Path(panda_urdf_xacro), mappings={"gazebo": "True"})
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    gui_config_path = os.path.join(panda_moveit_pkg, "config", "gazebo_gui.config")
    world_path = "empty.sdf"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ]),
        launch_arguments={
            "world": world_path,
            "verbose": "true",
        }.items()
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "panda",
            "-x", "0", "-y", "0", "-z", "0.5"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    gazebo_resource_path = AppendEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            str(Path(panda_description_pkg).parent.resolve()),
        ]
    )

    # Bridge clock if needed (optional)
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # MoveIt launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(panda_moveit_pkg, "launch", "demo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items()
    )

    nodes_to_launch = [
        robot_state_publisher,
        gazebo,
        spawn_entity,
        gazebo_resource_path,
        gz_ros2_bridge,
        moveit_launch,
    ]

    return LaunchDescription(
        declared_arguments + nodes_to_launch
    )