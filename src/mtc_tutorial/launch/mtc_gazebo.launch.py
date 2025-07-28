import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
import yaml

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
            description="Whether to use Gazebo/Ignition"
        ),
        DeclareLaunchArgument(
            "is_robot_state_publisher_launched",
            default_value="True",
            description="If rsp is launched in other node, don't launch it again"
        ),
    ]

    # Paths
    panda_desc_share = get_package_share_directory("panda_description")
    panda_moveit_share = get_package_share_directory("panda_moveit_config")
    urdf_path = os.path.join(panda_desc_share, "urdf", "panda.urdf")
    srdf_path = os.path.join(panda_moveit_share, "srdf", "panda.srdf")
    kinematics_yaml = os.path.join(panda_moveit_share, "config", "kinematics.yaml")
    controllers_yaml = os.path.join(panda_moveit_share, "config", "panda_controllers.yaml")
    rviz_config = os.path.join(panda_moveit_share, "launch", "moveit.rviz")

    # Load robot descriptions
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    with open(srdf_path, 'r') as infp:
        robot_description_semantic = infp.read()
    with open(kinematics_yaml, 'r') as infp:
        robot_description_kinematics = yaml.safe_load(infp)
    with open(controllers_yaml, 'r') as infp:
        controllers = yaml.safe_load(infp)

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        condition=None
    )

    # Gazebo/Ignition
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ]),
        launch_arguments=[
            ("gz_args", [" -v 4 -r empty.sdf "])
        ]
    )

    # Spawn Panda in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "panda"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    # Resource path for gripper meshes
    gazebo_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(get_package_share_directory("panda_description")).parent.resolve())
        ]
    )

    # Clock bridge
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # MoveIt move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": robot_description,
             "robot_description_semantic": robot_description_semantic,
             "robot_description_kinematics": robot_description_kinematics,
             "use_sim_time": LaunchConfiguration("use_sim_time")},
            controllers
        ]
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            {"robot_description": robot_description,
             "robot_description_semantic": robot_description_semantic,
             "robot_description_kinematics": robot_description_kinematics,
             "use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    # MTC node
    mtc_node = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description,
             "robot_description_semantic": robot_description_semantic,
             "robot_description_kinematics": robot_description_kinematics,
             "use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    # MoveIt controllers spawner (if needed)
    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # MoveIt launch (if you have a main launch for MoveIt, include it here)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([panda_moveit_share, "launch", "moveit.launch.py"])
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
        controllers_spawner,
        move_group_node,
        rviz_node,
        mtc_node,
        # moveit_launch,  # Uncomment if you want to use the main MoveIt launch
    ]

    return LaunchDescription(declared_arguments + nodes_to_launch)