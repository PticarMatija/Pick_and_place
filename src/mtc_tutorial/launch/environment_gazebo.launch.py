import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
from launch_param_builder import load_xacro


import yaml
import xacro

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            #default_value="panda_moveit_config_demo.rviz", #obican
            default_value="mtc.rviz", #za mtc
            description="RViz configuration file",
        )
    )

    #potrebno za moveit
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="my_config/panda_inertial.urdf.xacro") #da bude kompatibilno sa gazebom (ima inertial tagove itd) 
        .trajectory_execution(file_path="my_config/gripper_moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation,,, ovo ti je potrebno za moveit task constructor, bez ovog ce ti javljat: [my_pick_and_place_cpp-1] [ERROR] [1730125430.311979040] [mypickandplace]: Task execution failed
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_capabilities, {"use_sim_time": True}], 
    )

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("moveit2_tutorials"), "launch", rviz_base]
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher", 
        name="robot_state_publisher",
        output="both",
        
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        # "config",
        "my_config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node( #radi bez ovog, ovo je potrebno kad koristis samo rviz onda ne radi bez tog
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path], 
        output="both",
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ]
    )

    joint_state_broadcaster_spawner = Node( 
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", 
            "300",
            "--controller-manager", 
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node( 
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    hand_controller_spawner = Node( 
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
    )
    
    
    panda_description = get_package_share_directory("moveit_resources_panda_description")
    gazebo_resource_path = AppendEnvironmentVariable( 
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(panda_description).parent.resolve())
            ]
        )
    
    #gazebo ignition
    gazebo = IncludeLaunchDescription( #ignition
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4 -r empty.sdf "]
                    # ("gz_args", [f" -v 4 -r  {panda_ros2_gazebo}"]
                    )
                ]
            )
    
    #spawn robot in gazebo ignition
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "panda"],
    )
    
    nodes_to_start = [
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node, 
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        hand_controller_spawner,
        
        gz_ros2_bridge,
        
        gazebo_resource_path,
        gazebo,
        spawn_entity,
        
    ]

    return LaunchDescription(
        declared_arguments + nodes_to_start
    )

