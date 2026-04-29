import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    # Path to the base moveit config we are leveraging
    kortex_moveit_package = "kinova_gen3_6dof_robotiq_2f_85_moveit_config"
    my_package = "vla_kinova_tabletop"
    
    # Configuration arguments that must match the URDF export exactly
    # These are used to rebuild the description dynamically
    launch_arguments = {
        "arm": "gen3",
        "dof": "6",
        "gripper": "robotiq_2f_85",
        "vision": "true",
        "sim_isaac": "true",
        "use_fake_hardware": "false",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "use_external_cable": "false",
        "isaac_arm_joint_commands": "/isaac_arm_commands",
        "isaac_gripper_joint_commands": "/isaac_gripper_commands",
    }

    # 1. URDF Generation
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            os.path.join(get_package_share_directory(my_package), "urdf", "gen3.xacro"), " ",
            "arm:=gen3 dof:=6 gripper:=robotiq_2f_85 vision:=true sim_isaac:=true use_fake_hardware:=false use_external_cable:=false ",
            "isaac_arm_joint_commands:=/isaac_arm_commands ",
            "isaac_gripper_joint_commands:=/isaac_gripper_commands",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 2. MoveIt Configuration
    # Leverages Kortex package for SRDF, OMPL, and Kinematics, but overrides controllers
    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name=kortex_moveit_package)
        .robot_description(
            file_path=os.path.join(get_package_share_directory(my_package), "urdf", "gen3.xacro"),
            mappings=launch_arguments
        )
        .trajectory_execution(file_path=os.path.join(get_package_share_directory(my_package), "config", "moveit_controllers.yaml"))
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 3. Nodes
    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # ROS 2 Control Node (The Bridge)
    # This node loads the TopicBasedSystem specified in the URDF
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(get_package_share_directory(my_package), "config", "ros2_controllers.yaml"),
        ],
        output="both",
    )

    # Controller Spawners
    jsb_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    jtc_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_trajectory_controller"])
    gripper_spawner = Node(package="controller_manager", executable="spawner", arguments=["robotiq_gripper_controller"])

    # Move Group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    # RViz
    rviz_config_file = os.path.join(get_package_share_directory(my_package), "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return [
        rsp_node,
        ros2_control_node,
        jsb_spawner,
        jtc_spawner,
        gripper_spawner,
        move_group_node,
        rviz_node,
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
