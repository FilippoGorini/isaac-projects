import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    my_package = "vla_kinova_tabletop"
    
    # Generate the standard URDF using xacro without any extra patches
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            os.path.join(get_package_share_directory(my_package), "urdf", "gen3.xacro"), " ",
            "arm:=gen3 dof:=6 gripper:=robotiq_2f_85 vision:=true sim_isaac:=true use_fake_hardware:=false ",
            "isaac_arm_joint_commands:=/isaac_arm_commands ",
            "isaac_gripper_joint_commands:=/isaac_gripper_commands",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Node 1: Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Node 2: ROS 2 Control Node (The Bridge)
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

    home_node = Node(
        package="vla_kinova_tabletop",
        executable="home_robot.py",
        output="screen",
    )

    home_on_jtc_ready = RegisterEventHandler(
        OnProcessExit(target_action=jtc_spawner, on_exit=[home_node])
    )

    return [
        rsp_node,
        ros2_control_node,
        jsb_spawner,
        jtc_spawner,
        gripper_spawner,
        home_on_jtc_ready,
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
