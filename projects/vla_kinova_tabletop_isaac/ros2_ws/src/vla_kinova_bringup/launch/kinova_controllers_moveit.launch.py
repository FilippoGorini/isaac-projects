import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    kortex_moveit_package = "kinova_gen3_6dof_robotiq_2f_85_moveit_config"
    my_package = "vla_kinova_bringup"
    description_pkg = "vla_kinova_description"
    use_sim     = LaunchConfiguration("use_sim").perform(context)
    robot_ip    = LaunchConfiguration("robot_ip").perform(context)
    auto_home   = LaunchConfiguration("auto_home").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity").perform(context)
    gripper_max_force = LaunchConfiguration("gripper_max_force").perform(context)
    tf_publish_rate = LaunchConfiguration("tf_publish_rate").perform(context)
    is_sim    = use_sim.lower() == "true"
    use_sim_time = is_sim

    # Controller stack (RSP, ros2_control, spawners, homing)
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(my_package),
            "launch",
            "kinova_controllers.launch.py",
        )),
        launch_arguments={
            "use_sim": use_sim,
            "robot_ip": robot_ip,
            "auto_home": auto_home,
            "gripper_max_velocity": gripper_max_velocity,
            "gripper_max_force": gripper_max_force,
            "tf_publish_rate": tf_publish_rate,
        }.items(),
    )

    # Build moveit_config so move_group / RViz get robot_description, SRDF and
    # kinematics. (The controllers launch already produces the robot_description
    # consumed by RSP and ros2_control; this is the MoveIt-side copy.)
    moveit_mappings = {
        "arm": "gen3",
        "dof": "6",
        "gripper": "robotiq_2f_85",
        "vision": "true",
        "sim_isaac": use_sim,
        "use_fake_hardware": "false",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "use_external_cable": "false",
        "isaac_arm_joint_commands": "/isaac_arm_commands",
        "isaac_gripper_joint_commands": "/isaac_gripper_commands",
    }
    if not is_sim:
        moveit_mappings["robot_ip"] = robot_ip
        moveit_mappings["use_internal_bus_gripper_comm"] = "true"

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name=kortex_moveit_package)
        .robot_description(
            file_path=os.path.join(get_package_share_directory(description_pkg), "urdf", "gen3.xacro"),
            mappings=moveit_mappings,
        )
        .trajectory_execution(file_path=os.path.join(get_package_share_directory(my_package), "config", "moveit_controllers.yaml"))
        .joint_limits(file_path=os.path.join(get_package_share_directory(my_package), "config", "joint_limits.yaml"))
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )

    rviz_config_file = os.path.join(get_package_share_directory(my_package), "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )

    return [controllers_launch, move_group_node, rviz_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Set to true when running in Isaac Sim, false for the real robot",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.50.12",
            description="IP address of the real Kinova arm (ignored when use_sim:=true)",
        ),
        DeclareLaunchArgument(
            "auto_home",
            default_value="false",
            description="Run the homing script on startup (always true in sim, opt-in on real robot)",
        ),
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Gripper go-to speed limit [0-100%], forwarded to kinova_controllers.launch.py",
        ),
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Gripper grasp force limit [0-100%], forwarded to kinova_controllers.launch.py",
        ),
        DeclareLaunchArgument(
            "tf_publish_rate",
            default_value="200.0",
            description="robot_state_publisher /tf publish frequency [Hz], forwarded to kinova_controllers.launch.py",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Bring up the MoveIt RViz (MotionPlanning) alongside move_group",
        ),
        OpaqueFunction(function=launch_setup),
    ])
