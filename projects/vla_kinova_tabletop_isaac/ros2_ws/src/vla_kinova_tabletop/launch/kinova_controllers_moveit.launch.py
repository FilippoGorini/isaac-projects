import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    kortex_moveit_package = "kinova_gen3_6dof_robotiq_2f_85_moveit_config"
    my_package = "vla_kinova_tabletop"
    use_sim   = LaunchConfiguration("use_sim").perform(context)
    robot_ip  = LaunchConfiguration("robot_ip").perform(context)
    auto_home = LaunchConfiguration("auto_home").perform(context)
    is_sim    = use_sim.lower() == "true"
    use_sim_time = is_sim

    xacro_args = (
        f"arm:=gen3 dof:=6 gripper:=robotiq_2f_85 vision:=true "
        f"sim_isaac:={use_sim} use_fake_hardware:=false use_external_cable:=false "
        f"gripper_joint_name:=robotiq_85_left_knuckle_joint "
        f"isaac_arm_joint_commands:=/isaac_arm_commands "
        f"isaac_gripper_joint_commands:=/isaac_gripper_commands"
    )
    if not is_sim:
        # On real hardware, route the Robotiq gripper through the Kinova arm's
        # internal bus (Kortex). Otherwise the gripper plugin tries /dev/ttyUSB0.
        xacro_args += f" robot_ip:={robot_ip} use_internal_bus_gripper_comm:=true"

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

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            os.path.join(get_package_share_directory(my_package), "urdf", "gen3.xacro"), " ",
            xacro_args,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name=kortex_moveit_package)
        .robot_description(
            file_path=os.path.join(get_package_share_directory(my_package), "urdf", "gen3.xacro"),
            mappings=moveit_mappings,
        )
        .trajectory_execution(file_path=os.path.join(get_package_share_directory(my_package), "config", "moveit_controllers.yaml"))
        .joint_limits(file_path=os.path.join(get_package_share_directory(my_package), "config", "joint_limits.yaml"))
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        # Manually increase the frequency for /tf publishing so that quest2ros2 node can better read current end effector pose
        parameters=[robot_description, {"use_sim_time": use_sim_time, "publish_frequency": 150.0}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(get_package_share_directory(my_package), "config", "ros2_controllers.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        output="both",
    )

    jsb_spawner     = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    jtc_spawner     = Node(package="controller_manager", executable="spawner", arguments=["joint_trajectory_controller"])
    gripper_spawner = Node(package="controller_manager", executable="spawner", arguments=["robotiq_gripper_controller"])

    # Real-robot-only controllers: twist (loaded inactive) and fault reset.
    # tcp/twist.* interfaces and fault reset are exposed by kortex_driver, not Isaac sim.
    real_only_spawners = []
    if not is_sim:
        real_only_spawners.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["twist_controller", "--inactive"],
        ))
        real_only_spawners.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["fault_controller"],
        ))

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
    )

    nodes = [rsp_node, ros2_control_node, jsb_spawner, jtc_spawner, gripper_spawner, *real_only_spawners, move_group_node, rviz_node]

    # Home on startup: always in sim, opt-in on real robot via auto_home:=true
    run_home = is_sim or auto_home.lower() == "true"
    if run_home:
        home_node = Node(
            package="vla_kinova_tabletop",
            executable="home_robot.py",
            output="screen",
        )
        nodes.append(RegisterEventHandler(
            OnProcessExit(target_action=jtc_spawner, on_exit=[home_node])
        ))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Set to true when running in Isaac Sim, false for the real robot",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.2.12",
            description="IP address of the real Kinova arm (ignored when use_sim:=true)",
        ),
        DeclareLaunchArgument(
            "auto_home",
            default_value="false",
            description="Run the homing script on startup (always true in sim, opt-in on real robot)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
