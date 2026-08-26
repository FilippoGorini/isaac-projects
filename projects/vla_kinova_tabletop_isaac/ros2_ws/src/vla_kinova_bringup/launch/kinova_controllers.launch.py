import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    my_package = "vla_kinova_bringup"
    description_pkg = "vla_kinova_description"
    use_sim    = LaunchConfiguration("use_sim").perform(context)
    robot_ip   = LaunchConfiguration("robot_ip").perform(context)
    auto_home  = LaunchConfiguration("auto_home").perform(context)
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity").perform(context)
    gripper_max_force = LaunchConfiguration("gripper_max_force").perform(context)
    tf_publish_rate = LaunchConfiguration("tf_publish_rate").perform(context)
    unwrap_joint_states = LaunchConfiguration("unwrap_joint_states").perform(context)
    open_loop_control = LaunchConfiguration("open_loop_control").perform(context).lower() == "true"
    is_sim     = use_sim.lower() == "true"
    use_sim_time = is_sim

    xacro_args = (
        f"arm:=gen3 dof:=6 gripper:=robotiq_2f_85 vision:=true "
        f"sim_isaac:={use_sim} use_fake_hardware:=false "
        f"gripper_joint_name:=robotiq_85_left_knuckle_joint "
        f"gripper_max_velocity:={gripper_max_velocity} "
        f"gripper_max_force:={gripper_max_force} "
        f"isaac_arm_joint_commands:=/isaac_arm_commands "
        f"isaac_gripper_joint_commands:=/isaac_gripper_commands "
        f"unwrap_joint_states:={unwrap_joint_states}"
    )
    if not is_sim:
        # On real hardware, route the Robotiq gripper through the Kinova arm's
        # internal bus (Kortex). Otherwise the gripper plugin tries /dev/ttyUSB0.
        xacro_args += f" robot_ip:={robot_ip} use_internal_bus_gripper_comm:=true"

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            os.path.join(get_package_share_directory(description_pkg), "urdf", "gen3.xacro"), " ",
            xacro_args,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        # Manually increase the frequency for /tf publishing so that quest2ros2 node can better read current end effector pose
        parameters=[robot_description, {"use_sim_time": use_sim_time, "publish_frequency": float(tf_publish_rate)}],
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

    # Create temp file and use it to override open_loop_control parameter for the JTC (by default we're in closed loop but we need open loop for the VLA deployment)
    jtc_args = ["joint_trajectory_controller"]
    if open_loop_control:
        ol = tempfile.NamedTemporaryFile(
            mode="w", suffix="_jtc_open_loop.yaml", delete=False)
        ol.write("joint_trajectory_controller:\n"
                 "  ros__parameters:\n"
                 "    open_loop_control: true\n")
        ol.close()
        jtc_args += ["--param-file", ol.name]

    jsb_spawner     = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    jtc_spawner     = Node(package="controller_manager", executable="spawner", arguments=jtc_args)
    gripper_spawner = Node(package="controller_manager", executable="spawner", arguments=["robotiq_gripper_controller"])

    nodes = [rsp_node, ros2_control_node, jsb_spawner, jtc_spawner, gripper_spawner]

    # Real-robot-only controllers: twist (loaded inactive) and fault reset.
    # tcp/twist.* interfaces and fault reset are exposed by kortex_driver, not Isaac sim.
    if not is_sim:
        twist_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["twist_controller", "--inactive"],
        )
        fault_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["fault_controller"],
        )
        # We now also load the gripper velocity controller (inactive). The twist pose tracking launchfile switches it active instead
        gripper_vel_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_velocity_controller", "--inactive"],
        )
        # Also load gripper position controller (inactive) if we want to track continuous VLA gripper output (even though this results in steppy motion)
        gripper_pos_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_position_controller", "--inactive"],
        )
        nodes += [twist_spawner, fault_spawner, gripper_vel_spawner, gripper_pos_spawner]

    # Home on startup: always in sim, opt-in on real robot via auto_home:=true
    run_home = is_sim or auto_home.lower() == "true"
    if run_home:
        home_node = Node(
            package=my_package,
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
            default_value="192.168.50.12",
            description="IP address of the real Kinova arm (ignored when use_sim:=true)",
        ),
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Gripper go-to speed limit [0-100%] used in the low-level cyclic "
                        "frame. Lower it to smooth the discrete go-to stepping (slower gripper).",
        ),
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Gripper grasp force limit [0-100%]. Lower it for delicate objects. "
                        "Note: only applied in low-level cyclic mode; high-level twist teleop ignores it.",
        ),
        DeclareLaunchArgument(
            "tf_publish_rate",
            default_value="200.0",
            description="robot_state_publisher /tf publish frequency [Hz]. Raise it for "
                        "smoother end-effector pose tracking (e.g. quest teleop), lower it to save CPU.",
        ),
        DeclareLaunchArgument(
            "auto_home",
            default_value="false",
            description="Run the homing script on startup (always true in sim, opt-in on real robot)",
        ),
        DeclareLaunchArgument(
            "unwrap_joint_states",
            default_value="false",
            description="Publish continuous (unwrapped) joint positions instead of the stock "
                        "[-pi,pi] wrap. Keep false for MoveIt / closed-loop JTC; set true for "
                        "teleop, data collection, and VLA deployment (real hardware only).",
        ),
        DeclareLaunchArgument(
            "open_loop_control",
            default_value="false",
            description="JTC open_loop_control. false (closed loop, plans against measured "
                        "state) for MoveIt / regular bring-up; true (open loop) for VLA "
                        "deployment.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
