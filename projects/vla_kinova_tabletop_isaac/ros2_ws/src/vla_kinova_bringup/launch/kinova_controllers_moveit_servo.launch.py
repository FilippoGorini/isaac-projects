import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    my_package = "vla_kinova_bringup"
    description_pkg = "vla_kinova_description"
    kortex_moveit_package = "kinova_gen3_6dof_robotiq_2f_85_moveit_config"

    use_sim   = LaunchConfiguration("use_sim").perform(context)
    robot_ip  = LaunchConfiguration("robot_ip").perform(context)
    auto_home = LaunchConfiguration("auto_home").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity").perform(context)
    tf_publish_rate = LaunchConfiguration("tf_publish_rate").perform(context)
    is_sim    = use_sim.lower() == "true"
    use_sim_time = is_sim

    # Bring up controllers
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
            "tf_publish_rate": tf_publish_rate,
        }.items(),
    )

    # Build moveit_config so the pose_tracking_node has robot_description / SRDF / kinematics.
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
        # Servo reads joint vel/accel limits from the RobotModel. URDF typically has
        # only velocity limits; joint_limits.yaml adds the acceleration limits
        .joint_limits(file_path=os.path.join(get_package_share_directory(my_package), "config", "joint_limits.yaml"))
        .to_moveit_configs()
    )

    # Servo params: merge servo.yaml + pose_tracking_settings.yaml under 'moveit_servo' ns.
    config_dir = os.path.join(get_package_share_directory(my_package), "config")
    servo_yaml = _load_yaml(os.path.join(config_dir, "servo.yaml"))
    pose_tracking_yaml = _load_yaml(os.path.join(config_dir, "pose_tracking_settings.yaml"))
    servo_params = {"moveit_servo": {**servo_yaml, **pose_tracking_yaml}}

    pose_tracking_node = Node(
        package=my_package,
        executable="pose_tracking_node",
        name="pose_tracking_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            servo_params,
            {"use_sim_time": use_sim_time},
        ],
        # Quest2ROS2's right_arm_controller publishes the desired EE pose on
        # /target_frame. PoseTracking subscribes to 'target_pose' internally,
        # so we remap to bridge the two without code changes.
        remappings=[("target_pose", "/target_frame")],
        # LC_NUMERIC override: same Italian-locale gotcha as elsewhere.
        # (This was needed on the lab desktops as they had different settings which ended up causing crashes)
        additional_env={"LC_NUMERIC": "en_US.UTF-8"},
    )

    # Servo-specific RViz config: no MotionPlanning panel (would error on missing
    # /move_group services since we deliberately don't launch move_group here).
    rviz_config_file = os.path.join(get_package_share_directory(my_package), "config", "servo.rviz")
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

    delayed_pose_tracking = TimerAction(
        period=5.0,
        actions=[pose_tracking_node],
    )

    return [controllers_launch, delayed_pose_tracking, rviz_node]


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
            "launch_rviz",
            default_value="true",
            description="Bring up RViz alongside the servo stack",
        ),
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Gripper go-to speed limit [0-100%], forwarded to kinova_controllers.launch.py",
        ),
        DeclareLaunchArgument(
            "tf_publish_rate",
            default_value="200.0",
            description="robot_state_publisher /tf publish frequency [Hz], forwarded to kinova_controllers.launch.py",
        ),
        OpaqueFunction(function=launch_setup),
    ])
