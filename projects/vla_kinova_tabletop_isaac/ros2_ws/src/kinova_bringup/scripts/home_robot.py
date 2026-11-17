#!/usr/bin/env python3
import os
import sys
import argparse
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def load_profile(robot):
    """robot.yaml parameters of a vla_robots profile."""
    path = os.path.join(get_package_share_directory('vla_robots'), 'config', robot, 'robot.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


class HomeRobot(Node):
    def __init__(self, robot, positions=None):
        super().__init__('home_robot')

        # Joints, home pose and JTC topic come from the robot profile
        profile = load_profile(robot)
        self.joint_names = profile['arm_joints']
        self.positions = positions if positions is not None else profile['home_positions']
        if len(self.positions) != len(self.joint_names):
            raise SystemExit(f'{len(self.positions)} positions for {len(self.joint_names)} joints')

        self.publisher_ = self.create_publisher(JointTrajectory, profile['arm_trajectory_topic'], 10)

        self.timer = self.create_timer(1.0, self.home)
        self.done = False

        self.get_logger().info(f'Homing robot (profile: {robot}) to positions: {self.positions}')

    def home(self):
        if self.done:
            return

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start = Duration(sec=4, nanosec=0)

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info('Sent homing trajectory')
        self.done = True

        self.create_timer(2.0, lambda: sys.exit(0))


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Home the arm to the robot profile\'s home_positions')
    parser.add_argument('--robot', type=str, default='kinova_gen3_6dof', help='vla_robots profile')
    parser.add_argument('--positions', type=float, nargs='+', help='Target joint positions (overrides the profile)')

    ros_args = rclpy.utilities.remove_ros_args(sys.argv[1:])
    parsed_args = parser.parse_args(ros_args)

    node = HomeRobot(robot=parsed_args.robot, positions=parsed_args.positions)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
