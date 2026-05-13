#!/usr/bin/env python3
import sys
import argparse
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class HomeRobot(Node):
    def __init__(self, prefix='', positions=None):
        super().__init__('home_robot')

        prefix = prefix.strip('"\'')

        if positions is None:
            self.positions = [0.0, -0.565, -1.621, 0.0, -1.782, 1.5708]
        else:
            self.positions = positions

        self.joint_names = [f'{prefix}joint_{i+1}' for i in range(len(self.positions))]

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(1.0, self.home)
        self.done = False

        self.get_logger().info(f'Homing robot (prefix: "{prefix}") to positions: {self.positions}')

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

    parser = argparse.ArgumentParser(description='Home the Kinova arm to the default observation position')
    parser.add_argument('--prefix', type=str, default='', help='Joint name prefix')
    parser.add_argument('--positions', type=float, nargs='+', help='Target joint positions')

    ros_args = rclpy.utilities.remove_ros_args(sys.argv[1:])
    parsed_args = parser.parse_args(ros_args)

    node = HomeRobot(prefix=parsed_args.prefix, positions=parsed_args.positions)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
