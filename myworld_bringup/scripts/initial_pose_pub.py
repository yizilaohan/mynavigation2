#!/usr/bin/env python3

"""Seed external localization by publishing /initialpose, like one RViz 2D Pose Estimate click."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class InitialPosePub(Node):
    def __init__(self):
        super().__init__('initial_pose_pub')
        # 每次启动必定重定位的种子位姿：机器人停靠点在地图系 (map frame) 坐标，
        # 与 nav2_regulated_modules 测试工具里的 A 点 (9.16, -4.37) 一致。
        self.declare_parameter('x', 9.1631439)
        self.declare_parameter('y', -4.3752599)
        self.declare_parameter('yaw', -2.1613309)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/initialpose')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('publish_duration', 5.0)

        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)
        frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        publish_duration = float(self.get_parameter('publish_duration').value)

        # RViz 2D Pose Estimate covariance: 0.25 m in x/y, 0.0685 rad^2 in yaw.
        self.message = PoseWithCovarianceStamped()
        self.message.header.frame_id = frame_id
        self.message.pose.pose.position.x = x
        self.message.pose.pose.position.y = y
        self.message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        self.message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.message.pose.covariance[0] = 0.25
        self.message.pose.covariance[7] = 0.25
        self.message.pose.covariance[35] = 0.068538919091789

        initial_pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, topic, initial_pose_qos)
        self.publish_count = max(1, int(round(publish_duration * publish_rate)))
        self.publish_period = 1.0 / publish_rate
        self.get_logger().info(
            f'Seeding {frame_id} pose ({x:.7f}, {y:.7f}, yaw {yaw:.7f}) on {topic}')

    def run(self):
        # Repeat briefly so a still-initializing external localizer does not miss the seed.
        for _ in range(self.publish_count):
            if not rclpy.ok():
                break
            self.message.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.message)
            time.sleep(self.publish_period)
        self.get_logger().info('Initial pose seed complete')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePub()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
