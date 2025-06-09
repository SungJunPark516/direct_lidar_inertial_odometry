#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from copy import deepcopy

class RepublishModifiedTopics(Node):
    def __init__(self):
        super().__init__('republish_modified_topics')

        # Publishers
        self.map_pub = self.create_publisher(PointCloud2, '/map/repub', 10)
        self.keyframes_pub = self.create_publisher(PointCloud2, '/dlio/odom_node/keyframes/repub', 10)
        self.odom_pub = self.create_publisher(Odometry, '/dlio/odom_node/odom/repub', 10)

        # Subscribers
        self.create_subscription(PointCloud2, '/map', self.map_callback, 10)
        self.create_subscription(PoseArray, '/dlio/odom_node/keyframes', self.keyframes_callback, 10)
        self.create_subscription(Odometry, '/dlio/odom_node/odom', self.odom_callback, 10)

    def map_callback(self, msg):
        new_msg = deepcopy(msg)
        new_msg.header.frame_id = 'map'
        num_points = new_msg.width * new_msg.height
        self.get_logger().info(f"[map_callback] Number of points in PointCloud2: {num_points}")
        self.map_pub.publish(new_msg)

    def keyframes_callback(self, msg):
        points = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            intensity = 1.0
            points.append([x, y, z, intensity])

        header = deepcopy(msg.header)
        header.frame_id = 'map'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, points)
        self.get_logger().info(f"[keyframes_callback] Number of keyframes: {len(msg.poses)}")
        self.keyframes_pub.publish(cloud_msg)

    def odom_callback(self, msg):
        new_msg = deepcopy(msg)
        new_msg.header.frame_id = 'map'
        self.odom_pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RepublishModifiedTopics()
    print("RepublishModifiedTopics node is running...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
