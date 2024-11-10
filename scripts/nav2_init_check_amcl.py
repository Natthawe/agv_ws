#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

def main():
    # Init
    rclpy.init()
    node = Node("navigation_check")
    nav = BasicNavigator()

    # Check if navigation stack is running by looking for the /amcl_pose topic
    topic_name = '/amcl_pose'
    if node.count_publishers(topic_name) > 0:
        # Navigation stack is running, set the initial pose

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = -1.89
        initial_pose.pose.position.y = -0.57
        initial_pose.pose.position.z = 0.0

        nav.setInitialPose(initial_pose)
        node.get_logger().info("Initial pose published successfully.")
    else:
        # Navigation stack is not running
        node.get_logger().warn("Navigation stack is not running. Not publishing initial pose.")

    # Shut down
    rclpy.shutdown()

if __name__ == '__main__':
    main()
