#! /usr/bin/env python3

import rclpy
import time
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():

    # Start the ROS 2 Python Client Library
    rclpy.init()

    # Launch the ROS 2 Navigation Stack
    navigator = BasicNavigator()

    # Wait for navigation to fully activate. Use this line if autostart is set to true.
    navigator.waitUntilNav2Active()

    # Set the robot's goal poses
    goal_poses = []

    #pose1
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.0 #6.0
    goal_pose.pose.position.y = 0.0 #1.0
    goal_pose.pose.orientation.w = 1.0
    goal_poses.append(goal_pose)

    # Go through the goal poses
    navigator.goThroughPoses(goal_poses)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        print('Distance remaining: ' + '{:.2f}'.format(
            feedback.distance_remaining) + ' meters.' + ' | '
            'Estimated time of arrival: ' + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')
        # print(feedback)

    result = navigator.getResult()            
    if result == TaskResult.SUCCEEDED:
        print('Goal Succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was Canceled!')
    elif result == TaskResult.FAILED:
        print('Goal Failed!')
    else:
        print('Goal has an invalid return status!')        

    exit(0)

if __name__ == '__main__':
  main()