#! /usr/bin/env python3
import time # Time library

from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module

'''
Navigates a robot through goal poses.
'''
def main():

  # Start the ROS 2 Python Client Library
  rclpy.init()

  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()

  # # Set our demo's initial pose
  # initial_pose = PoseStamped()
  # initial_pose.header.frame_id = 'map'
  # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  # initial_pose.pose.position.x = 0.0
  # initial_pose.pose.position.y = 0.0
  # initial_pose.pose.orientation.z = 1.0
  # initial_pose.pose.orientation.w = 0.0
  # navigator.setInitialPose(initial_pose)

  # Wait for navigation to fully activate. Use this line if autostart is set to true.
  navigator.waitUntilNav2Active()

  # Set the robot's goal poses
  goal_poses = []
  
  # pose1
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 2.0
  goal_pose.pose.position.y = 0.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
  goal_poses.append(goal_pose)
  
  # pose2
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 2.0
  goal_pose.pose.position.y = -5.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
  goal_poses.append(goal_pose)

  # pose3
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -2.0
  goal_pose.pose.position.y = -5.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
  goal_poses.append(goal_pose)

  # pose4
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -2.0
  goal_pose.pose.position.y = 0.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
  goal_poses.append(goal_pose)  
  
  # pose1
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 2.0
  goal_pose.pose.position.y = 0.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
  goal_poses.append(goal_pose)
  
  # pose2
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 2.0
  goal_pose.pose.position.y = -5.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
  goal_poses.append(goal_pose)

  # pose3
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -2.0
  goal_pose.pose.position.y = -5.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = -1.0
  goal_pose.pose.orientation.w = 0.0
  goal_poses.append(goal_pose)


  # Go through the goal poses
  navigator.goThroughPoses(goal_poses)

  while not navigator.isTaskComplete():

    feedback = navigator.getFeedback()
    print('Distance remaining: ' + '{:.2f}'.format(
          feedback.distance_remaining) + ' meters.' + ' | '
          'Estimated time of arrival: ' + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                + ' seconds.')
    
    # # Some navigation timeout to demo cancellation
    # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=10000.0):
    #     navigator.cancelTask()

  # Do something depending on the return code
  result = navigator.getResult()
  if result == TaskResult.SUCCEEDED:
      print('Goal succeeded!')
  elif result == TaskResult.CANCELED:
      print('Goal was canceled!')
  elif result == TaskResult.FAILED:
      print('Goal failed!')
  else:
      print('Goal has an invalid return status!')

  exit(0)

if __name__ == '__main__':
  main()
