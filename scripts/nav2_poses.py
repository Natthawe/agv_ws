#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations 
from rclpy.duration import Duration


def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z) #x, y, z
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose


def main():
    # Init
    rclpy.init()
    nav = BasicNavigator()

    # initial_pose = create_pose_stamped(nav, -2.0, -0.5, 0.0) #BasicNavigator, position_x, position_y, orientation_z
    # nav.setInitialPose(initial_pose)

    # Wait for Nav2
    nav.waitUntilNav2Active()
    
    # Send Nav2 goal
    goal_pose1 = create_pose_stamped(nav, 7.0, 0.4, 0.0)
    goal_pose2 = create_pose_stamped(nav, 7.0, 3.0, 1.57)
    goal_pose3 = create_pose_stamped(nav, -4.0, 2.8, 3.14)
    goal_pose4= create_pose_stamped(nav, -4.0, 2.8, 3.14)
    goal_pose5 = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    
    # go to one pose
    # nav.goToPose(goal_pose1)
    # # Feedback
    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    #     # print(feedback)

    # Follow Waypoints
    for i in range(3):
        waypoints = [goal_pose1, goal_pose2, goal_pose3, goal_pose4, goal_pose5]
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()    
            print('Current Waypoint: ' + str(feedback.current_waypoint))

    result = nav.getResult()            
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
