#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations 

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
    goal_pose1 = create_pose_stamped(nav, 7.951173305511475, -0.004980068653821945, 0.0)
    # goal_pose2 = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    goal_pose2 = create_pose_stamped(nav, 1.2981137037277222,1.5658499002456665, -1.57)
    goal_pose3 = create_pose_stamped(nav, 1.1472392082214355, -0.7482457160949707, 0.0)
    
    # go to one pose
    # nav.goToPose(goal_pose1)
    # # Feedback
    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    #     # print(feedback)

    # Follow Waypoints
    for i in range(1):
        waypoints = [goal_pose1, goal_pose2, goal_pose3]
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            # print(feedback)        


    print(nav.getResult())        

    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
