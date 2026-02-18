#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set initial pose (important if not using simulation default)
    # Since we are using fake localization, the robot starts at (0,0) on the map
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    # navigator.setInitialPose(initial_pose)

    # Wait for Nav2 to be active
    navigator.waitUntilNav2Active()

    # Define waypoints
    waypoints = []
    
    # Waypoint 1
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.0
    goal_pose1.pose.position.y = 0.5
    goal_pose1.pose.orientation.w = 1.0
    waypoints.append(goal_pose1)

    # Waypoint 2
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.0
    goal_pose2.pose.position.y = 0.0
    goal_pose2.pose.orientation.w = 1.0
    waypoints.append(goal_pose2)

    # Waypoint 3
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 0.0
    goal_pose3.pose.position.y = 0.0
    goal_pose3.pose.orientation.w = 1.0
    waypoints.append(goal_pose3)

    print("Sending waypoints...")
    navigator.followWaypoints(waypoints)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing waypoint: ' + str(feedback.current_waypoint))
        time.sleep(1)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
