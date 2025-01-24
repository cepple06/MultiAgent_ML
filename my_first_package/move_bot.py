#!/usr/bin/env python

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np

def main(robot_name,x,y):
    # Initialize the ROS node
    rospy.init_node("move_robot_node")

    # Start the action client for move_base
    client = actionlib.SimpleActionClient(robot_name+"/move_base", MoveBaseAction)
    rospy.loginfo("Wait for the action server to come up")
    client.wait_for_server()

    # Prepare the goal message
    goal = MoveBaseGoal()
    target_frame = robot_name+"/odom"
    goal.target_pose.header.frame_id = target_frame
    goal.target_pose.header.stamp = rospy.Time.now()

    # Read the list of coordinates from a file
    #coordinate_file = '/home/mesh/catkin_ws/src/move_robot/scripts/coordinate.txt'
    #coordinate = np.genfromtxt(coordinate_file, delimiter=',')  # Read as a 2D array

    # Send the goals one by one
    rospy.logwarn("Sending new goal point.")
    
    # Set the target position and orientation
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1  # Neutral orientation

    # Send the goal to the move_base server
    client.send_goal(goal)

    # Wait for the robot to reach the goal
    wait = client.wait_for_result()
    while not wait:
        wait = client.wait_for_result()

    # Log the success

# Run the script
if __name__ == "__main__":
    try:
        main("tb3_0",.5,-1)
    except:
        exit()
