#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import roslaunch
import time
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import os


uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

def launch_all_robots(x):
    for i in range(x):
        launch_file = "src/turtlebot3_simulations/turtlebot3_gazebo/launch/singleTurtlebot.launch"
        # Launch arguments should be passed as a list of strings, not tuples
        launch_args = ['first_tb3:=tb3_' + str(i), 'first_tb3_x_pos:=' + str(i) + ".0"]

        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, launch_args)])
        launch.start()
        rospy.loginfo(f"TurtleBot tb3_{i} launched")
        time.sleep(2)  # Add a delay to ensure proper initialization

def launch_all_nav(x):
    
    for i in range(x):
        launch_file = "src/turtlebot3/turtlebot3_navigation/launch/turtlebot3_navigation.launch"
        # Launch arguments should be passed as a list of strings, not tuples
        launch_args = ['first_tb3:=tb3_' + str(i),"open_rviz:=false"]

        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, launch_args)])
        launch.start()
                 
        '''else:
            launch_file = "src/turtlebot3/turtlebot3_navigation/launch/turtlebot3_navigation.launch"
        # Launch arguments should be passed as a list of strings, not tuples
            launch_args = ['first_tb3:=tb3_' + str(i), "open_rviz:=false"]

            launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, launch_args)])
            launch.start()'''
        rospy.loginfo(f"TurtleBot tb3_{i} launched")
        time.sleep(2)  # Add a delay to ensure proper initialization

def launch_all_slam(x):
    
    for i in range(x):
            launch_file = "src/turtlebot3_simulations/turtlebot3_gazebo/launch/multi_turtlebot3_slam.launch"
            # Launch arguments should be passed as a list of strings, not tuples
            launch_args = ['ns:=tb3_' + str(i)]

            launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, launch_args)])
            launch.start()
            rospy.loginfo(f"TurtleBot tb3_{i} launched")
            time.sleep(2)  # Add a delay to ensure proper initialization

def launch_all_map(x):
    
    for i in range(x):
            launch_file = "src/turtlebot3_simulations/turtlebot3_gazebo/launch/single_map_merge.launch"
            # Launch arguments should be passed as a list of strings, not tuples
            launch_args = ['first_tb3:=tb3_' + str(i), 'first_tb3_x_pos:=' + str(i) + ".0"]

            launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, launch_args)])
            launch.start()
            rospy.loginfo(f"TurtleBot tb3_{i} launched")
            time.sleep(2)  # Add a delay to ensure proper initialization

def launch_rviz():
    launch_file = "src/turtlebot3/turtlebot3_navigation/launch/launch_rviz.launch"
    launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file)])
    launch.start()

def update_rviz_config(robot_number):
    # Path to your RViz configuration file
    rviz_config_path = os.path.expanduser('src/turtlebot3/turtlebot3_navigation/rviz/turtlebot3_navigation1.rviz')
    
    # Define the new topic based on the robot number
    new_topic = f'/tb3_{robot_number}/move_base_simple/goal'
    
    # Read the existing RViz config
    with open(rviz_config_path, 'r') as file:
        rviz_config = file.readlines()
    
    # Update the 2D Nav Goal topic in the config
    with open(rviz_config_path, 'w') as file:
        for line in rviz_config:
            # Look for the "Topic" line under "SetGoal" and update it
            if 'Name: Goal' in line:
                # After finding the SetGoal tool, update the next "Topic" entry
                next_line_index = rviz_config.index(line) + 1
                if 'Topic' in rviz_config[next_line_index]:
                    rviz_config[next_line_index] = f'    Topic: {new_topic}\n'
            file.write(line)
            if '- Class: rviz/SetGoal' in line:
                # After finding the SetGoal tool, update the next "Topic" entry
                next_line_index = rviz_config.index(line) + 1
                if 'Topic' in rviz_config[next_line_index]:
                    rviz_config[next_line_index] = f'    Topic: {new_topic}\n'
            file.write(line)

    print(f'Updated RViz config for robot {robot_number} with 2D Nav Goal topic: {new_topic}')
def send_goal(robot_namespace, x, y, theta):
    # Publisher for the selected robot's goal topic
    goal_topic = f"/{robot_namespace}/move_base_simple/goal"
    goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)

    # Wait for the publisher to connect
    rospy.sleep(1)

    # Create the PoseStamped message
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = "map"
    goal_msg.header.stamp = rospy.Time.now()

    # Set the position
    goal_msg.pose.position.x = x
    goal_msg.pose.position.y = y
    goal_msg.pose.position.z = 0.0

    # Set the orientation (quaternion)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
    goal_msg.pose.orientation.x = quaternion[0]
    goal_msg.pose.orientation.y = quaternion[1]
    goal_msg.pose.orientation.z = quaternion[2]
    goal_msg.pose.orientation.w = quaternion[3]

    # Publish the goal
    goal_pub.publish(goal_msg)
    rospy.loginfo(f"Goal sent to {robot_namespace}: x={x}, y={y}, theta={theta}")

if __name__ == '__main__':
    rospy.init_node("multi_robot_thing")
    rospy.loginfo("Node started")

    # Start the world first
    world_launch_file = "/home/gov_laptop/MultiAgent_ML/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_empty_world.launch"
    world_launch = roslaunch.parent.ROSLaunchParent(uuid, [world_launch_file])
    world_launch.start()
    time.sleep(2)  # Add a delay to ensure proper initialization

    # Launch the multiple robots

    x = int(input("How many robots? "))
    launch_all_robots(x)
    launch_all_slam(x)
    launch_all_map(x)

    launch_all_nav(x)
    launch_rviz()
    done = "False"

    while done != "True":
        robot_name = input("Which robot would you like to control? Enter 0-Max Robots: ")

        goal_topic = "tb3_"+str(robot_name)
        send_goal(goal_topic, 2,2,10)
   
        done = input("Are you done? Enter True or False: ")
    
    rospy.spin()

