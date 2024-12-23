#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import roslaunch
import time

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

def launch_multiple_files():
    x = int(input("How many robots? "))

    for i in range(x):
        launch_file = "/home/gov_laptop/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/singleTurtlebot.launch"
        # Launch arguments should be passed as a list of strings, not tuples
        launch_args = ['first_tb3:=tb3_' + str(i), 'first_tb3_x_pos:=' + str(i) + ".0"]

        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, launch_args)])
        launch.start()
        rospy.loginfo(f"TurtleBot tb3_{i} launched")
        time.sleep(2)  # Add a delay to ensure proper initialization


if __name__ == '__main__':
    rospy.init_node("multi_robot_thing")
    rospy.loginfo("Node started")

    # Start the world first
    world_launch_file = "/home/gov_laptop/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/world.launch"
    world_launch = roslaunch.parent.ROSLaunchParent(uuid, [world_launch_file])
    world_launch.start()
    time.sleep(2)  # Add a delay to ensure proper initialization

    # Launch the multiple robots
    launch_multiple_files()

    done = "False"

    while done != "True":
        robot_name = input("Which robot would you like to control? Enter 0-Max Robots: ")
        pubV = rospy.Publisher("/tb3_" + str(robot_name) + "/cmd_vel", Twist, queue_size=10)
        
        x = float(input("x coordinate? "))
        y = float(input("y coordinate? "))

        twist = Twist()
        twist.linear.x = x
        twist.angular.z = y  # Changed from angular.y to angular.z for TurtleBot movement

        pubV.publish(twist)
        done = input("Are you done? Enter True or False: ")
    
    rospy.spin()
