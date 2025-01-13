#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import roslaunch
import time
import rospy
import signal



uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

def launch_all_robots(x):
    for i in range(x):
        launch_file = "/home/airsimdemo/MultiAgent_ML/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/single_gmapping_turtlebot3.launch" #Location of launching a single robot that works with others
        launch_args = ['first_tb3:=tb3_' + str(i),'first_tb3_x_pos:=' + str(i-2) + ".0",'first_tb3_y_pos:=0.5','first_tb3_yaw:=0.0']#This handles the name and spawing location and yaw
        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, launch_args)])
        launch.start()
        rospy.loginfo(f"TurtleBot tb3_{i} launched")
        time.sleep(2)  # Add a delay to ensure proper initialization

def launch_navigation(x):
    for i in range(x):
        launch_file = "/home/airsimdemo/MultiAgent_ML/catkin_ws/MultiAgent_ML-main/turtlebot3/turtlebot3_navigation/launch/turtlebot3_navigation.launch"#Location of the turtlebot3_navigation that makes it so rviz can use move_base and amcl
        if i == 0:
            launch_args = ['open_rviz:=true','cmd_vel_topic:=/tb3_' + str(i) + '/cmd_vel','scan_topic:=tb3_' + str(i) + '/scan','first_tb3:=tb3_' + str(i),'first_tb3_x_pos:=' + str(i-2) + ".0",'first_tb3_y_pos:=0.5','first_tb3_yaw:=0.0']
            #The reason there is an if statement is that it would open rviz twice but now it only opens once.
        else:
            launch_args = ['open_rviz:=false','cmd_vel_topic:=tb3_' + str(i) + '/cmd_vel','scan_topic:=tb3_' + str(i) + '/scan','first_tb3:=tb3_' + str(i),'first_tb3_x_pos:=' + str(i-2) + ".0",'first_tb3_y_pos:=0.5','first_tb3_yaw:=0.0']
            #The arguments have to match the original robot name and the topics it would be published to.
        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, launch_args)])
        launch.start()
        rospy.loginfo(f"TurtleBot tb3_{i} launched")
        time.sleep(3)  # Add a delay to ensure proper initialization

def launch_slam(x):
    for i in range(x):
        launch_file = "/home/airsimdemo/MultiAgent_ML/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/multi_turtlebot3_slam.launch"#this launches the scanner and configures the robot in the tf tree.
        launch_args = ['ns:=tb3_' + str(i)]#make sure this is the name of the robot
        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file,launch_args)])
        launch.start()
        rospy.loginfo(f"TurtleBot tb3_{i} launched")
        time.sleep(3)  # Add a delay to ensure proper initialization

def launch_map_merge(x):
    for i in range(x):
        launch_file = "/home/airsimdemo/MultiAgent_ML/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/single_map_merge.launch"
        launch_args = ['first_tb3:=tb3_' + str(i),'first_tb3_x_pos:=' + str(i) + ".0",'first_tb3_y_pos:=0.5'] #These spawn coordinates are a little weird and dont seem to work too well but just make them match to make sure it wont break.
        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file,launch_args)])
        launch.start()
        rospy.loginfo(f"TurtleBot tb3_{i} launched")
        time.sleep(3)  # Add a delay to ensure proper initialization



if __name__ == '__main__':
    rospy.init_node("multi_robot_thing")
   
    rospy.loginfo("Node started")
    # Start the world first
    world_launch_file = "/home/airsimdemo/MultiAgent_ML/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_pentagon.launch"
    world_launch = roslaunch.parent.ROSLaunchParent(uuid, [world_launch_file])
    world_launch.start()
    time.sleep(2)  # Add a delay to ensure proper initialization
    global x 
    x = int(input("How many robots? "))
    launch_all_robots(x) #launches the robots
    launch_slam(x)#launches the scanner and configures the tf tree
    launch_map_merge(x)# configures the maps created by the multiple robots and links them to one map
    launch_navigation(x)#launches the navigation for rviz so that move_base and the amcl can work with the multiple robots.
    rospy.spin()
