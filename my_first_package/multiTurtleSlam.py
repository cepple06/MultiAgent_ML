#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import roslaunch
import time
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""

class GotoPoint():
    def __init__(self, publisher, base_frame):
        #rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        #rospy.on_shutdown(self.shutdown)
        self.cmd_vel = publisher
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.base_frame = base_frame

        try:
            self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between odom and " + self.base_frame)
            rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        (goal_x, goal_y, goal_z) = self.getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance

        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            move_cmd.angular.z = angular_speed * path_angle-rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(linear_speed * distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        (position, rotation) = self.get_odom()

        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            r.sleep()


        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())

    def getkey(self):
        x, y, z = input("| x | y | z |\n").split()
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

def launch_all_robots(x):
    for i in range(x):
        launch_file = "/home/airsimdemo/MultiAgent_ML/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/single_gmapping_turtlebot3.launch"
        launch_args = ['first_tb3:=tb3_' + str(i),'first_tb3_x_pos:=' + str(i) + ".0"]
        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, launch_args)])
        launch.start()
        rospy.loginfo(f"TurtleBot tb3_{i} launched")
        time.sleep(2)  # Add a delay to ensure proper initialization

def launch_slam(x):
    for i in range(x):
        launch_file = "/home/airsimdemo/MultiAgent_ML/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/multi_turtlebot3_slam.launch"
        launch_args = ['ns:=tb3_' + str(i)]
        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file,launch_args)])
        launch.start()
        rospy.loginfo(f"TurtleBot tb3_{i} launched")
        time.sleep(2)  # Add a delay to ensure proper initialization

def launch_map_merge(x):
    for i in range(x):
        launch_file = "/home/airsimdemo/MultiAgent_ML/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/single_map_merge.launch"
        launch_args = ['first_tb3:=tb3_' + str(i),'first_tb3_x_pos:=' + str(i) + ".0"]
        launch = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file,launch_args)])
        launch.start()
        rospy.loginfo(f"TurtleBot tb3_{i} launched")
        time.sleep(2)  # Add a delay to ensure proper initialization



if __name__ == '__main__':
    rospy.init_node("multi_robot_thing")
   # rospy.on_shutdown(GotoPoint.shutdown)
    rospy.loginfo("Node started")

    # Start the world first
    world_launch_file = "/home/airsimdemo/MultiAgent_ML/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/emptyworld.launch"
    world_launch = roslaunch.parent.ROSLaunchParent(uuid, [world_launch_file])
    world_launch.start()
    time.sleep(2)  # Add a delay to ensure proper initialization
    global x 
    x = int(input("How many robots? "))
    # Launch the multiple robots
    launch_all_robots(x)
    launch_slam(x)
    launch_map_merge(x)
    done = "False"

    while done != "True":
        robot_name = input("Which robot would you like to control? Enter 0-Max Robots: ")
        pubV = rospy.Publisher("/tb3_" + str(robot_name) + "/cmd_vel", Twist, queue_size=10)
        base_frame = "tb3_" + str(robot_name) + "/base_footprint"  # Set the specific base_frame for the chosen robot
      #  GotoPoint(pubV, base_frame) 

    rospy.spin()
