import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
import threading

def move_robot(robot_name, x, y):
    rospy.loginfo(f"Starting to move {robot_name}")
    
    # Start the motion client
    client = actionlib.SimpleActionClient(robot_name+"/move_base", MoveBaseAction)
    rospy.loginfo(f"Wait for {robot_name} action server to come up")
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    target_frame = robot_name+"/odom"
    goal.target_pose.header.frame_id = target_frame
    goal.target_pose.header.stamp = rospy.Time.now()

    # Sending goal to the robot
    rospy.logwarn(f"Sending new goal point to {robot_name}.")
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1
    
    client.send_goal(goal)
    client.wait_for_result()
    
    rospy.loginfo(f"{robot_name} arrived at the destination.")

def main():
    rospy.init_node("move_robots_node")

    # Define the robots and their respective goal positions
    robots = [
        ("tb3_0", -1, -2),
        ("tb3_1", -1, 1.5),
        ("tb3_2", -0.5, 0)
    ]
    
    # Create a thread for each robot
    threads = []
    for robot_name, x, y in robots:
        t = threading.Thread(target=move_robot, args=(robot_name, x, y))
        threads.append(t)
        t.start()

    # Wait for all threads to complete
    for t in threads:
        t.join()

if __name__ == "__main__":
    main()