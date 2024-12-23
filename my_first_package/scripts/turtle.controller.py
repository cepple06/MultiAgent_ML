#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

previous_x = 0


    
def call_set_pen_service(r, g, b, width ,off):
    try:
        set_pen = rospy.ServiceProxy("/turtle1/set_pen",SetPen)
        response =set_pen(r,g,b,width,off)
        #rospy.loginfo(response)
        
    except rospy.ServiceException as e:
        rospy.logwarn("Service Cannot set pen")
        rospy.logwarn(e)


def pose_callback(msg: Pose):
    cmd = Twist()
    if msg.x  > 9 or msg.x <2:
        cmd.angular.z = 3.0
    elif msg.y >9 or msg.y<2:
        cmd.angular.z = 3.0
    else:
        cmd.angular.z = 0
    cmd.linear.x = cmd.linear.x +5.0
    pub.publish(cmd)

    global previous_x
    if msg.x >= 5.5 and previous_x < 5.5:
        call_set_pen_service(255,0,0,3,0)
    elif msg.x <=5.5 and previous_x>5.5:
        call_set_pen_service(0,255,0,3,0)
    previous_x = msg.x


if __name__ == '__main__':
    rospy.init_node("Turtle_controller")
    rospy.wait_for_service("/turtle1/set_pen")
    #call_set_pen_service(255,0,0,3,0)
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/turtle1/pose",Pose, callback=pose_callback)
    rospy.loginfo("Node started")

    rospy.spin()