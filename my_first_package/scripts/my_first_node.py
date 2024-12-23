#!/usr/bin/env python3
import rospy


if __name__ == '__main__':
    rospy.init_node("Poopy")

    rospy.loginfo("This is info")
    rospy.logwarn("This is a warning not to test me")
    rospy.logerr("You made an error")

    rospy.sleep(1.0)

    rospy.loginfo("This will end")

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("Sup")
        rate.sleep()

