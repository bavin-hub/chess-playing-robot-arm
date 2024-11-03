#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__=="__main__":
    rospy.init_node("simple_pub")
    pub = rospy.Publisher("greeting", String, queue_size=10)

    rate = rospy.Rate(2) # in hz

    while not rospy.is_shutdown():
        msg = String()
        msg.data = "hello world"
        pub.publish(msg)
        rate.sleep()

