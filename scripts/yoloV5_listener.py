#!/usr/bin/env python
import rospy
from amov_gimbal_sdk_ros.msg import border_coordinates

def callback(data):
    rospy.loginfo(data.top_left_x + data.top_left_y + data.height + data.width)

def listener():
    rospy.init_node('listener',anonymous=True)

    rospy.Subscriber("border",border_coordinates,callback)

    rospy.spin()

if __name__=='__main__':
    listener()