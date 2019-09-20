#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Polygon

# Initialize ROS
rospy.init_node("ai_node")


# Callbacks
boxes = []
def boxesCallback(msg):
    boxes = msg.points

# Setup code
sub = rospy.Subscriber("/perception/boxes", Polygon, boxesCallback)


# Main loop
rospy.loginfo("ai is running")
rate = rospy.Rate(30)
while not rospy.is_shutdown():


    # Rate limiting
    rate.sleep()









