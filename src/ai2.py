#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Polygon, PoseStamped

# Initialize ROS
rospy.init_node("ai_node")


# Callbacks
boxes = []
def boxesCallback(msg):
    global boxes
    boxes = msg.points

pos = PoseStamped().pose.position
def poseCallback(msg):
    global pos
    pos = msg.pose.position
    rospy.loginfo(pos)

# Setup code
boxsub = rospy.Subscriber("/perception/boxes", Polygon, boxesCallback)
possub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseCallback)

# Main loop
rospy.loginfo("ai is running")
rate = rospy.Rate(30)
while not rospy.is_shutdown():


    # Rate limiting
    rate.sleep()









