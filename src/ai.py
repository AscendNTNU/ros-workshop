#!/usr/bin/env python

import rospy


# Initialize ROS
rospy.init_node("ai_node")


# Set up subscribers here



# Main loop
rospy.loginfo("ai is running")
rate = rospy.Rate(30)
while not rospy.is_shutdown():


    # Rate limiting
    rate.sleep()









