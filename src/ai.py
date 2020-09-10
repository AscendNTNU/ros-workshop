#!/usr/bin/env python3

import rospy

# Set up callbacks for subscribers here



# Set up service calls here



# Initialize ros
rospy.init_node("ai_node")


# Set up subscribers here



# Main loop
rospy.loginfo("AI is running")
rate = rospy.Rate(30)
while not rospy.is_shutdown():


    # Rate limiting
    rate.sleep()









