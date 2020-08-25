#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Polygon, PoseStamped, Point

# Set up callback here
boxes = Polygon()
def boxesCallback(message):
    global boxes
    boxes = message


drone_position = Point()
def poseCallback(message):
    global drone_position 
    drone_position = message.pose.position


def main():

    # Initialize ros
    rospy.init_node("ai_node")


    # Set up subscribers here
    rospy.Subscriber("/perception/boxes", Polygon, boxesCallback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseCallback)

    # Set up publisher here


    # Main loop
    rospy.loginfo("AI is running")
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rospy.loginfo(boxes)


        # Rate limiting
        rate.sleep()

if __name__ == '__main__':
    # This is just here so ROS doesn't throw a warning when we close the nodes in the terminal
    try:
        main()
    except rospy.ROSInterruptException:
        pass




