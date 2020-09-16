#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray, Polygon

# Set up callback here
boxes_message = PoseArray()
def callback(message):
    global boxes_message
    boxes_message = message

def main():

    # Initialize ROS
    rospy.init_node("perception_node")


    # Set up subscriber here 
    rospy.Subscriber("/simulator/boxes", PoseArray, callback)

    # Set up publisher here
    publisher = rospy.Publisher("/perception/boxes", Polygon, queue_size=1)

    # Main loop
    rospy.loginfo("Perception is running")
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():

        polygon_message = Polygon()
        for pose in boxes_message.poses:
            point = pose.position
            polygon_message.points.append(point)

        publisher.publish(polygon_message)


        # Rate limiting
        rate.sleep()

if __name__ == '__main__':
    # This is just here so ROS doesn't throw a warning when we close the nodes in the terminal
    try:
        main()
    except rospy.ROSInterruptException:
        pass


