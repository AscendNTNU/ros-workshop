#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Polygon, PoseStamped, Point

# Initialize ROS
rospy.init_node("ai_node")


# Callbacks
boxes = []
def boxesCallback(msg):
    global boxes
    boxes = msg.points

pos = None
def poseCallback(msg):
    global pos
    pos = msg.pose.position

# Setup code
boxsub = rospy.Subscriber("/perception/boxes", Polygon, boxesCallback)
possub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseCallback)

pub = rospy.Publisher("/control/position_setpoint", Point, queue_size=1)

# Main loop
rospy.loginfo("ai is running")
rate = rospy.Rate(30)
while not rospy.is_shutdown():
    if pos != None:
        closest, distance2 = -1, float("inf")
        for i, box in enumerate(boxes):
            d2 = (box.x-pos.x)**2 + (box.y-pos.y)**2
            if d2 < distance2:
                closest = i
                distance2 = d2

        if closest != -1:
            msg = Point(
                boxes[closest].x,
                boxes[closest].y,
                boxes[closest].z)

            pub.publish(msg)

    # Rate limiting
    rate.sleep()









