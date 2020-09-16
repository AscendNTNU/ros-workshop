#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import PositionTarget
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from ros_workshop.srv import FlyCommand, FlyCommandRequest, FlyCommandResponse


# Callback for subscriber of drone position
drone_position = Point()
def poseCallback(message):
    global drone_position 
    got_drone_position = True
    drone_position = message.pose.position


# Functions for controling the drone
setpoint = PositionTarget() 

# Write your service function for take off here
def handleTakeOffRequest(request):
    response = TriggerResponse()	
    
    rospy.loginfo("Handling take off request")
    # If the drone is at ground
    if drone_position.z <= 0.1:
        response.success = True
        response.message = "All ok!"
    
        # Set the take off position to 3 m in air 
        setpoint.position.z = 3.0
    else:
        response.success = False 
        response.message = "Seems like the drone is already flying!"

    return response

def handleFlyRequest(request):
    response = FlyCommandResponse()	
    rospy.loginfo("Handling fly request to %s, %s!", request.x, request.y)
    if drone_position.z <= 0.1:
        response.success = False
        response.messsage = "Seems like the drone hasn't taken off yet"
    elif setpoint.position.x == request.x and setpoint.position.y == request.y:
        response.success = False
        response.messsage = "We are already flying to this setpoint!"
    else:
        response.success = True
        response.message = "All ok!"
        setpoint.position.x = request.x
        setpoint.position.y = request.y
        setpoint.position.z = 3.0 

    return response


def main():

    # Initialize ros
    rospy.init_node("control_node")

    # Set up drone pose subscriber
    rospy.Subscriber("/drone/pose", PoseStamped, poseCallback)

    # Set up setpoint publisher
    setpoint_publisher = rospy.Publisher("/drone/setpoint", PositionTarget, queue_size = 1)

    # Write service handlers here
    take_off_service = rospy.Service("/control/take_off", Trigger, handleTakeOffRequest)
    fly_command_service = rospy.Service("/control/fly", FlyCommand, handleFlyRequest)
    
    # Main loop
    rospy.loginfo("Control is running")
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        setpoint_publisher.publish(setpoint)

        # Rate limiting
        rate.sleep()

if __name__ == '__main__':
    # This is just here so ROS doesn't throw a warning when we close the nodes in the terminal
    try:
        main()
    except rospy.ROSInterruptException:
        pass




