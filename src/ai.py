#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Polygon, PoseStamped, Point
from std_srvs.srv import Trigger, TriggerResponse
from ros_workshop.srv import FlyCommand, FlyCommandResponse

# Set up callback here
boxes = Polygon()
def boxesCallback(message):
    global boxes
    boxes = message

drone_position = Point()
def poseCallback(message):
    global drone_position 
    drone_position = message.pose.position

# Set up service functions here
def sendTakeOffCommand():
    take_off_service_topic = "/control/take_off"
    # In case the Control node isn't ready yet, we wait until it is ready to receive commands.
    rospy.wait_for_service(take_off_service_topic) 
    #  We want to wrap the service call in a try/catch since it can fail and raise a ServiceException
    try:
        # Think of this as creating a take off function we can call
        take_off = rospy.ServiceProxy(take_off_service_topic, Trigger)
        # We then call this function and get the response from the control node
        take_off_response = take_off()
            
        # Do something with the response, for example check the status flag 
        if take_off_response.success:
            rospy.loginfo("Take off successfully sent to Control node! Got response: %s", take_off_response.message)
        else:
            rospy.logerr("Oh no! The Control node responded with an error: %s", take_off_response.message)

    except rospy.ServiceException as exception:
        print("Service class failed: %s" %exception)

def sendFlyCommand(x, y):
    fly_service_topic = "/control/fly"
    # In case the Control node isn't ready yet, we wait until it is ready to receive commands.
    rospy.wait_for_service(fly_service_topic) 
    #  We want to wrap the service call in a try/catch since it can fail and raise a ServiceException
    try:
        # Think of this as creating a fly function we can call
        fly = rospy.ServiceProxy(fly_service_topic, FlyCommand)
        # We then call this function and get the response from the control node
        fly_command_response = fly(x, y)
            
        # Do something with the response, for example check the status flag 
        if fly_command_response.success:
            rospy.loginfo("Fly to x, y successfully sent to Control node! Got response: %s", fly_command_response.message)
        else:
            rospy.logerr("Oh no! The Control node responded with an error: %s", fly_command_response.message)

    except rospy.ServiceException as exception:
        print("Service class failed: %s" %exception)

sent_command = False

def main():
    global sent_command

    # Initialize ros
    rospy.init_node("ai_node")


    # Set up subscribers here
    rospy.Subscriber("/perception/boxes", Polygon, boxesCallback)
    rospy.Subscriber("/drone/pose", PoseStamped, poseCallback)

    # Main loop
    rospy.loginfo("AI is running")
    sendTakeOffCommand()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if drone_position.z >= 2.5 and not sent_command:
            sendFlyCommand(5, 5)
            sent_command = True

        # Rate limiting
        rate.sleep()

if __name__ == '__main__':
    # This is just here so ROS doesn't throw a warning when we close the nodes in the terminal
    try:
        main()
    except rospy.ROSInterruptException:
        pass




