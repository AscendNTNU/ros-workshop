#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Polygon, PoseStamped, Point
from std_srvs.srv import Trigger, TriggerResponse
from ros_workshop.srv import FlyCommand, FlyCommandResponse

# Set up callback here
boxes = [] 
boxes_to_explore = []
exploring = True

def boxesCallback(message):
    global boxes, boxes_to_explore
    boxes = message.points
    if len(boxes_to_explore) == 0 and exploring:
        boxes_to_explore = boxes

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

def find_closest_box_index(drone_position, boxes_to_explore):
    closest, min_distance_squared = -1, float("inf")
    for i, box in enumerate(boxes_to_explore):
        distance_squared = (box.x - drone_position.x)**2 + (box.y - drone_position.y)**2
        if distance_squared < min_distance_squared:
            closest = i
            min_distance_squared = distance_squared
    
    return closest 

def is_over_target(drone_position, target):
    return math.sqrt((target.x - drone_position.x)**2 + (target.y - drone_position.y)**2) < 0.1

def main():
    global boxes_to_explore, drone_position, exploring

    # Initialize ros
    rospy.init_node("ai_node")

    # Set up subscribers here
    rospy.Subscriber("/perception/boxes", Polygon, boxesCallback)
    rospy.Subscriber("/drone/pose", PoseStamped, poseCallback)

    # Main loop
    rospy.loginfo("AI is running")
    sendTakeOffCommand()
    rate = rospy.Rate(30)
    
    target = None
    current_box_target_index = None
    
    while not rospy.is_shutdown():
        if drone_position.z >= 2.5 and boxes_to_explore:
            if current_box_target_index == None:
                current_box_target_index = find_closest_box_index(drone_position, boxes_to_explore)
                target = boxes_to_explore[current_box_target_index]
                sendFlyCommand(target.x, target.y)
            else:
                if is_over_target(drone_position, boxes_to_explore[current_box_target_index]):
                    if len(boxes_to_explore) == 1:
                        rospy.loginfo("Done!")
                        exploring = False

                    del boxes_to_explore[current_box_target_index]
                    current_box_target_index = None

        # Rate limiting
        rate.sleep()

if __name__ == '__main__':
    # This is just here so ROS doesn't throw a warning when we close the nodes in the terminal
    try:
        main()
    except rospy.ROSInterruptException:
        pass




