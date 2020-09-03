#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

# Write your service functions here
def handle_take_off_command(request):
    response = TriggerResponse()	
    response.success = True
    response.message = "All ok!"
    rospy.loginfo("Handling take off!")

    return response

def main():

    # Initialize ros
    rospy.init_node("control_node")

    # Write service handlers here
    take_off_service = rospy.Service("/control/take_off", Trigger, handle_take_off_command)

    # Main loop
    rospy.loginfo("Control is running")
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():

        # Rate limiting
        rate.sleep()

if __name__ == '__main__':
    # This is just here so ROS doesn't throw a warning when we close the nodes in the terminal
    try:
        main()
    except rospy.ROSInterruptException:
        pass




