#include <ros/ros.h>


int main(int argc, char** argv){
    // Initialise ROS
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n;

    // Set up subscribers/publishers here
    


    // Main loop
    ROS_INFO("control is running");
    ros::Rate rate(30);
    while (ros::ok()) {


      rate.sleep();
      ros::spinOnce();
    }

    return 0;
}


