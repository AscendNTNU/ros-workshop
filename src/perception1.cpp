#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

// Define callbacks here
void callback(const geometry_msgs::PoseArray& msg) {
  ROS_INFO("Message recieved"); // debug print
}


int main(int argc, char** argv){
    // Initialise ROS
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle n;

    // Set up subscribers/publishers here
    ros::Subscriber sub = n.subscribe("/simulator/boxes", 1, callback);

    // Main loop
    ROS_INFO("perception is running");
    ros::Rate rate(30);
    while (ros::ok()) {


      rate.sleep();
      ros::spinOnce();
    }

    return 0;
}


