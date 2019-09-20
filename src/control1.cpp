#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/PositionTarget.h>

geometry_msgs::Point::ConstPtr target;
void targetCallback(geometry_msgs::Point::ConstPtr msg){
  target = msg;
}

int main(int argc, char** argv){
    // Initialise ROS
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n;

    // Set up subscribers/publishers here
    auto sub = n.subscribe("control/position_setpoint", 1, targetCallback);
    auto pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);

    mavros_msgs::PositionTarget msg;

    // Main loop
    ROS_INFO("control is running");
    ros::Rate rate(30);
    while (ros::ok()) {
      if (target != nullptr) {
        msg.position = *target;


        pub.publish(msg);
      }

      rate.sleep();
      ros::spinOnce();
    }

    return 0;
}


