#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Polygon.h>

// Define callbacks here
geometry_msgs::PoseArray boxmsg;
void callback(const geometry_msgs::PoseArray& msg) {
  boxmsg = msg;
}


int main(int argc, char** argv){
    // Initialise ROS
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle n;

    // Set up subscribers/publishers here
    ros::Subscriber sub = n.subscribe("/simulator/boxes", 1, callback);
    ros::Publisher pub = n.advertise<geometry_msgs::Polygon>("perception/boxes", 1);

    geometry_msgs::Polygon polygonmsg;

    // Main loop
    ROS_INFO("perception is running");
    ros::Rate rate(30);
    while (ros::ok()) {
      polygonmsg.points.clear();
      for (const auto& pose : boxmsg.poses) {
        geometry_msgs::Point32 p;
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.z = pose.position.z;

        polygonmsg.points.push_back(p);
      }

      pub.publish(polygonmsg);

      rate.sleep();
      ros::spinOnce();
    }

    return 0;
}


