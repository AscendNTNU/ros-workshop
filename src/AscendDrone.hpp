#ifndef ASCEND_DRONE_HPP
#define ASCEND_DRONE_HPP
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <array>

class ControlDrone {
private:
	ros::NodeHandle nh_;
	ros::Publisher posPub_;
	ros::Subscriber posSub_;
	mavros_msgs::PositionTarget targetPos_;
	geometry_msgs::PoseStamped currPos_;
	ros::Timer timer_;
	void timerCB(const ros::TimerEvent& e);
	void posCB(const geometry_msgs::PoseStamped& msg);
public:
	ControlDrone();
	void takeoff();
	void setTarget(float x, float y, float z);
	void land();
};

class PerceptionDrone {
private:
	ros::NodeHandle nh_;
	ros::Subscriber posSub_;
	geometry_msgs::PoseStamped lastPos_;
	void posCB(geometry_msgs::PoseStamped::ConstPtr ptr);
public:
	PerceptionDrone();
	std::array<float, 3> getPosition();
};
#endif
