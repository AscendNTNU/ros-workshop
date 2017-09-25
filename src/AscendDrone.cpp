#include "AscendDrone.hpp"

namespace {
#define IGNORE_PX (1 << 0)  // Position ignore flags
#define IGNORE_PY (1 << 1)
#define IGNORE_PZ (1 << 2)
#define IGNORE_VX (1 << 3)  // Velocity vector ignore flags
#define IGNORE_VY (1 << 4)
#define IGNORE_VZ (1 << 5)
#define IGNORE_AFX (1 << 6) // Acceleration/Force vector ignore flags
#define IGNORE_AFY (1 << 7)
#define IGNORE_AFZ (1 << 8)
#define FORCE (1 << 9)  // Force in af vector flag
#define IGNORE_YAW (1 << 10)
#define IGNORE_YAW_RATE (1 << 11)
#define SETPOINT_TYPE_TAKEOFF 0x1000
#define SETPOINT_TYPE_LAND 0x2000
#define SETPOINT_TYPE_LOITER 0x3000
#define SETPOINT_TYPE_IDLE 0x4000

constexpr uint16_t default_mask = IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE;
};

ControlDrone::ControlDrone() {
    assert(ros::isInitialized());
    posPub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    posSub_ = nh_.subscribe("mavros/local_position/pose", 10, &ControlDrone::posCB, this);  
    timer_ = nh_.createTimer(ros::Duration(0.03), &ControlDrone::timerCB, this);
    targetPos_.type_mask = default_mask;
}

void ControlDrone::posCB(const geometry_msgs::PoseStamped& msg) {
    currPos_ = msg; 
}

void ControlDrone::timerCB(const ros::TimerEvent& e) {
    posPub_.publish(targetPos_);    
}

void ControlDrone::takeoff() {
    targetPos_.type_mask = default_mask | SETPOINT_TYPE_TAKEOFF;
    targetPos_.position.x = currPos_.pose.position.x;
    targetPos_.position.y = currPos_.pose.position.y;
    targetPos_.position.z = currPos_.pose.position.z;
}

void ControlDrone::setTarget(float x, float y, float z) {
    targetPos_.type_mask = default_mask;
    targetPos_.position.x = x;
    targetPos_.position.y = y;
    targetPos_.position.z = z;
}

void ControlDrone::land() {
    targetPos_.type_mask = default_mask | SETPOINT_TYPE_LAND;
    targetPos_.position.x = currPos_.pose.position.x;
    targetPos_.position.y = currPos_.pose.position.y;
    targetPos_.position.z = currPos_.pose.position.z;
}

PerceptionDrone::PerceptionDrone() {
    assert(ros::isInitialized());
    posSub_ = nh_.subscribe("mavros/local_position/pose", 10, &PerceptionDrone::posCB, this);
}

void PerceptionDrone::posCB(geometry_msgs::PoseStamped::ConstPtr ptr) {
    lastPos_ = *ptr;
}

std::array<float, 3> PerceptionDrone::getPosition() {
    if(ros::Time::now() - lastPos_.header.stamp > ros::Duration(1)) {
        ROS_ERROR("Not recieving position");
    }
    std::array<float, 3> tmp;
    tmp[0] = lastPos_.pose.position.x;
    tmp[1] = lastPos_.pose.position.y;
    tmp[2] = lastPos_.pose.position.z;
    return tmp;
}
