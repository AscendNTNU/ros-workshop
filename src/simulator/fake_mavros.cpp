#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>

class FakeMavros {
  private:
    ros::NodeHandle ros_node;
    ros::Subscriber target_sub;
    ros::Publisher local_position_pub, state_pub;
    ros::ServiceServer arming_srv, mode_srv;
    ros::Timer event_timer;

    gazebo::transport::NodePtr gz_node;
    gazebo::transport::PublisherPtr setpoint_pub, yaw_pub;
    gazebo::transport::SubscriberPtr pose_pub;

    bool data_recieved = false;
    double x=0.0,y=0.0,z=0.0,yaw=0.0;

    geometry_msgs::PoseStamped pose;
    mavros_msgs::State state;
    
    void positionTargetCb(const mavros_msgs::PositionTarget& msg) {
      x = msg.position.x;
      y = msg.position.y;
      z = msg.position.z;
      yaw = static_cast<double>(msg.yaw);

      data_recieved = true;
      ROS_INFO_ONCE("Received position/yaw target");
    }

    void timerPublishEvent(const ros::TimerEvent& event) {
      if (data_recieved) {
        gazebo::msgs::Vector3d setpoint_msg;
        setpoint_msg.set_x(x);
        setpoint_msg.set_y(y);
        setpoint_msg.set_z(z);

        // NOTE: gazebo::msgs doesn't have a nice builtin for doubles
        // so we use a vector3d instead. x,y must to set for message
        // to publish correctly but will be ignored in plugin.
        gazebo::msgs::Vector3d yaw_msg;
        yaw_msg.set_x(0.0);
        yaw_msg.set_y(0.0);
        yaw_msg.set_z(yaw);

        setpoint_pub->Publish(setpoint_msg);
        yaw_pub->Publish(yaw_msg);
      }

      local_position_pub.publish(pose);
      state_pub.publish(state);
    }

    bool armingSrv(mavros_msgs::CommandBool::Request& req, mavros_msgs::CommandBool::Response& res) {
      state.armed = req.value;
      if (state.armed)
        ROS_INFO_NAMED("mavros", "arming");
      else
        ROS_INFO_NAMED("mavros", "disarming");

      return true;
    }

    bool setModeSrv(mavros_msgs::SetMode::Request& req, mavros_msgs::SetMode::Response& res) {
      state.mode = req.custom_mode;
      res.mode_sent = true;
      ROS_INFO_NAMED("mavros", "set mode to %s", state.mode.c_str());
      return true;
    }

    void gzPoseCb(ConstPosePtr& msg) {
      ROS_INFO_ONCE("received pose from gazebo");
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";

      pose.pose.position.x = msg->position().x();
      pose.pose.position.y = msg->position().y();
      pose.pose.position.z = msg->position().z();
      pose.pose.orientation.x = msg->orientation().x();
      pose.pose.orientation.y = msg->orientation().y();
      pose.pose.orientation.z = msg->orientation().z();
      pose.pose.orientation.w = msg->orientation().w();
    }

  public:
    FakeMavros() = delete;
    FakeMavros(const std::string& gz_setpoint_topic, const std::string& gz_yaw_topic, const std::string& gz_pose_topic) 
      : ros_node("drone") {
      gz_node.reset(new gazebo::transport::Node());
      gz_node->Init();
      setpoint_pub = gz_node->Advertise<gazebo::msgs::Vector3d>(gz_setpoint_topic, 1);
      yaw_pub = gz_node->Advertise<gazebo::msgs::Vector3d>(gz_yaw_topic, 1);
      //ROS_INFO("wait for setpoint connection");
      //setpoint_pub->WaitForConnection();
      //ROS_INFO("wait for yaw connection");
      //yaw_pub->WaitForConnection();
      pose_pub = gz_node->Subscribe(gz_pose_topic, &FakeMavros::gzPoseCb, this);

      target_sub = ros_node.subscribe("setpoint", 1, &FakeMavros::positionTargetCb, this);
      local_position_pub = ros_node.advertise<geometry_msgs::PoseStamped>("pose", 1);
      state_pub = ros_node.advertise<mavros_msgs::State>("state", 1);
      arming_srv = ros_node.advertiseService("cmd/arming", &FakeMavros::armingSrv, this);
      mode_srv = ros_node.advertiseService("set_mode", &FakeMavros::setModeSrv, this);

      state.armed = false;
      state.guided = false;
      state.mode = "";

      ros::Rate publish_rate(30.0);
      event_timer = ros_node.createTimer(publish_rate, &FakeMavros::timerPublishEvent, this);
    }
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "fakemavros");
  gazebo::client::setup(argc, argv);
  //gazebo::transport::init();
  //gazebo::transport::run();

  std::string gz_setpoint_topic, gz_yaw_topic, gz_pose_topic;
  ros::NodeHandle n("~");
  n.getParam("gz_setpoint_topic", gz_setpoint_topic);
  n.getParam("gz_yaw_topic", gz_yaw_topic);
  n.getParam("gz_pose_topic", gz_pose_topic);
  
  ROS_FATAL_COND(gz_setpoint_topic.empty(), "gz_setpoint_topic not given");
  ROS_FATAL_COND(gz_yaw_topic.empty(), "gz_yaw_topic not given");
  ROS_FATAL_COND(gz_pose_topic.empty(), "gz_pose_topic not given");
  if (gz_setpoint_topic.empty() || gz_yaw_topic.empty() || gz_pose_topic.empty()) {
    ROS_FATAL("Namespace is %s", n.getNamespace().c_str());
    gazebo::client::shutdown();
    return 1;
  }

  FakeMavros mavros{gz_setpoint_topic, gz_yaw_topic, gz_pose_topic};

  ros::spin();

  gazebo::client::shutdown();
  return 0;
}
