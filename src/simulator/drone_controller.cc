#include <functional> 
#include <algorithm> 

#include <gazebo/gazebo.hh> 
#include <gazebo/physics/physics.hh> 
#include <gazebo/common/common.hh> 
#include <ignition/math/Vector3.hh>

#include "gazebo_version_defines.h"

namespace gazebo {
  class DroneController : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
            {
              this->model = _parent;

              minimum_flying_altitude = _sdf->GetElement("minimum_flying_altitude")->Get<double>();

              const auto max_xy_speed = _sdf->GetElement("max_xy_speed")->Get<double>();
              const auto max_z_speed = _sdf->GetElement("max_z_speed")->Get<double>();
              const auto max_z_accel = _sdf->GetElement("max_z_accel")->Get<double>();
              const auto mpc_xy_p = _sdf->GetElement("mpc_xy_p")->Get<double>();
              const auto mpc_xy_d = _sdf->GetElement("mpc_xy_d")->Get<double>();
              const auto mpc_z_p = _sdf->GetElement("mpc_z_p")->Get<double>();
              const auto mpc_z_i = _sdf->GetElement("mpc_z_i")->Get<double>();
              const auto mpc_xy_vel_p = _sdf->GetElement("mpc_xy_vel_p")->Get<double>();
              const auto mpc_xy_vel_i = _sdf->GetElement("mpc_xy_vel_i")->Get<double>();
              const auto mpc_xy_vel_d = _sdf->GetElement("mpc_xy_vel_d")->Get<double>();
              const auto mpc_max_att = _sdf->GetElement("mpc_max_att")->Get<double>();
              const auto mpc_z_vel_p = _sdf->GetElement("mpc_z_vel_p")->Get<double>();
              const auto mpc_z_vel_i = _sdf->GetElement("mpc_z_vel_i")->Get<double>();
              const auto mpc_z_vel_d = _sdf->GetElement("mpc_z_vel_d")->Get<double>();
              const auto mpc_z_vel_i_max = _sdf->GetElement("mpc_z_vel_i_max")->Get<double>();
              const auto mc_att_p = _sdf->GetElement("mc_att_p")->Get<double>();
              const auto mc_att_d = _sdf->GetElement("mc_att_d")->Get<double>();
              const auto mc_yaw_p = _sdf->GetElement("mc_yaw_p")->Get<double>();
              const auto mc_yaw_d = _sdf->GetElement("mc_yaw_d")->Get<double>();
              const auto mc_attrate_p = _sdf->GetElement("mc_attrate_p")->Get<double>();
              const auto mc_attrate_d = _sdf->GetElement("mc_attrate_d")->Get<double>();
              const auto mc_attrate_max = _sdf->GetElement("mc_attrate_max")->Get<double>();
              const auto mc_yawrate_p = _sdf->GetElement("mc_yawrate_p")->Get<double>();
              const auto mc_yawrate_max = _sdf->GetElement("mc_yawrate_max")->Get<double>();

              this->node.reset(new transport::Node());
              node->Init();
              force_pub = node->Advertise<msgs::Vector3d>("~/drone/force", 1);
              pose_pub = node->Advertise<msgs::Pose>("~/drone/pose", 1);
              setpoint_sub = node->Subscribe("~/drone/setpoint", &DroneController::setpointCb, this);
              yaw_sub = node->Subscribe("~/drone/yaw", &DroneController::yawCb, this);


              this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                  std::bind(&DroneController::OnUpdate, this));

              baselink = model->GetLink("base_link");

              xpos_pid = common::PID(mpc_xy_p, 0, mpc_xy_d);
              xpos_pid.SetCmdMax(max_xy_speed);
              xpos_pid.SetCmdMin(-max_xy_speed);
              ypos_pid = common::PID(mpc_xy_p, 0, mpc_xy_d);
              ypos_pid.SetCmdMax(max_xy_speed);
              ypos_pid.SetCmdMin(-max_xy_speed);
              zpos_pid = common::PID(mpc_z_p, mpc_z_i, 0);
              zpos_pid.SetCmdMax(max_z_speed);
              zpos_pid.SetCmdMin(-max_z_speed);

              xvel_pid = common::PID(mpc_xy_vel_p, mpc_xy_vel_i, mpc_xy_vel_d);
              xvel_pid.SetCmdMax(mpc_max_att);
              xvel_pid.SetCmdMin(-mpc_max_att);
              yvel_pid = common::PID(mpc_xy_vel_p, mpc_xy_vel_i, mpc_xy_vel_d);
              yvel_pid.SetCmdMax(mpc_max_att);
              yvel_pid.SetCmdMin(-mpc_max_att);
              zvel_pid = common::PID(mpc_z_vel_p, mpc_z_vel_i, mpc_z_vel_d);
              zvel_pid.SetIMax(mpc_z_vel_i_max);
              zvel_pid.SetIMin(-mpc_z_vel_i_max);
              zvel_pid.SetCmdMax(max_z_accel);
              zvel_pid.SetCmdMin(0);

              roll_pid = common::PID(mc_att_p, 0, mc_att_d);
              roll_pid.SetCmdMax(mc_attrate_max);
              roll_pid.SetCmdMin(-mc_attrate_max);
              pitch_pid = common::PID(mc_att_p, 0, mc_att_d);
              pitch_pid.SetCmdMax(mc_attrate_max);
              pitch_pid.SetCmdMin(-mc_attrate_max);
              yaw_pid = common::PID(mc_yaw_p, 0, mc_yaw_d);
              yaw_pid.SetCmdMax(mc_yawrate_max);
              yaw_pid.SetCmdMin(-mc_yawrate_max);

              rollrate_pid = common::PID(mc_attrate_p, 0, mc_attrate_d);
              pitchrate_pid = common::PID(mc_attrate_p, 0, mc_attrate_d);
              yawrate_pid = common::PID(mc_yawrate_p, 0, 0);

              pos_ref = EXTRACT_POS(WORLD_POSE(baselink));
              
              last_update_time = SIM_TIME(model);

              std::cout << "Drone controller is done setting up" << std::endl;
            }
    public: void OnUpdate()
            {
              auto current_time = SIM_TIME(model);
              const auto dt = current_time - this->last_update_time;
              pose = WORLD_POSE(baselink);
              const auto pos = EXTRACT_POS(pose);
              const auto rpy = EXTRACT_ROT(pose);
              const auto vel = WORLD_LINEAR_VEL(baselink);
              const auto accel = WORLD_LINEAR_ACCEL(baselink);
              const auto rpy_rate = RELATIVE_ANGULAR_ACCEL(baselink);
              const auto altitude = EXTRACT_Z(pos);

              auto force_z = 0.0;
              if (setpoint_recieved) {
                const auto pos_error = pos - pos_ref;
                double vel_x_ref=0.0, vel_y_ref=0.0;
                if (altitude >= minimum_flying_altitude) {
                  vel_x_ref = xpos_pid.Update(EXTRACT_X(pos_error), dt);
                  vel_y_ref = ypos_pid.Update(EXTRACT_Y(pos_error), dt);
                } 
                const auto vel_z_ref = zpos_pid.Update(EXTRACT_Z(pos_error), dt);
                ignition::math::Vector3d vel_ref{vel_x_ref, vel_y_ref, vel_z_ref};
                const auto vel_error = vel - vel_ref;

                const auto xacc_ref = xvel_pid.Update(EXTRACT_X(vel_error), dt);
                const auto yacc_ref = yvel_pid.Update(EXTRACT_Y(vel_error), dt);
                const auto zacc_ref = zvel_pid.Update(EXTRACT_Z(vel_error), dt);


                ignition::math::Vector3d torque;
                if (altitude >= minimum_flying_altitude) {
                  const auto yaw = EXTRACT_Z(rpy);
                  const auto roll_ref  = std::sin(yaw)*xacc_ref - std::cos(yaw)*yacc_ref;
                  const auto pitch_ref = std::cos(yaw)*xacc_ref + std::sin(yaw)*yacc_ref;

                  const auto roll_error = smallestSignedAngle(EXTRACT_X(rpy) - roll_ref);
                  const auto pitch_error = smallestSignedAngle(EXTRACT_Y(rpy) - pitch_ref);
                  const auto yaw_error = smallestSignedAngle(EXTRACT_Z(rpy) - yaw_ref);
                  const auto rollrate_ref = roll_pid.Update(roll_error, dt);
                  const auto pitchrate_ref = pitch_pid.Update(pitch_error, dt);
                  const auto yawrate_ref = yaw_pid.Update(yaw_error, dt);

                  const auto rollrate_error = EXTRACT_X(rpy_rate) - rollrate_ref;
                  const auto pitchrate_error = EXTRACT_Y(rpy_rate) - pitchrate_ref;
                  const auto yawrate_error = EXTRACT_Z(rpy_rate) - yawrate_ref;
                  const auto roll_torque = rollrate_pid.Update(rollrate_error, dt);
                  const auto pitch_torque = pitchrate_pid.Update(pitchrate_error, dt);
                  const auto yaw_torque = yawrate_pid.Update(yawrate_error, dt);

                  torque.X() = roll_torque*EXTRACT_IXX(baselink); 
                  torque.Y() = pitch_torque*EXTRACT_IYY(baselink);
                  torque.Z() = yaw_torque*EXTRACT_IZZ(baselink);
                }

                const auto mass = EXTRACT_MASS(baselink);
                force_z = mass*zacc_ref;
                if (force_z < 0) {
                  force_z = 0.0;
                }

                ignition::math::Vector3d force{0, 0, force_z};

                baselink->AddRelativeForce(force);
                baselink->AddRelativeTorque(torque);

                this->last_update_time = current_time;

              }

              // Visualisation
              ignition::math::Vector3d forces{0,0,force_z};
              msgs::Vector3d force_msg;
              msgs::Set(&force_msg, forces);
              force_pub->Publish(force_msg);

              // publish pose
              msgs::Pose pose_msg;
              msgs::Set(&pose_msg, pose);
              pose_pub->Publish(pose_msg);
            }

    private: double smallestSignedAngle(double angle) const 
             {
               // TODO: replace with more computationally reasonable function
               const auto ssa = std::atan2(std::sin(angle), std::cos(angle));
               return ssa;
             }
  
    private: void setpointCb(ConstVector3dPtr& msg) 
             {
               setpoint_recieved = true;
               pos_ref.X() = msg->x();
               pos_ref.Y() = msg->y();
               pos_ref.Z() = msg->z();
             }

    private: void yawCb(ConstVector3dPtr& msg) 
             {
               yaw_ref = msg->z();
             }
            
    private: bool setpoint_recieved = false;
    private: ignition::math::Pose3d pose;
    private: ignition::math::Vector3d pos_ref;
    private: double yaw_ref = 0.0;

    private: common::Time last_update_time;
    private: physics::LinkPtr baselink;
    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
             
    private: transport::NodePtr node;
    private: transport::SubscriberPtr setpoint_sub;
    private: transport::SubscriberPtr yaw_sub;
    private: transport::PublisherPtr force_pub;
    private: transport::PublisherPtr pose_pub;

    private: common::PID xpos_pid;
    private: common::PID ypos_pid;
    private: common::PID zpos_pid;
    private: common::PID xvel_pid;
    private: common::PID yvel_pid;
    private: common::PID zvel_pid;
    private: common::PID roll_pid;
    private: common::PID pitch_pid;
    private: common::PID yaw_pid;
    private: common::PID rollrate_pid;
    private: common::PID pitchrate_pid;
    private: common::PID yawrate_pid;

    private: double minimum_flying_altitude=1.0;
  };


  GZ_REGISTER_MODEL_PLUGIN(DroneController);
}
