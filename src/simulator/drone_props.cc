#include <gazebo/gazebo.hh> 
#include <gazebo/physics/physics.hh> 
#include <gazebo/common/common.hh> 
#include <ignition/math/Vector3.hh>

#include "gazebo_version_defines.h"

namespace gazebo {
  class DronePropsSpinner : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
            {
              this->model = _parent;
              joints.push_back(model->GetJoint("rotor_0_joint"));
              joints.push_back(model->GetJoint("rotor_1_joint"));
              joints.push_back(model->GetJoint("rotor_2_joint"));
              joints.push_back(model->GetJoint("rotor_3_joint"));

              max_prop_speed = _sdf->GetElement("max_prop_speed")->Get<double>();

              for (auto jointptr : joints) {
                jointptr->SetVelocityLimit(0, max_prop_speed);
              }

              force_z_filter.SetWindowSize(150);

              this->node.reset(new transport::Node());
              node->Init();
              this->force_sub = node->Subscribe("~/drone/force", &DronePropsSpinner::forceCb, this);
              this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                  std::bind(&DronePropsSpinner::OnUpdate, this));

            }

    public: void OnUpdate()
            {
              force_z_filter.Update(force.Z());
              joints[0]->SetVelocity(0, force_z_filter.Get());
              joints[1]->SetVelocity(0, force_z_filter.Get());
              joints[2]->SetVelocity(0, -force_z_filter.Get());
              joints[3]->SetVelocity(0, -force_z_filter.Get());
            }

    private: void forceCb(ConstVector3dPtr& msg) {
               force.X() = msg->x();
               force.Y() = msg->y();
               force.Z() = msg->z();
             }

    private: physics::ModelPtr model;
    private: physics::Joint_V joints;
    private: event::ConnectionPtr updateConnection;

    private: transport::NodePtr node;
    private: transport::SubscriberPtr force_sub;

    private: double max_prop_speed=0.0;
    private: ignition::math::Vector3d force;
    private: common::MovingWindowFilter<double> force_z_filter;
  };

  GZ_REGISTER_MODEL_PLUGIN(DronePropsSpinner);
}
