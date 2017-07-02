#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <boost/algorithm/string/replace.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

namespace igm = ignition::math;
namespace gzp = gazebo::physics;
namespace gzt = gazebo::transport;

namespace gazebo
{
  class ModelServo : public ModelPlugin
  {
    private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;

    gazebo::transport::NodePtr node;
    //gzt::node snode;
    std::string topic;

    public:
    double     Torque;
    igm::Angle Reference;
    gazebo::transport::SubscriberPtr sub;

    public:

    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
    {
      // Store the pointer to the model
      model = _parent;
      printf("Hello Servo Motor : AX-12A\n");
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelServo::OnUpdate, this, _1));

      node = gzt::NodePtr(new gzt::Node());
      node->Init();
      std::cout << "Servo Node Initialized " << std::endl;

      topic = sdf->Get<std::string>("topic");
      sub = node->Subscribe(topic,&ModelServo::OnTopicReception,this);
      std::cout << "Subscribed to topic " << topic << std::endl;


      Torque = 0;
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      model->GetJoint("j_Body_Wheel")->SetForce(0,Torque);
    }

    void OnTopicReception(ConstIntPtr& msg)
    {
      std::cout << "OnTopicReception>";
      Torque = (double)msg->data();
      Torque /=(100000);
      std::cout << "Message> Torque set to : " << Torque << std::endl;
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelServo)
}