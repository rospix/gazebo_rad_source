#include <gazebo_rad_source/gazebo_rad_source.h>
#include <gazebo_rad_msgs/RadiationSource.pb.h>
#include <gazebo_rad_msgs/RadiationSource.h>
#include <boost/thread.hpp>

using namespace gazebo;

/* Destructor //{ */
Source::~Source() {
  terminated = true;
  publisher_thread.join();
  ROS_INFO("[RadiationSource%u]: Plugin terminated", model_->GetId());
}
//}

/* Load //{ */
void Source::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_       = _model;
  material     = _sdf->Get<std::string>("material");
  material     = _sdf->Get<double>("activity");
  publish_rate = _sdf->Get<double>("publish_rate");

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Source::EarlyUpdate, this, _1));

  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init();
  this->gazebo_publisher_ = gazebo_node_->Advertise<gazebo_rad_msgs::msgs::RadiationSource>("~/radiation/sources", 1);

  terminated       = false;
  publisher_thread = boost::thread(boost::bind(&Source::PublisherLoop, this));
  ROS_INFO("[RadiationSource%u]: Plugin terminated", model_->GetId());
}
//}

/* PublisherLoop //{ */
void Source::PublisherLoop() {
  while (!terminated) {

    /* Gazebo message //{ */
    gazebo_rad_msgs::msgs::RadiationSource msg;
    msg.set_x(model_->WorldPose().Pos().X());
    msg.set_y(model_->WorldPose().Pos().Y());
    msg.set_z(model_->WorldPose().Pos().Z());
    msg.set_id(model_->GetId());
    msg.set_material(material);
    msg.set_activity(activity);
    gazebo_publisher_->Publish(msg);
    //}

    /* ROS message (debug) //{ */
    gazebo_rad_msgs::RadiationSource debug_msg;
    debug_msg.activity = activity;
    debug_msg.material = material;
    debug_msg.id       = model_->GetId();
    debug_msg.x        = model_->WorldPose().Pos().X();
    debug_msg.y        = model_->WorldPose().Pos().Y();
    debug_msg.z        = model_->WorldPose().Pos().Z();
    debug_msg.stamp    = ros::Time::now();
    //}

    std::this_thread::sleep_for(std::chrono::duration<double>(1 / publish_rate));
  }
}
//}

/* EarlyUpdate //{ */
void Source::EarlyUpdate(const common::UpdateInfo &) {
  std::cout << "weeeeeee\n";
}
//}
