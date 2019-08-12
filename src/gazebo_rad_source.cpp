#include <sdf/sdf.hh>
#include <boost/thread.hpp>

#include <gazebo_rad_source/gazebo_rad_source.h>

using namespace gazebo;

/* Destructor //{ */
Source::~Source() {
  // inform other gazebo nodes
  gazebo_rad_msgs::msgs::Termination msg;
  msg.set_id(model_->GetId());
  termination_publisher_->Publish(msg);

  // terminate
  terminated = true;
  publisher_thread.join();
  ROS_INFO("[RadiationSource%u]: Plugin terminated", model_->GetId());
}
//}

/* Load //{ */
void Source::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  // init local variables
  model_       = _model;
  param_change = true;

  position      = Eigen::Vector3d(model_->WorldPose().Pos().X(), model_->WorldPose().Pos().Y(), model_->WorldPose().Pos().Z());

  // parse sdf params
  if (_sdf->HasElement("material")) {
    material = _sdf->Get<std::string>("material");
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'material' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("activity")) {
    activity = _sdf->Get<double>("activity");
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'activity' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("publish_rate")) {
    publish_rate  = _sdf->Get<double>("publish_rate");
    sleep_seconds = std::chrono::duration<double>(1 / publish_rate);
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'publish_rate' was not specified", model_->GetId());
  }

  // init gazebo node
  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init();

  // init ros node
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_rad_source", ros::init_options::NoSigintHandler);
  ros_node.reset(new ros::NodeHandle("~"));

  // gazebo communication
  this->gazebo_publisher_      = gazebo_node_->Advertise<gazebo_rad_msgs::msgs::RadiationSource>("~/radiation/sources", 1);
  this->termination_publisher_ = gazebo_node_->Advertise<gazebo_rad_msgs::msgs::Termination>("~/radiation/termination", 1);

  // ros communication
  ros_publisher       = ros_node->advertise<gazebo_rad_msgs::RadiationSource>("/radiation/sources", 1);
  change_activity_sub = ros_node->subscribe("/radiation/debug/set_activity", 1, &Source::SetActivityCallback, this);
  change_material_sub = ros_node->subscribe("/radiation/debug/set_material", 1, &Source::SetMaterialCallback, this);

  terminated       = false;
  publisher_thread = boost::thread(boost::bind(&Source::PublisherLoop, this));
  ROS_INFO("[RadiationSource%u]: Plugin initialized", model_->GetId());
}
//}

/* PublisherLoop //{ */
void Source::PublisherLoop() {
  while (!terminated) {

    Eigen::Vector3d new_position(model_->WorldPose().Pos().X(), model_->WorldPose().Pos().Y(), model_->WorldPose().Pos().Z());

    if (new_position == position && !param_change) {
      std::this_thread::sleep_for(sleep_seconds);
      continue;
    }

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
    ros_publisher.publish(debug_msg);
    //}

    std::this_thread::sleep_for(sleep_seconds);
    position     = new_position;
    param_change = false;
  }
}
//}

/* SetActivityCallback //{ */
void Source::SetActivityCallback(const gazebo_rad_msgs::DebugSetActivityPtr &msg) {
  unsigned int my_id  = model_->GetId();
  unsigned int msg_id = msg->id;
  if (my_id == msg_id) {
    activity = msg->activity;
    ROS_INFO("[RadiationSource%u]: Activity changed to %.1f Bq", model_->GetId(), activity);
    param_change = true;
  }
}
//}

/* setMaterialCallback //{ */
void Source::SetMaterialCallback(const gazebo_rad_msgs::DebugSetMaterialPtr &msg) {
  unsigned int my_id  = model_->GetId();
  unsigned int msg_id = msg->id;
  if (my_id == msg_id) {
    material = msg->material;
    ROS_INFO("[RadiationSource%u]: Material changed to %s", model_->GetId(), material.c_str());
    param_change = true;
  }
}
//}
