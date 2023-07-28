/* includes //{ */

#include <sdf/sdf.hh>
#include <boost/thread.hpp>

#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_rad_msgs/Termination.pb.h>
#include <gazebo_rad_msgs/RadiationSource.pb.h>
#include <gazebo_rad_msgs/RadiationSource.h>
#include <gazebo_rad_msgs/DebugSetActivity.h>
#include <gazebo_rad_msgs/DebugSetMaterial.h>

#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/Float64Srv.h>
#include <mrs_msgs/String.h>
#include <mrs_lib/mutex.h>

//}

namespace gazebo
{

/* class Source //{ */

class GAZEBO_VISIBLE Source : public ModelPlugin {
public:
  Source();
  virtual ~Source();

  void onWorldUpdate();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
  Eigen::Vector3d position;

  bool          terminated;
  boost::thread publisher_thread;
  void          PublisherLoop();

  std::string material_;
  std::mutex  material_mutex_;

  double     activity_;
  std::mutex activity_mutex_;

  double                        energy;
  double                        publish_rate;
  std::chrono::duration<double> sleep_seconds;

  ignition::math::Vector3d velocity_;
  std::mutex               velocity_mutex_;

  physics::ModelPtr       model_;
  transport::NodePtr      gazebo_node_;
  transport::PublisherPtr gazebo_publisher_;
  transport::PublisherPtr termination_publisher_;
  event::ConnectionPtr    updateConnection_;

  ros::NodeHandle    ros_nh_;
  ros::Publisher     ros_publisher;
  ros::Publisher     ros_gt_publisher;
  ros::ServiceServer set_activity_server;
  ros::ServiceServer set_material_server;
  ros::ServiceServer set_motion_server;

  bool SetActivityCallback(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res);
  bool SetMaterialCallback(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  bool SetMotionCallback(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res);
};

//}

// | -------------------- Plugin interface -------------------- |

/* Load() //{ */
void Source::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  // init local variables
  model_ = _model;

  position = Eigen::Vector3d(model_->WorldPose().Pos().X(), model_->WorldPose().Pos().Y(), model_->WorldPose().Pos().Z());

  /* parse sdf params //{ */

  if (_sdf->HasElement("material")) {
    material_ = _sdf->Get<std::string>("material");
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'material' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("activity")) {
    activity_ = _sdf->Get<double>("activity");
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'activity' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("energy")) {
    energy = _sdf->Get<double>("energy");
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'energy' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("publish_rate")) {
    publish_rate  = _sdf->Get<double>("publish_rate");
    sleep_seconds = std::chrono::duration<double>(1 / publish_rate);
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'publish_rate' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("velocity")) {
    auto velocity = _sdf->GetElement("velocity");
    if (velocity->HasElement("x")) {
      velocity_.X() = velocity->Get<double>("x");
    } else {
      ROS_WARN("[RadiationSource%u]: parameter 'velocity/x' was not specified", model_->GetId());
      velocity_.X() = 0;
    }
    if (velocity->HasElement("y")) {
      velocity_.Y() = velocity->Get<double>("y");
    } else {
      ROS_WARN("[RadiationSource%u]: parameter 'velocity/y' was not specified", model_->GetId());
      velocity_.Y() = 0;
    }
    if (velocity->HasElement("z")) {
      velocity_.Z() = velocity->Get<double>("z");
    } else {
      ROS_WARN("[RadiationSource%u]: parameter 'velocity/z' was not specified", model_->GetId());
      velocity_.Z() = 0;
    }
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'velocity' was not specified", model_->GetId());
  }

  //}

  // init gazebo node
  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init();

  // init ros node
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_rad_source", ros::init_options::NoSigintHandler);
  ros_nh_ = ros::NodeHandle("~");

  // gazebo communication
  this->gazebo_publisher_      = gazebo_node_->Advertise<gazebo_rad_msgs::msgs::RadiationSource>("/radiation/sources", 1);
  this->termination_publisher_ = gazebo_node_->Advertise<gazebo_rad_msgs::msgs::Termination>("/radiation/termination", 1);

  // ros communication
  ros_publisher       = ros_nh_.advertise<gazebo_rad_msgs::RadiationSource>("/radiation/sources", 1);
  ros_gt_publisher    = ros_nh_.advertise<geometry_msgs::PoseStamped>(_model->GetName() + "/source_gt", 1);
  set_activity_server = ros_nh_.advertiseService(_model->GetName() + "/set_activity", &Source::SetActivityCallback, this);
  set_material_server = ros_nh_.advertiseService(_model->GetName() + "/set_material", &Source::SetMaterialCallback, this);
  set_motion_server   = ros_nh_.advertiseService(_model->GetName() + "/set_motion", &Source::SetMotionCallback, this);

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&Source::onWorldUpdate, this));

  terminated       = false;
  publisher_thread = boost::thread(boost::bind(&Source::PublisherLoop, this));
  ROS_INFO("[RadiationSource%u]: Plugin initialized", model_->GetId());
}
//}

/* Source() //{ */

Source::Source() {
}

//}

/* ~Source() //{ */

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

// | ---------------------- ROS callbacks --------------------- |

/* SetActivityCallback() //{ */

bool Source::SetActivityCallback(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res) {
  std::stringstream msg;
  auto              activity = mrs_lib::get_mutexed(activity_mutex_, activity_);
  if (req.value != activity) {
    mrs_lib::set_mutexed(activity_mutex_, req.value, activity_);
    msg << "Activity set to " << req.value;
    ROS_INFO("[RadiationSource%u]: %s", model_->GetId(), msg.str().c_str());
    res.success = true;
  } else {
    msg << "Activity is already at " << req.value;
    res.success = false;
  }
  res.message = msg.str();
  return true;
}

//}

/* setMaterialCallback() //{ */

bool Source::SetMaterialCallback(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {
  std::stringstream msg;
  auto              material = mrs_lib::get_mutexed(material_mutex_, material_);
  if (req.value != material) {
    mrs_lib::set_mutexed(material_mutex_, req.value, material_);
    msg << "Material set to" << req.value;
    ROS_INFO("[RadiationSource%u]: %s", model_->GetId(), msg.str().c_str());
    res.success = true;
  } else {
    msg << "Material is already " << req.value;
    res.success = false;
  }
  res.message = msg.str();
  return true;
}

//}

// | --------------------- Custom routines -------------------- |

/* PublisherLoop() //{ */

void Source::PublisherLoop() {

  while (!terminated) {

    /* Gazebo message //{ */

    auto material = mrs_lib::get_mutexed(material_mutex_, material_);
    auto activity = mrs_lib::get_mutexed(activity_mutex_, activity_);

    gazebo_rad_msgs::msgs::RadiationSource msg;
    msg.set_x(model_->WorldPose().Pos().X());
    msg.set_y(model_->WorldPose().Pos().Y());
    msg.set_z(model_->WorldPose().Pos().Z());
    msg.set_id(model_->GetId());
    msg.set_material(material);
    msg.set_activity(activity);
    msg.set_energy(energy);
    gazebo_publisher_->Publish(msg);

    //}

    /* ROS message (debug) //{ */
    
    gazebo_rad_msgs::RadiationSource debug_msg;
    debug_msg.activity    = activity;
    debug_msg.material    = material;
    debug_msg.energy      = energy;
    debug_msg.id          = model_->GetId();
    debug_msg.world_pos.x = model_->WorldPose().Pos().X();
    debug_msg.world_pos.y = model_->WorldPose().Pos().Y();
    debug_msg.world_pos.z = model_->WorldPose().Pos().Z();
    debug_msg.stamp       = ros::Time::now();
    ros_publisher.publish(debug_msg);

    //}

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp       = ros::Time::now();
    pose_msg.header.frame_id    = "uav1/gps_origin";
    pose_msg.pose.position.x    = model_->WorldPose().Pos().X();
    pose_msg.pose.position.y    = model_->WorldPose().Pos().Y();
    pose_msg.pose.position.z    = model_->WorldPose().Pos().Z();
    pose_msg.pose.orientation.x = 0;
    pose_msg.pose.orientation.y = 0;
    pose_msg.pose.orientation.z = 1;
    pose_msg.pose.orientation.w = 0;
    ros_gt_publisher.publish(pose_msg);

    std::this_thread::sleep_for(sleep_seconds);
  }
}

//}

/* onWorldUpdate //{ */
void Source::onWorldUpdate() {
  auto velocity = mrs_lib::get_mutexed(velocity_mutex_, velocity_);
  model_->SetLinearVel(velocity);
}
//}

/* SetMotionCallback //{ */
bool Source::SetMotionCallback(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res) {
  ignition::math::Vector3d velocity;
  velocity.X() = req.goal[0];
  velocity.Y() = req.goal[1];
  velocity.Z() = req.goal[2];

  mrs_lib::set_mutexed(velocity_mutex_, velocity, velocity_);

  std::stringstream msg;
  msg << "Velocity set: " << velocity.X() << ", " << velocity.Y() << ", " << velocity.Z();
  ROS_INFO("[RadiationSource%u]: %s", model_->GetId(), msg.str().c_str());
  res.message = msg.str();
  res.success = true;
  return true;
}
//}

}  // namespace gazebo

GZ_REGISTER_MODEL_PLUGIN(gazebo::Source)
