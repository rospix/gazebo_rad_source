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

//}

namespace gazebo
{

/* class Source //{ */

class GAZEBO_VISIBLE Source : public ModelPlugin {
public:
  Source();
  virtual ~Source();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
  Eigen::Vector3d position;

  bool          terminated;
  boost::thread publisher_thread;
  void          PublisherLoop();

  std::string                   material;
  double                        activity;
  double                        energy;
  double                        publish_rate;
  std::chrono::duration<double> sleep_seconds;

  physics::ModelPtr       model_;
  transport::NodePtr      gazebo_node_;
  transport::PublisherPtr gazebo_publisher_;
  transport::PublisherPtr termination_publisher_;
  event::ConnectionPtr    updateConnection_;

  ros::NodeHandle ros_nh_;
  ros::Publisher  ros_publisher;
  ros::Publisher  ros_gt_publisher;
  ros::Subscriber change_activity_sub;
  ros::Subscriber change_material_sub;

  void SetActivityCallback(const gazebo_rad_msgs::DebugSetActivityPtr &msg);
  void SetMaterialCallback(const gazebo_rad_msgs::DebugSetMaterialPtr &msg);
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
    material = _sdf->Get<std::string>("material");
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'material' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("activity")) {
    activity = _sdf->Get<double>("activity");
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
  this->gazebo_publisher_      = gazebo_node_->Advertise<gazebo_rad_msgs::msgs::RadiationSource>("~/radiation/sources", 1);
  this->termination_publisher_ = gazebo_node_->Advertise<gazebo_rad_msgs::msgs::Termination>("~/radiation/termination", 1);

  // ros communication
  ros_publisher       = ros_nh_.advertise<gazebo_rad_msgs::RadiationSource>("/radiation/sources", 1);
  ros_gt_publisher    = ros_nh_.advertise<geometry_msgs::PoseStamped>(_model->GetName() + "/source_gt", 1);
  change_activity_sub = ros_nh_.subscribe("/radiation/debug/set_activity", 1, &Source::SetActivityCallback, this);
  change_material_sub = ros_nh_.subscribe("/radiation/debug/set_material", 1, &Source::SetMaterialCallback, this);

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

void Source::SetActivityCallback(const gazebo_rad_msgs::DebugSetActivityPtr &msg) {

  unsigned int my_id  = model_->GetId();
  unsigned int msg_id = msg->id;

  if (my_id == msg_id) {
    activity = msg->activity;
    ROS_INFO("[RadiationSource%u]: Activity changed to %.1f Bq", model_->GetId(), activity);
  }
}

//}

/* setMaterialCallback() //{ */

void Source::SetMaterialCallback(const gazebo_rad_msgs::DebugSetMaterialPtr &msg) {

  unsigned int my_id  = model_->GetId();
  unsigned int msg_id = msg->id;

  if (my_id == msg_id) {
    material = msg->material;
    ROS_INFO("[RadiationSource%u]: Material changed to %s", model_->GetId(), material.c_str());
  }
}

//}

// | --------------------- Custom routines -------------------- |

/* PublisherLoop() //{ */

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
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "uav1/gps_origin";
    pose_msg.pose.position.x = model_->WorldPose().Pos().X();
    pose_msg.pose.position.y = model_->WorldPose().Pos().Y();
    pose_msg.pose.position.z = model_->WorldPose().Pos().Z();
    pose_msg.pose.orientation.x = 0;
    pose_msg.pose.orientation.y = 0;
    pose_msg.pose.orientation.z = 1;
    pose_msg.pose.orientation.w = 0;
    ros_gt_publisher.publish(pose_msg);

    std::this_thread::sleep_for(sleep_seconds);
  }
}

//}

}  // namespace gazebo

GZ_REGISTER_MODEL_PLUGIN(gazebo::Source)
