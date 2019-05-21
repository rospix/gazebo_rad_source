#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>

#include <sdf/sdf.hh>
#include <common.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <gazebo_rad_msgs/RadiationSource.pb.h>
#include <gazebo_rad_msgs/RadiationSource.h>
#include <gazebo_rad_msgs/DebugSetActivity.h>
#include <gazebo_rad_msgs/DebugSetMaterial.h>

#include <geometry_msgs/PoseStamped.h>

namespace gazebo
{

  /* class RadiationSource //{ */
  // Class definition
  class GAZEBO_VISIBLE RadiationSource : public ModelPlugin {
  public:
    // Constructor & Destructor
    RadiationSource();
    virtual ~RadiationSource();

    // Make this plugin callable from ROS
    void QueueThread() {
      static const double timeout = 0.01;
      while (this->rosNode->ok()) {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  protected:
    // Called when plugin is loaded into Gazebo world
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    // Connect to Gazebo event system, called every simulation update (100Hz)
    virtual void OnUpdate(const common::UpdateInfo &);

  private:
    std::string namespace_;

    physics::EntityPtr modelEntity;

    physics::ModelPtr    model_;
    physics::WorldPtr    world_;
    event::ConnectionPtr updateConnection_;

    transport::NodePtr      node_handle_;
    transport::PublisherPtr rad_pub;

    boost::thread callback_queue_thread_;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::CallbackQueue               rosQueue;
    std::thread                      rosQueueThread;

    // Debugging ros topics
    ros::Publisher  debug_pos_pub;
    ros::Subscriber change_activity_sub, change_material_sub;

    gazebo_rad_msgs::msgs::RadiationSource radiation_msg;

    void setActivityCallback(const gazebo_rad_msgs::DebugSetActivityConstPtr &msg);
    void setMaterialCallback(const gazebo_rad_msgs::DebugSetMaterialConstPtr &msg);

  private:
    std::string material_;
    double      activity_;
  };

  GZ_REGISTER_MODEL_PLUGIN(RadiationSource)
  RadiationSource::RadiationSource() : ModelPlugin() {
  }

  // End this node properly -> avoid threading exceptions
  RadiationSource::~RadiationSource() {
    // end all ROS related stuff
    this->rosNode->shutdown();
    // wait for threads to finish
    this->rosQueueThread.join();
    this->callback_queue_thread_.join();
    // end connection to gazebo
    updateConnection_->~Connection();
  }

  //}

  /* setActivityCallback */  //{
  // Change activity of the source by a ROS message
  void RadiationSource::setActivityCallback(const gazebo_rad_msgs::DebugSetActivityConstPtr &msg) {
    unsigned int my_id  = model_->GetId();
    unsigned int msg_id = msg->id;
    if (my_id == msg_id) {
      this->activity_ = msg->activity;
      ROS_INFO("[RadiationSource%u]: Activity changed to %.1f Bq", this->model_->GetId(), this->activity_);
    }
  }
  //}

  /* setMaterialCallback */  //{
  // Change material of the source by a ROS message
  void RadiationSource::setMaterialCallback(const gazebo_rad_msgs::DebugSetMaterialConstPtr &msg) {
    unsigned int my_id  = model_->GetId();
    unsigned int msg_id = msg->id;
    if (my_id == msg_id) {
      this->material_ = msg->material;
      ROS_INFO("[RadiationSource%u]: Material changed to %s", this->model_->GetId(), this->material_.c_str());
    }
  }
  //}

  /* Load() //{ */
  // Called once when the model is loaded into Gazebo
  void RadiationSource::Load(physics::ModelPtr _model, [[maybe_unused]] sdf::ElementPtr _sdf) {

    int    argc = 0;
    char **argv = NULL;

    // Create a new ROS node
    ros::init(argc, argv, "gazebo_ros_radiation", ros::init_options::NoSigintHandler);
    this->rosNode.reset(new ros::NodeHandle("~"));

    // Store pointers to the model and world handles
    model_ = _model;
    world_ = model_->GetWorld();

    // param check
    if (_sdf->HasElement("material")) {
      getSdfParam<std::string>(_sdf, "material", material_, material_);
    } else {
      ROS_INFO("[RadiationSrouce]: parameter 'material' was not specified");
    }

    // param check
    if (_sdf->HasElement("activity")) {
      getSdfParam<double>(_sdf, "activity", activity_, activity_);
    } else {
      ROS_INFO("[RadiationSource]: parameter 'activity' was not specified");
    }

    // Gazebo messaging system
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&RadiationSource::OnUpdate, this, _1));
    this->rad_pub     = node_handle_->Advertise<gazebo_rad_msgs::msgs::RadiationSource>("~/radiation/sources", 1);

    // ROS messaging system
    this->debug_pos_pub       = this->rosNode->advertise<geometry_msgs::PoseStamped>("/radiation/sources", 1);
    this->change_activity_sub = this->rosNode->subscribe("/radiation/debug/set_activity", 1, &RadiationSource::setActivityCallback, this);
    this->change_material_sub = this->rosNode->subscribe("/radiation/debug/set_material", 1, &RadiationSource::setMaterialCallback, this);

    // Make the node callable from ROS
    this->rosQueueThread = std::thread(std::bind(&RadiationSource::QueueThread, this));

    ROS_INFO("[RadiationSource%u]: initialized", this->model_->GetId());
  }

  //}

  /* OnUpdate() //{ */
  // Called on every simulation update (100 Hz)
  void RadiationSource::OnUpdate(const common::UpdateInfo &) {

    auto pose = model_->WorldPose();

    radiation_msg.set_x(pose.Pos().X());
    radiation_msg.set_y(pose.Pos().Y());
    radiation_msg.set_z(pose.Pos().Z());
    radiation_msg.set_material(material_);
    radiation_msg.set_activity(activity_);
    radiation_msg.set_id(this->model_->GetId());

    rad_pub->Publish(radiation_msg);

    geometry_msgs::PoseStamped debug_pose;
    debug_pose.header.stamp.nsec = this->model_->GetId();
    debug_pose.pose.orientation.x = activity_;
    debug_pose.header.frame_id    = "local_origin";
    debug_pose.pose.position.x    = pose.Pos().X();
    debug_pose.pose.position.y    = pose.Pos().Y();
    debug_pose.pose.position.z    = pose.Pos().Z();

    debug_pos_pub.publish(debug_pose);
  }

  //}

}  // namespace gazebo
