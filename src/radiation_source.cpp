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

#include <mutex>

#include <gazebo_rad_msgs/RadiationSource.pb.h>
#include <gazebo_rad_msgs/RadiationSource.h>

namespace gazebo
{

  /* class RadiationSource //{ */

  class GAZEBO_VISIBLE RadiationSource : public ModelPlugin {
  public:
    RadiationSource();
    virtual ~RadiationSource();

    void QueueThread() {
      static const double timeout = 0.01;
      while (this->rosNode->ok()) {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  protected:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo&);

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
    ros::Publisher                   test_pub;

    gazebo_rad_msgs::msgs::RadiationSource radiation_msg;

    common::Time last_gps_time_;
    common::Time last_time_;

    std::string modelName;

  private:
    std::string material_;
    double      activity_;
  };

  GZ_REGISTER_MODEL_PLUGIN(RadiationSource)
  RadiationSource::RadiationSource() : ModelPlugin() {
  }

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

  /* Load() //{ */

  void RadiationSource::Load(physics::ModelPtr _model, [[maybe_unused]] sdf::ElementPtr _sdf) {

    int    argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_ros_radiation", ros::init_options::NoSigintHandler);

    this->rosNode.reset(new ros::NodeHandle("~"));

    // Store the pointer to the model.
    model_ = _model;

    world_ = model_->GetWorld();

    last_time_     = world_->SimTime();
    last_gps_time_ = world_->SimTime();

    if (_sdf->HasElement("material")) {
      getSdfParam<std::string>(_sdf, "material", material_, material_);
    } else {
      ROS_INFO("[RadiationSrouce]: parameter 'material' was not specified");
    }

    if (_sdf->HasElement("activity")) {
      getSdfParam<double>(_sdf, "activity", activity_, activity_);
    } else {
      ROS_INFO("[RadiationSource]: parameter 'activity' was not specified");
    }

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&RadiationSource::OnUpdate, this, _1));

    this->modelName = model_->GetName();

    this->rad_pub = node_handle_->Advertise<gazebo_rad_msgs::msgs::RadiationSource>("~/radiation", 1);

    this->test_pub = this->rosNode->advertise<gazebo_rad_msgs::RadiationSource>("radiation", 1);

    this->rosQueueThread = std::thread(std::bind(&RadiationSource::QueueThread, this));

    ROS_INFO("[RadiationSource%u]: initialized", this->model_->GetId());
  }

  //}

  /* OnUpdate() //{ */

  void RadiationSource::OnUpdate(const common::UpdateInfo&) {

    common::Time current_time = world_->SimTime();

    ignition::math::Pose3d T_W_I = model_->WorldPose();

    radiation_msg.set_x(T_W_I.Pos().X());
    radiation_msg.set_y(T_W_I.Pos().Y());
    radiation_msg.set_z(T_W_I.Pos().Z());
    radiation_msg.set_material(material_);
    radiation_msg.set_activity(activity_);
    radiation_msg.set_id(this->node_handle_->GetId());

    rad_pub->Publish(radiation_msg);

    last_time_ = current_time;

    gazebo_rad_msgs::RadiationSource rad_out;
    rad_out.stamp    = ros::Time::now();
    rad_out.x        = T_W_I.Pos().X();
    rad_out.y        = T_W_I.Pos().Y();
    rad_out.z        = T_W_I.Pos().Z();
    rad_out.activity = activity_;
    rad_out.material = material_;
    rad_out.id       = this->node_handle_->GetId();

    test_pub.publish(rad_out);
  }

  //}

}  // namespace gazebo
