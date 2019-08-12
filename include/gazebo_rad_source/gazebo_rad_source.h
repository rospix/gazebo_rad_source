#ifndef GAZEBO_RAD_SOURCE_H
#define GAZEBO_RAD_SOURCE_H

#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo_rad_msgs/Termination.pb.h>
#include <gazebo_rad_msgs/RadiationSource.pb.h>
#include <gazebo_rad_msgs/RadiationSource.h>
#include <gazebo_rad_msgs/DebugSetActivity.h>
#include <gazebo_rad_msgs/DebugSetMaterial.h>

namespace gazebo
{
class GAZEBO_VISIBLE Source : public ModelPlugin {
public:
  Source();
  virtual ~Source();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
  Eigen::Vector3d position;

  bool          terminated;
  bool          param_change;
  boost::thread publisher_thread;
  void          PublisherLoop();

  std::string                   material;
  double                        activity;
  double                        publish_rate;
  std::chrono::duration<double> sleep_seconds;

  physics::ModelPtr       model_;
  transport::NodePtr      gazebo_node_;
  transport::PublisherPtr gazebo_publisher_;
  transport::PublisherPtr termination_publisher_;
  event::ConnectionPtr    updateConnection_;

  std::unique_ptr<ros::NodeHandle> ros_node;
  ros::Publisher                   ros_publisher;
  ros::Subscriber                  change_activity_sub;
  ros::Subscriber                  change_material_sub;

  void SetActivityCallback(const gazebo_rad_msgs::DebugSetActivityPtr &msg);
  void SetMaterialCallback(const gazebo_rad_msgs::DebugSetMaterialPtr &msg);
};

GZ_REGISTER_MODEL_PLUGIN(Source)
Source::Source() : ModelPlugin() {
}
}  // namespace gazebo

#endif /* GAZEBO_RAD_SOURCE_H */
