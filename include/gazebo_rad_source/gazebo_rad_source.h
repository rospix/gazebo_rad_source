#ifndef GAZEBO_RAD_SOURCE_H
#define GAZEBO_RAD_SOURCE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
class GAZEBO_VISIBLE Source : public ModelPlugin {
public:
  Source();
  virtual ~Source();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void LateUpdate();

private:
  bool          terminated;
  boost::thread publisher_thread;
  void          PublisherLoop();

  std::string material;
  double      activity;
  double      publish_rate;

  physics::ModelPtr       model_;
  transport::NodePtr      gazebo_node_;
  transport::PublisherPtr gazebo_publisher_;
  event::ConnectionPtr    updateConnection_;
  void                    EarlyUpdate(const common::UpdateInfo &);
};

GZ_REGISTER_MODEL_PLUGIN(Source)
Source::Source() : ModelPlugin() {
}
}  // namespace gazebo

#endif /* GAZEBO_RAD_SOURCE_H */
