#include <memory>
#include <mutex>
#include <string>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace robot0_gazebo
{
class PlanarVelocityControl
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
public:
  PlanarVelocityControl() = default;
  ~PlanarVelocityControl() override = default;

  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &/*_eventMgr*/) override
  {
    this->model = ignition::gazebo::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      ignerr << "PlanarVelocityControl plugin should be attached to a model entity.\n";
      return;
    }

    this->canonicalLink = ignition::gazebo::Link(this->model.CanonicalLink(_ecm));
    if (!this->canonicalLink.Valid(_ecm))
    {
      ignerr << "Failed to find canonical link for PlanarVelocityControl.\n";
      return;
    }

    this->canonicalLink.EnableVelocityChecks(_ecm, true);

    std::string topic = "/cmd_vel";
    if (_sdf->HasElement("topic"))
    {
      topic = _sdf->Get<std::string>("topic");
    }

    this->node.Subscribe(topic, &PlanarVelocityControl::OnCmdVel, this);
    ignmsg << "PlanarVelocityControl initialized on canonical link: "
           << this->canonicalLink.Name(_ecm).value_or("unknown")
           << ", subscribed to: " << topic << std::endl;
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    std::lock_guard<std::mutex> lock(this->mutex);

    // Get current link orientation
    auto worldPose = this->canonicalLink.WorldPose(_ecm);
    ignition::math::Quaterniond rot = worldPose ? worldPose->Rot() : ignition::math::Quaterniond::Identity;

    // Get current velocities in world frame from physics engine
    ignition::math::Vector3d worldLinVel = this->canonicalLink.WorldLinearVelocity(_ecm).value_or(ignition::math::Vector3d::Zero);
    ignition::math::Vector3d worldAngVel = this->canonicalLink.WorldAngularVelocity(_ecm).value_or(ignition::math::Vector3d::Zero);

    // Convert to link body frame
    ignition::math::Vector3d linkLinVel = rot.Inverse().RotateVector(worldLinVel);
    ignition::math::Vector3d linkAngVel = rot.Inverse().RotateVector(worldAngVel);

    // Command in Link frame:
    // - X, Y: Command from /cmd_vel (planar translation)
    // - Z: PRESERVE current physics velocity (gravity free-fall, terrain compliance)
    ignition::math::Vector3d cmdLin(this->targetLinear.X(), this->targetLinear.Y(), linkLinVel.Z());
    this->canonicalLink.SetLinearVelocity(_ecm, cmdLin);

    // Angular command in Link frame:
    // - Z (Yaw): Command from /cmd_vel (turning)
    // - X (Roll), Y (Pitch): PRESERVE physics orientation rates
    ignition::math::Vector3d cmdAng(linkAngVel.X(), linkAngVel.Y(), this->targetAngular.Z());
    this->canonicalLink.SetAngularVelocity(_ecm, cmdAng);
  }

private:
  void OnCmdVel(const ignition::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->targetLinear = ignition::msgs::Convert(_msg.linear());
    this->targetAngular = ignition::msgs::Convert(_msg.angular());
  }

  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
  ignition::gazebo::Link canonicalLink{ignition::gazebo::kNullEntity};
  ignition::transport::Node node;
  std::mutex mutex;
  ignition::math::Vector3d targetLinear{0, 0, 0};
  ignition::math::Vector3d targetAngular{0, 0, 0};
};
}

IGNITION_ADD_PLUGIN(
    robot0_gazebo::PlanarVelocityControl,
    ignition::gazebo::System,
    robot0_gazebo::PlanarVelocityControl::ISystemConfigure,
    robot0_gazebo::PlanarVelocityControl::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    robot0_gazebo::PlanarVelocityControl,
    "robot0_gazebo::PlanarVelocityControl")
