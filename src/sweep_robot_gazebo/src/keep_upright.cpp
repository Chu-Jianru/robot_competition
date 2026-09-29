#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <cmath>

namespace gazebo
{
// planar_move 每步把横滚、俯仰角速度清零，重力来不及把车扳正，
// 撞墙产生的倾角会一步步累加。物理步结束后把车拉回水平。
class KeepUpright : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr) override
  {
    model_ = model;
    nominal_z_ = model_->WorldPose().Pos().Z();
    update_ = event::Events::ConnectWorldUpdateEnd(
      std::bind(&KeepUpright::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    ignition::math::Pose3d pose = model_->WorldPose();
    const double roll = pose.Rot().Roll();
    const double pitch = pose.Rot().Pitch();
    const double yaw = pose.Rot().Yaw();
    const double z = pose.Pos().Z();
    if (std::abs(roll) >= 1e-4 || std::abs(pitch) >= 1e-4 || std::abs(z - nominal_z_) >= 1e-4) {
      pose.Set(pose.Pos().X(), pose.Pos().Y(), nominal_z_, 0.0, 0.0, yaw);
      model_->SetWorldPose(pose);
    }
    ignition::math::Vector3d linear = model_->WorldLinearVel();
    ignition::math::Vector3d angular = model_->WorldAngularVel();
    if (std::abs(linear.Z()) < 1e-4 && std::abs(angular.X()) < 1e-4 && std::abs(angular.Y()) < 1e-4) {
      return;
    }
    linear.Z() = 0.0;
    model_->SetLinearVel(linear);
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, angular.Z()));
  }

  physics::ModelPtr model_;
  event::ConnectionPtr update_;
  double nominal_z_{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(KeepUpright)
}  // namespace gazebo
