#include <cmath>
#include <functional>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <ros/ros.h>

namespace maxt_pkg
{
// Keep trajectory evaluation inside Gazebo's update thread, independent of ROS
// transport latency and /clock delivery. ROS parameters are read only at load.
class MovingTargetPlugin : public gazebo::ModelPlugin
{
public:
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
        model_ = model;
        center_pose_ = model_->WorldPose();
        bool enabled = true;
        if (ros::isInitialized())
        {
            const std::string ns = sdf->HasElement("robotNamespace")
                ? sdf->Get<std::string>("robotNamespace") : "/moving_target";
            ros::NodeHandle params(ns);
            params.param("enabled", enabled, true);
            params.param("x", center_pose_.Pos().X(), center_pose_.Pos().X());
            params.param("y", center_pose_.Pos().Y(), center_pose_.Pos().Y());
            params.param("z", center_pose_.Pos().Z(), center_pose_.Pos().Z());
            params.param("amplitude", amplitude_, 2.0);
            params.param("period", period_, 12.0);
        }
        if (!enabled)
            return;
        if (!std::isfinite(period_) || period_ <= 0.0 ||
            !std::isfinite(amplitude_) || amplitude_ < 0.0)
        {
            gzerr << "MovingTargetPlugin: period must be positive and amplitude "
                  << "nonnegative (both finite).\n";
            return;
        }
        Reset();
        update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&MovingTargetPlugin::OnUpdate, this, std::placeholders::_1));
        gzmsg << "MovingTargetPlugin: " << model_->GetName()
              << " follows a sinusoid every physics step, period=" << period_
              << ", amplitude=" << amplitude_ << "\n";
    }

    void Reset() override
    {
        start_time_ = model_->GetWorld()->SimTime().Double();
        previous_time_ = start_time_;
    }

private:
    void OnUpdate(const gazebo::common::UpdateInfo &info)
    {
        const double now = info.simTime.Double();
        // Also handle an explicit clock rewind, without integrating accumulated dt.
        if (now < previous_time_)
            start_time_ = now;
        previous_time_ = now;
        const double phase = 2.0 * std::acos(-1.0) *
            std::fmod(now - start_time_, period_) / period_;
        auto pose = center_pose_;
        pose.Pos().Y() += amplitude_ * std::sin(phase);
        // Preserve the world-defined orientation (the image faces upward).
        model_->SetWorldPose(pose);
    }

    gazebo::physics::ModelPtr model_;
    ignition::math::Pose3d center_pose_;
    double amplitude_ = 2.0;
    double period_ = 12.0;
    double start_time_ = 0.0;
    double previous_time_ = 0.0;
    gazebo::event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(MovingTargetPlugin)
}  // namespace maxt_pkg
