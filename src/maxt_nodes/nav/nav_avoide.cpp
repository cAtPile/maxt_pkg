#include <maxt_pkg/maxt_nodes/nav/nav_avoide_node.hpp>
#include <maxt_pkg/mavkit/maxt_mav.hpp>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2/LinearMath/Transform.h>
#include <cmath>

namespace maxt {

NavAvoideNode::NavAvoideNode(const std::string& name, const BT::NodeConfiguration& config,
                             MavKit& mav, const ros::NodeHandle& nh)
    : BT::StatefulActionNode(name, config), mav_(mav), nh_(nh, ros::this_node::getNamespace()),
      tf_listener_(tf_buffer_) {
    nh_.setCallbackQueue(&queue_);
    goals_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal", 1);
    targets_ = nh_.advertise<mavros_msgs::PositionTarget>("primitive_planner/setpoint", 1);
    planner_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
}

BT::PortsList NavAvoideNode::providedPorts() {
    return {
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z"),
        BT::InputPort<double>("timeout", 60.0, "timeout (s)"),
        BT::InputPort<double>("tolerance", 0.1, "endpoint tolerance (m)")
    };
}

BT::NodeStatus NavAvoideNode::onStart() {
    auto& p = goal_.pose.position;
    if (!getInput("x", p.x) || !getInput("y", p.y) || !getInput("z", p.z) ||
        !getInput("timeout", timeout_) || !getInput("tolerance", tolerance_) ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(timeout_) || timeout_ <= 0 ||
        !std::isfinite(tolerance_) || tolerance_ <= 0) {
        ROS_ERROR("[NavAvoide] Invalid goal, timeout or tolerance.");
        return BT::NodeStatus::FAILURE;
    }
    sent_ = endpoint_commanded_ = false;
    start_ = ros::WallTime::now();
    commands_ = nh_.subscribe("/position_cmd", 1, &NavAvoideNode::command, this);
    lidar_cloud_ = nh_.subscribe("/lidar/points", 1, &NavAvoideNode::cloud, this);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavAvoideNode::onRunning() {
    if ((ros::WallTime::now() - start_).toSec() >= timeout_) {
        ROS_WARN("[NavAvoide] Timed out before reaching the goal.");
        onHalted();
        return BT::NodeStatus::FAILURE;
    }
    const auto pose = mav_.GetRawPose();
    if (!sent_ && !pose.header.frame_id.empty() &&
        goals_.getNumSubscribers() && commands_.getNumPublishers()) {
        goal_.header.frame_id = pose.header.frame_id;
        goal_.header.stamp = ros::Time::now();
        goal_.pose.orientation.w = 1;
        goals_.publish(goal_);
        sent_ = true;
        const auto& p = goal_.pose.position;
        ROS_INFO("[NavAvoide] Published /goal in %s: pos=(%.3f, %.3f, %.3f), stamp=%.3f",
                 goal_.header.frame_id.c_str(), p.x, p.y, p.z, goal_.header.stamp.toSec());
    }
    queue_.callAvailable(ros::WallDuration(0));
    if (!endpoint_commanded_) return BT::NodeStatus::RUNNING;
    const auto& p = pose.pose.position;
    const auto& goal = goal_.pose.position;
    if (!(std::hypot(p.x - goal.x, p.y - goal.y, p.z - goal.z) <= tolerance_)) {
        return BT::NodeStatus::RUNNING;
    }
    onHalted();
    return BT::NodeStatus::SUCCESS;
}

void NavAvoideNode::command(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    const char* rejection = nullptr;
    if (!sent_) {
        rejection = "goal not published";
    } else if (msg->header.stamp < goal_.header.stamp) {
        rejection = "command predates goal";
    } else if (msg->trajectory_flag != msg->TRAJECTORY_STATUS_READY) {
        rejection = "trajectory status is not READY";
    }
    ROS_INFO_THROTTLE(1.0,
        "[NavAvoide] /position_cmd %s: trajectory_id=%u flag=%u frame=%s stamp=%.3f "
        "pos=(%.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f) acc=(%.3f, %.3f, %.3f) "
        "yaw=%.3f yaw_rate=%.3f",
        rejection ? rejection : "accepted",
        static_cast<unsigned>(msg->trajectory_id), static_cast<unsigned>(msg->trajectory_flag),
        msg->header.frame_id.c_str(), msg->header.stamp.toSec(),
        msg->position.x, msg->position.y, msg->position.z,
        msg->velocity.x, msg->velocity.y, msg->velocity.z,
        msg->acceleration.x, msg->acceleration.y, msg->acceleration.z,
        msg->yaw, msg->yaw_dot);
    if (rejection) return;

    mavros_msgs::PositionTarget target;
    target.header = msg->header;
    target.coordinate_frame = target.FRAME_LOCAL_NED;
    target.position = msg->position;
    target.velocity = msg->velocity;
    target.acceleration_or_force = msg->acceleration;
    target.yaw = msg->yaw;
    target.yaw_rate = msg->yaw_dot;
    targets_.publish(target);
    mav_.setRawTarget(target, false);
    mav_.setSetpointMode(SetpointMode::RAW_CTRL);
    const auto& p = goal_.pose.position;
    endpoint_commanded_ = std::hypot(p.x - target.position.x, p.y - target.position.y,
                                     p.z - target.position.z) <= tolerance_;
}

void NavAvoideNode::cloud(const sensor_msgs::PointCloud::ConstPtr& msg) {
    const auto frame = mav_.GetRawPose().header.frame_id;
    if (frame.empty() || planner_cloud_.getNumSubscribers() == 0) return;

    try {
        const auto stamped = tf_buffer_.lookupTransform(frame, msg->header.frame_id,
                                                        msg->header.stamp, ros::Duration(0.03));
        const auto& q = stamped.transform.rotation;
        const auto& t = stamped.transform.translation;
        const tf2::Transform to_local(tf2::Quaternion(q.x, q.y, q.z, q.w),
                                      tf2::Vector3(t.x, t.y, t.z));
        sensor_msgs::PointCloud transformed = *msg;
        transformed.header.frame_id = frame;
        for (auto& point : transformed.points) {
            const auto p = to_local * tf2::Vector3(point.x, point.y, point.z);
            point.x = p.x();
            point.y = p.y();
            point.z = p.z();
        }
        sensor_msgs::PointCloud2 output;
        if (sensor_msgs::convertPointCloudToPointCloud2(transformed, output)) {
            planner_cloud_.publish(output);
        }
    } catch (const tf2::TransformException& e) {
        ROS_WARN_THROTTLE(1.0, "[NavAvoide] Cannot transform lidar cloud to %s: %s",
                          frame.c_str(), e.what());
    }
}

void NavAvoideNode::onHalted() {
    commands_.shutdown();
    lidar_cloud_.shutdown();
    queue_.clear();
    mav_.setSetpointMode(SetpointMode::HEARTBEAT);
}

} // namespace maxt
