#include <maxt_pkg/maxt_nodes/qr_detect_node.hpp>
#include "maxt_pkg/StringStamped.h"
#include <boost/algorithm/string.hpp>

namespace maxt {

QRDetectNode::QRDetectNode(const std::string& name, const BT::NodeConfiguration& config, MavKit& mav)
    : MavActionNode(name, config, mav), received_(false) {
}

BT::PortsList QRDetectNode::providedPorts() {
    return {
        BT::OutputPort<std::vector<std::string>>("qr_data"),
        BT::OutputPort<std::string>("target_1"),
        BT::OutputPort<std::string>("target_2"),
        BT::OutputPort<std::string>("land_target"),
        BT::InputPort<double>("timeout", 5.0, "Timeout in seconds for QR detection")
    };
}

void QRDetectNode::qrCallback(const maxt_pkg::StringStamped::ConstPtr& msg) {
    {
        std::lock_guard<std::mutex> lock(qr_mutex_);
        qr_data_ = msg->data;
    }
    received_ = true;
    
    // 打印QR检测结果
    ROS_INFO("QRDetectNode: Received QR codes:");
    for (const auto& code : qr_data_) {
        ROS_INFO("  - %s", code.c_str());
    }
}

BT::NodeStatus QRDetectNode::onStart() {
    received_ = false;
    qr_data_.clear();
    start_time_ = ros::Time::now();
    
    // 读取超时参数，默认为5秒
    getInput<double>("timeout", timeout_);
    
    // 订阅QR检测结果话题
    ros::NodeHandle nh;
    qr_sub_ = nh.subscribe("qr_detect_result", 1, &QRDetectNode::qrCallback, this);
    
    ROS_INFO("QRDetectNode: Started QR detection with timeout %.2f seconds", timeout_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus QRDetectNode::onRunning() {
    // 检查是否接收到QR数据或超时
    if (received_) {
        // 获取黑板
        auto blackboard = config().blackboard;
        
        // 将QR数据存储到黑板
        blackboard->set("qr_data", qr_data_);

        // 解析QR数据并存储到相应的黑板端口
        if (!qr_data_.empty()) {
            // 假设第一个QR码包含目标信息
            std::string qr_code = qr_data_[0];
            std::vector<std::string> parts;
            boost::split(parts, qr_code, boost::is_any_of(","));

            if (parts.size() >= 3) {
                boost::trim(parts[0]);
                boost::trim(parts[1]);
                boost::trim(parts[2]);
                blackboard->set("target_1", parts[0]);
                blackboard->set("target_2", parts[1]);
                blackboard->set("land_target", parts[2]);
                
                ROS_INFO("QRDetectNode: Parsed QR data:");
                ROS_INFO("  Target 1: %s", parts[0].c_str());
                ROS_INFO("  Target 2: %s", parts[1].c_str());
                ROS_INFO("  Land Target: %s", parts[2].c_str());
                ROS_INFO("QRDetectNode: Stored data to blackboard");
            } else {
                ROS_WARN("QRDetectNode: Invalid QR data format, expected 3 comma-separated values");
            }
        }
        
        // 取消订阅
        qr_sub_.shutdown();
        
        ROS_INFO("QRDetectNode: QR detection completed successfully");
        return BT::NodeStatus::SUCCESS;
    }
    
    // 超时处理
    if ((ros::Time::now() - start_time_).toSec() > timeout_) {
        ROS_WARN("QRDetectNode: QR detection timeout after %.2f seconds", timeout_);
        qr_sub_.shutdown();
        return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::RUNNING;
}

void QRDetectNode::onHalted() {
    // 取消订阅
    qr_sub_.shutdown();
    ROS_INFO("QRDetectNode: Halted");
}

} // namespace maxt
