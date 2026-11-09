#include <maxt_pkg/maxt_core.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <std_msgs/String.h>
#include <sstream>

namespace maxt {

MaxtCore::MaxtCore(ros::NodeHandle& nh) : nh_(nh) {
    // 获取参数：XML路径和运行频率
    nh_.param<std::string>("bt_xml_path", xml_path_, "path/to/your/tree.xml");
    nh_.param<double>("bt_tick_rate", tick_rate_, 20.0);
}

MaxtCore::~MaxtCore() {
    if (spinner_) {
        spinner_->stop();
    }
    ROS_INFO("MaxtCore: System shut down.");
}

bool MaxtCore::init() {
    // 1. 初始化硬件抽象层 MavKit
    mav_ = std::make_unique<MavKit>(nh_);
    if (!mav_->mavInit(nh_)) {
        ROS_ERROR("MaxtCore: Failed to initialize MavKit!");
        return false;
    }

    // 2. 启动异步 Spinner (开启后台回调线程)
    // 使用 2 个线程：一个处理 MAVROS 消息，一个处理可能的其他服务
    spinner_ = std::make_unique<ros::AsyncSpinner>(2);
    spinner_->start();

    // 3. 注册行为树节点
    registerNodes();

    // 4. 加载行为树 XML
    try {
        tree_ = factory_.createTreeFromFile(xml_path_, mission_blackboard_);
    } catch (const std::exception& e) {
        ROS_ERROR("MaxtCore: Failed to load BT XML: %s", e.what());
        return false;
    }

    // 5. 添加标准控制台日志（可选）
    logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
    mission_status_pub_ = nh_.advertise<std_msgs::String>("mission_status", 1, true);
    publishMissionState("IDLE");

    ROS_INFO("MaxtCore: System initialized successfully.");
    return true;
}

void MaxtCore::publishMissionState(const std::string& state) {
    boost::property_tree::ptree snapshot;
    snapshot.put("state", state);
    snapshot.put("xml_path", xml_path_);
    snapshot.put("stamp", ros::WallTime::now().toSec());
    boost::property_tree::ptree active;
    // 遍历全部叶节点，支持并行任务；不把父级 Sequence 当作执行动作。
    if (state == "RUNNING") {
        BT::applyRecursiveVisitor(tree_.rootNode(), [&active](const BT::TreeNode* node) {
            if (node->type() != BT::NodeType::ACTION || node->status() != BT::NodeStatus::RUNNING) {
                return;
            }
            boost::property_tree::ptree entry;
            entry.put("uid", node->UID());
            entry.put("name", node->name());
            entry.put("type", node->registrationName());
            active.push_back(std::make_pair("", entry));
        });
    }
    // property_tree 的空数组会变成空字符串；没有运行节点时省略 active 字段。
    if (!active.empty()) {
        snapshot.add_child("active", active);
    }
    std::ostringstream stream;
    boost::property_tree::write_json(stream, snapshot, false);
    std_msgs::String message;
    message.data = stream.str();
    mission_status_pub_.publish(message);
}

void MaxtCore::run() {
    ros::Rate loop_rate(tick_rate_);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    ROS_INFO("MaxtCore: Starting BT execution...");

    // 主循环：只要 ROS 正常且行为树没运行结束/失败
    while (ros::ok() && status == BT::NodeStatus::RUNNING) {
        // 执行一次行为树 Tick
        status = tree_.tickRoot();
        publishMissionState(BT::toStr(status, false));

        // 但保留主线程的一些零散回调
        ros::spinOnce();
        
        loop_rate.sleep();
    }

    if (status == BT::NodeStatus::RUNNING) {
        publishMissionState("STOPPED");
    }

    if (status == BT::NodeStatus::SUCCESS) {
        ROS_INFO("MaxtCore: Mission Success!");
    } else if (status == BT::NodeStatus::FAILURE) {
        ROS_ERROR("MaxtCore: Mission Failed!");
    }
}

} // namespace maxt
