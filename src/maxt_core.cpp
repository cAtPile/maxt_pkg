#include <maxt_pkg/maxt_core.hpp>

#include <maxt_pkg/maxt_nodes/takeoff_node.hpp>
#include <maxt_pkg/maxt_nodes/goto_node.hpp>

#include <maxt_pkg/maxt_nodes/waitstep_node.hpp>
#include <maxt_pkg/maxt_nodes/hover_node.hpp>
#include <maxt_pkg/maxt_nodes/round_node.hpp>
#include <maxt_pkg/maxt_nodes/connect_check_node.hpp>
#include <maxt_pkg/maxt_nodes/touch_down_node.hpp>
#include <maxt_pkg/maxt_nodes/quintic_nav_node.hpp>


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
        tree_ = factory_.createTreeFromFile(xml_path_);
    } catch (const std::exception& e) {
        ROS_ERROR("MaxtCore: Failed to load BT XML: %s", e.what());
        return false;
    }

    // 5. 添加标准控制台日志（可选）
    logger_ = std::make_unique<BT::StdCoutLogger>(tree_);

    ROS_INFO("MaxtCore: System initialized successfully.");
    return true;
}

void MaxtCore::registerNodes() {

    factory_.registerBuilder<TakeoffNode>("Takeoff", 
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<TakeoffNode>(name, config, *mav_);
        });

    factory_.registerBuilder<GoToNode>("GoTo",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<GoToNode>(name, config, *mav_);
        });

    factory_.registerBuilder<QuinticNavNode>("QuinticNav",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<QuinticNavNode>(name, config, *mav_);
        });

    factory_.registerBuilder<WaitStepNode>("WaitStep",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<WaitStepNode>(name, config, *mav_);
        });

    factory_.registerBuilder<HoverNode>("Hover",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<HoverNode>(name, config, *mav_);
        });

    factory_.registerBuilder<RoundNode>("Round", 
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<RoundNode>(name, config, *mav_);
        });

    // 注册连接检查节点
    factory_.registerBuilder<ConnectCheckNode>("ConnectCheck", 
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<ConnectCheckNode>(name, config, *mav_);
        });

    factory_.registerBuilder<TouchDownNode>("TouchDown",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<TouchDownNode>(name, config, *mav_);
        });

    ROS_INFO("MaxtCore: All BT Nodes registered.");
}

void MaxtCore::run() {
    ros::Rate loop_rate(tick_rate_);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    ROS_INFO("MaxtCore: Starting BT execution...");

    // 主循环：只要 ROS 正常且行为树没运行结束/失败
    while (ros::ok() && status == BT::NodeStatus::RUNNING) {
        // 执行一次行为树 Tick
        status = tree_.tickRoot();

        // 但保留主线程的一些零散回调
        ros::spinOnce();
        
        loop_rate.sleep();
    }

    if (status == BT::NodeStatus::SUCCESS) {
        ROS_INFO("MaxtCore: Mission Success!");
    } else if (status == BT::NodeStatus::FAILURE) {
        ROS_ERROR("MaxtCore: Mission Failed!");
    }
}

} // namespace maxt
