#include <maxt_pkg/maxt_core.hpp>

#include <maxt_pkg/maxt_nodes/takeoff_node.hpp>
#include <maxt_pkg/maxt_nodes/goto_node.hpp>

#include <maxt_pkg/maxt_nodes/waitstep_node.hpp>
#include <maxt_pkg/maxt_nodes/hover_node.hpp>
#include <maxt_pkg/maxt_nodes/round_node.hpp>
#include <maxt_pkg/maxt_nodes/qr_detect_node.hpp>
#include <maxt_pkg/maxt_nodes/conditions_node.hpp>
#include <maxt_pkg/maxt_nodes/yolo_receiver_node.hpp>
#include <maxt_pkg/maxt_nodes/target_check2_node.hpp>
#include <maxt_pkg/maxt_nodes/connect_check_node.hpp>
#include <maxt_pkg/maxt_nodes/ring_center_receiver_node.hpp>
#include <maxt_pkg/maxt_nodes/deliver_node.hpp>
#include <maxt_pkg/maxt_nodes/all_deliver_node.hpp>
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

    // 2.5 预建立投放机构 publisher，避免临时建 pub 导致消息丢失
    {
        ros::NodeHandle nh;
        front_left_open_pub_  = nh.advertise<std_msgs::Empty>("/servo/front_left/open", 1);
        front_left_close_pub_ = nh.advertise<std_msgs::Empty>("/servo/front_left/close", 1);
        front_right_open_pub_ = nh.advertise<std_msgs::Empty>("/servo/front_right/open", 1);
        front_right_close_pub_= nh.advertise<std_msgs::Empty>("/servo/front_right/close", 1);
        back_left_open_pub_   = nh.advertise<std_msgs::Empty>("/servo/back_left/open", 1);
        back_left_close_pub_  = nh.advertise<std_msgs::Empty>("/servo/back_left/close", 1);
        back_right_open_pub_  = nh.advertise<std_msgs::Empty>("/servo/back_right/open", 1);
        back_right_close_pub_ = nh.advertise<std_msgs::Empty>("/servo/back_right/close", 1);
        all_open_pub_  = nh.advertise<std_msgs::Empty>("/servo/all/open", 1);
        all_close_pub_ = nh.advertise<std_msgs::Empty>("/servo/all/close", 1);
    }

    // 3. 注册行为树节点
    registerNodes();

    // 4. 加载行为树 XML
    try {
        tree_ = factory_.createTreeFromFile(xml_path_);
    } catch (const std::exception& e) {
        ROS_ERROR("MaxtCore: Failed to load BT XML: %s", e.what());
        return false;
    }

    // 初始化黑板的投放状态
    auto bb = tree_.rootBlackboard();
    bb->set("is_delivered_front_left", false);
    bb->set("is_delivered_front_right", false);
    bb->set("is_delivered_back_left", false);
    bb->set("is_delivered_back_right", false);

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

    factory_.registerBuilder<QRDetectNode>("QRDetect", 
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<QRDetectNode>(name, config, *mav_);
        });

    // 注册条件节点
    factory_.registerBuilder<CheckLandLeftNode>("CheckLandLeft", 
        [](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<CheckLandLeftNode>(name, config);
        });

    factory_.registerBuilder<CheckLandRightNode>("CheckLandRight", 
        [](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<CheckLandRightNode>(name, config);
        });

    // 注册目标比对条件节点
    factory_.registerBuilder<TargetCheck2Node>("TargetCheck2",
        [](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<TargetCheck2Node>(name, config);
        });

    // 注册YOLO接收节点
    factory_.registerBuilder<YoloReceiverNode>("YoloReceiver",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<YoloReceiverNode>(name, config, *mav_);
        });

    // 注册连接检查节点
    factory_.registerBuilder<ConnectCheckNode>("ConnectCheck", 
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<ConnectCheckNode>(name, config, *mav_);
        });

    // 注册圆环中心接收节点
    factory_.registerBuilder<RingCenterReceiverNode>("RingCenterReceiver",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<RingCenterReceiverNode>(name, config, *mav_);
        });

    // 注册全部门投放节点
    factory_.registerBuilder<AllDeliverNode>("AllDeliver",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<AllDeliverNode>(name, config, *mav_,
                all_open_pub_, all_close_pub_);
        });

    // 注册单仓投放节点
    factory_.registerBuilder<DeliverNode>("Deliver",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<DeliverNode>(name, config, *mav_,
                front_left_open_pub_, front_left_close_pub_,
                front_right_open_pub_, front_right_close_pub_,
                back_left_open_pub_, back_left_close_pub_,
                back_right_open_pub_, back_right_close_pub_);
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