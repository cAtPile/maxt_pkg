#include <maxt_pkg/bt_logic_test_nodes.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <ros/ros.h>

#include <exception>
#include <string>

namespace {

/** 注册仅用于控制流验证的无副作用测试节点。 */
void registerTestNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<maxt::TestConditionNode>("TestCondition");
    factory.registerNodeType<maxt::TestTickConditionNode>("TestTickCondition");
    factory.registerNodeType<maxt::TestLogNode>("TestLog");
    factory.registerNodeType<maxt::TestRunningNode>("TestRunning");
    factory.registerNodeType<maxt::TestFlakyNode>("TestFlaky");
}

}  // namespace

/** 加载并运行纯日志行为树；不创建 MavKit，也不连接 MAVROS。 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "maxt_bt_logic_test_node");
    ros::NodeHandle private_nh("~");

    std::string xml_path;
    double tick_rate = 10.0;
    private_nh.param<std::string>("bt_xml_path", xml_path, "");
    private_nh.param<double>("bt_tick_rate", tick_rate, 10.0);

    if (xml_path.empty()) {
        ROS_FATAL("BT logic test: private parameter '~bt_xml_path' is required");
        return 2;
    }
    if (tick_rate <= 0.0) {
        ROS_FATAL("BT logic test: '~bt_tick_rate' must be greater than zero");
        return 2;
    }

    try {
        BT::BehaviorTreeFactory factory;
        registerTestNodes(factory);
        BT::Tree tree = factory.createTreeFromFile(xml_path);
        BT::StdCoutLogger transition_logger(tree);
        ros::Rate rate(tick_rate);

        ROS_INFO("BT logic test: starting '%s'", xml_path.c_str());
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        while (ros::ok() && status == BT::NodeStatus::RUNNING) {
            status = tree.tickRoot();
            ros::spinOnce();
            rate.sleep();
        }

        if (status == BT::NodeStatus::SUCCESS) {
            ROS_INFO("BT logic test: all logic checks completed successfully");
            return 0;
        }
        ROS_ERROR("BT logic test: behavior tree returned FAILURE");
        return 1;
    } catch (const std::exception& error) {
        ROS_FATAL("BT logic test: %s", error.what());
        return 2;
    }
}
