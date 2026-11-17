// Test only: the real QR context and a harmless BT heartbeat. No MavKit object,
// flight BT registration, MAVROS service, setpoint or hardware is constructed.
#include <maxt_pkg/qr_survey_support.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float64.h>
#include <clocale>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
    std::setlocale(LC_CTYPE, "C.UTF-8");
    ros::init(argc, argv, "qr_ugv_dispatch_probe");
    ros::NodeHandle nh("~");
    auto context = std::make_shared<maxt::QrSurveyContext>(nh);
    auto start = nh.subscribe<std_msgs::String>("begin", 1,
        [context](const std_msgs::String::ConstPtr& message) {
            const auto split = message->data.find('|');
            if (split != std::string::npos)
                context->beginUgvMission(message->data.substr(0, split), message->data.substr(split+1));
        });
    auto heartbeat = nh.advertise<std_msgs::UInt64>("bt_ticks", 1);
    auto duration = nh.advertise<std_msgs::Float64>("bt_tick_seconds", 1);
    uint64_t count = 0;
    BT::BehaviorTreeFactory factory;
    factory.registerSimpleAction("Heartbeat", [&count, &heartbeat](BT::TreeNode&) {
        std_msgs::UInt64 message;
        message.data = ++count;
        heartbeat.publish(message);
        return BT::NodeStatus::SUCCESS;
    });
    auto tree = factory.createTreeFromText(
        "<root main_tree_to_execute='Test'><BehaviorTree ID='Test'><Heartbeat/></BehaviorTree></root>");
    ros::WallRate rate(100);
    while (ros::ok()) {
        const auto began = ros::SteadyTime::now();
        tree.tickRoot();
        tree.haltTree();
        std_msgs::Float64 elapsed;
        elapsed.data = (ros::SteadyTime::now()-began).toSec();
        duration.publish(elapsed);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
