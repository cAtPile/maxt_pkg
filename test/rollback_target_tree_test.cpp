#include <behaviortree_cpp_v3/bt_factory.h>
#include <maxt_pkg/maxt_nodes/recorded_targets_node.hpp>
#include <maxt_pkg/maxt_nodes/list_recorded_targets_node.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <stdexcept>
#include <vector>
#include <iostream>

namespace {
void check(bool ok) { if (!ok) throw std::runtime_error("rollback assertion failed"); }
struct Scenario {
    BT::Blackboard::Ptr history = BT::Blackboard::create();
    std::vector<std::string> events;
    bool nav_fail{false}, guidance_fail{false};
};
class Nav : public BT::SyncActionNode {
    Scenario& s_;
public:
    Nav(const std::string& n, const BT::NodeConfiguration& c, Scenario& s)
        : BT::SyncActionNode(n, c), s_(s) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("x"), BT::InputPort<double>("y"),
                BT::InputPort<double>("z"), BT::InputPort<double>("timeout")};
    }
    BT::NodeStatus tick() override {
        double x = getInput<double>("x").value();
        check(getInput<double>("y").value() == x + 1);
        check(getInput<double>("z").value() == x + 2);
        s_.events.push_back(x == 1 ? "nav A" : "nav B");
        return s_.nav_fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
};
class Guidance : public BT::SyncActionNode {
    Scenario& s_;
public:
    Guidance(const std::string& n, const BT::NodeConfiguration& c, Scenario& s)
        : BT::SyncActionNode(n, c), s_(s) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("target_id"), BT::InputPort<double>("timeout"),
                BT::InputPort<double>("tracking_duration"), BT::OutputPort<std::string>("message")};
    }
    BT::NodeStatus tick() override {
        const auto id = getInput<std::string>("target_id").value();
        s_.events.push_back("guide " + id);
        if (s_.guidance_fail) return BT::NodeStatus::FAILURE;
        std::vector<std::string> processed;
        s_.history->get("processed_qr", processed);
        processed.push_back(id);
        s_.history->set("processed_qr", processed);
        setOutput("message", std::string("done"));
        return BT::NodeStatus::SUCCESS;
    }
};
void run(int mode, const std::string& xml) {
    Scenario s;
    s.nav_fail = mode == 3;
    s.guidance_fail = mode == 4;
    std::map<std::string, geometry_msgs::PoseStamped> poses;
    if (mode != 1) {
        for (const auto& id : {"B", "A"}) {
            auto& p = poses[id]; p.header.stamp.sec = 1;
            p.pose.position.x = std::string(id) == "A" ? 1 : 4;
            p.pose.position.y = p.pose.position.x + 1;
            p.pose.position.z = p.pose.position.x + 2;
        }
        if (mode == 5) poses["A"].header.stamp.sec = 0;
        s.history->set("target_local_poses", poses);
    }
    if (mode == 2) s.history->set("processed_qr", std::vector<std::string>{"A"});
    BT::BehaviorTreeFactory f;
    f.registerBuilder<maxt::ListRecordedTargetsNode>("ListRecordedTargets", [&](const std::string& n, const BT::NodeConfiguration& c) {
        return std::make_unique<maxt::ListRecordedTargetsNode>(n, c, s.history); });
    f.registerBuilder<maxt::GetRecordedTargetNode>("GetRecordedTarget", [&](const std::string& n, const BT::NodeConfiguration& c) {
        return std::make_unique<maxt::GetRecordedTargetNode>(n, c, s.history); });
    f.registerBuilder<Nav>("NavRaw", [&](const std::string& n, const BT::NodeConfiguration& c) {
        return std::make_unique<Nav>(n, c, s); });
    f.registerBuilder<Guidance>("Guidance", [&](const std::string& n, const BT::NodeConfiguration& c) {
        return std::make_unique<Guidance>(n, c, s); });
    f.registerBehaviorTreeFromFile(xml);
    auto bb = BT::Blackboard::create(); bb->set("tracking_duration", 0.0);
    auto tree = f.createTree("RollbackTargetTree", bb);
    const bool failed = mode >= 3;
    check(tree.tickRoot() == (failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS));
    check(bb->get<std::string>("result") == (failed ? "failed" : "complete"));
    const std::vector<std::vector<std::string>> expected{
        {"nav A", "guide A", "nav B", "guide B"}, {}, {"nav B", "guide B"},
        {"nav A"}, {"nav A", "guide A"}, {}};
    check(s.events == expected[mode]);
    if (!failed) {
        s.events.clear(); check(tree.tickRoot() == BT::NodeStatus::SUCCESS);
        check(s.events.empty()); // 再次执行跳过所有已完成目标。
    }
}
} // namespace
int main(int argc, char** argv) {
    const std::string xml = argc > 1 ? argv[1] : "config/mission/subtree/RollbackTargetTree.xml";
    for (int i = 0; i < 6; ++i) run(i, xml);
    std::cout << "RollbackTargetTree: 6 scenarios PASS\n";
}
