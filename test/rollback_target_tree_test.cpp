#include <behaviortree_cpp_v3/bt_factory.h>
#include <maxt_pkg/maxt_nodes/recorded_targets_node.hpp>
#include <maxt_pkg/maxt_nodes/list_recorded_targets_node.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <maxt_pkg/maxt_nodes/traverse_path_generator_node.hpp>
#include <maxt_pkg/maxt_nodes/get_region_waypoint_node.hpp>
#include <map>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <limits>

namespace {
void check(bool ok) { if (!ok) throw std::runtime_error("rollback assertion failed"); }
struct Scenario {
    BT::Blackboard::Ptr history = BT::Blackboard::create();
    std::vector<std::string> events;
    bool nav_fail{false}, guidance_fail{false};
    int search_legs{0}, search_halts{0};
    bool search_fail{false}, visible{true};
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
class Search : public BT::StatefulActionNode {
    Scenario& s_;
public:
    Search(const std::string& n, const BT::NodeConfiguration& c, Scenario& s)
        : BT::StatefulActionNode(n, c), s_(s) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("nav_n"), BT::InputPort<double>("nav_t"),
                BT::InputPort<double>("nav_a"), BT::InputPort<double>("bp_x_raw"),
                BT::InputPort<double>("bp_y_raw"), BT::InputPort<double>("bp_z_raw"),
                BT::InputPort<double>("timeout")};
    }
    BT::NodeStatus onStart() override {
        ++s_.search_legs; s_.events.push_back("search");
        check(getInput<double>("nav_a").value() == 0.0);
        check(getInput<double>("bp_x_raw").value() == 10.0);
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override {
        return s_.search_fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
    void onHalted() override { ++s_.search_halts; }
};
class Detection : public BT::SyncActionNode {
    Scenario& s_;
public:
    Detection(const std::string& n, const BT::NodeConfiguration& c, Scenario& s)
        : BT::SyncActionNode(n, c), s_(s) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("topic"), BT::InputPort<bool>("record"),
                BT::OutputPort<std::string>("target_id")};
    }
    BT::NodeStatus tick() override {
        check(getInput<bool>("record").value());
        if (s_.visible) {
            const std::string id = s_.search_legs % 2 ? "A" : "B";
            std::map<std::string, geometry_msgs::PoseStamped> poses;
            s_.history->get("target_local_poses", poses);
            auto& p = poses[id]; p.header.stamp.sec = 1;
            p.pose.position.x = id == "A" ? 1 : 4;
            p.pose.position.y = p.pose.position.x + 1;
            p.pose.position.z = p.pose.position.x + 2;
            s_.history->set("target_local_poses", poses);
            setOutput("target_id", id);
        }
        return BT::NodeStatus::SUCCESS;
    }
};
void runTraverse(int mode, const std::string& xml) {
    Scenario s; s.visible = mode != 1;
    s.search_fail = mode == 2; s.nav_fail = mode == 3; s.guidance_fail = mode == 4;
    BT::BehaviorTreeFactory f;
    f.registerNodeType<maxt::TraversePathGeneratorNode>("TraversePathGenerator");
    f.registerNodeType<maxt::GetRegionWaypointNode>("GetRegionWaypoint");
    f.registerBuilder<Search>("NavAzimuth", [&](const std::string& n, const BT::NodeConfiguration& c) {
        return std::make_unique<Search>(n, c, s); });
    f.registerBuilder<Detection>("TargetCheck", [&](const std::string& n, const BT::NodeConfiguration& c) {
        return std::make_unique<Detection>(n, c, s); });
    f.registerBuilder<maxt::ListRecordedTargetsNode>("ListRecordedTargets", [&](const std::string& n, const BT::NodeConfiguration& c) {
        return std::make_unique<maxt::ListRecordedTargetsNode>(n, c, s.history, [] {
            geometry_msgs::PoseStamped pose; pose.header.stamp.sec = 1; return pose; }); });
    f.registerBuilder<maxt::GetRecordedTargetNode>("GetRecordedTarget", [&](const std::string& n, const BT::NodeConfiguration& c) {
        return std::make_unique<maxt::GetRecordedTargetNode>(n, c, s.history); });
    f.registerBuilder<Nav>("NavRaw", [&](const std::string& n, const BT::NodeConfiguration& c) {
        return std::make_unique<Nav>(n, c, s); });
    f.registerBuilder<Guidance>("Guidance", [&](const std::string& n, const BT::NodeConfiguration& c) {
        return std::make_unique<Guidance>(n, c, s); });
    f.registerSimpleAction("ConnectCheck", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; });
    f.registerSimpleAction("Takeoff", [](BT::TreeNode& n) {
        n.setOutput("home_x", 0.0); n.setOutput("home_y", 0.0);
        n.setOutput("home_z", 0.0); n.setOutput("return_z", 6.0);
        return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<double>("target_alt"), BT::OutputPort<double>("home_x"),
        BT::OutputPort<double>("home_y"), BT::OutputPort<double>("home_z"), BT::OutputPort<double>("return_z")});
    f.registerSimpleAction("ReHeading", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                           {BT::InputPort<double>("timeout")});
    f.registerSimpleAction("NavGeo", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                           {BT::InputPort<double>("latitude"), BT::InputPort<double>("longitude"), BT::InputPort<double>("timeout")});
    f.registerSimpleAction("BasePointAzimuthFix", [](BT::TreeNode& n) {
        n.setOutput("bp_x_raw", 10.0); n.setOutput("bp_y_raw", 0.0); n.setOutput("bp_z_raw", 0.0);
        return BT::NodeStatus::SUCCESS;
    }, {BT::OutputPort<double>("bp_x_raw"), BT::OutputPort<double>("bp_y_raw"), BT::OutputPort<double>("bp_z_raw")});
    f.registerSimpleAction("Nav", [&](BT::TreeNode& n) {
        check(n.getInput<double>("x").value() == 0.0);
        check(n.getInput<double>("z").value() == 6.0);
        s.events.push_back("home"); return BT::NodeStatus::SUCCESS;
    }, Nav::providedPorts());
    f.registerSimpleAction("TouchDown", [&](BT::TreeNode&) {
        s.events.push_back("land"); return BT::NodeStatus::SUCCESS;
    });
    auto bb = BT::Blackboard::create();
    auto tree = f.createTreeFromFile(xml, bb);
    auto status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < 30 && status == BT::NodeStatus::RUNNING; ++i) status = tree.tickRoot();
    check(status == (mode >= 2 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS));
    check(bb->get<std::string>("traverse_result") == (mode >= 2 ? "failed" : "complete"));
    check(s.search_halts == 0);
    std::vector<std::string> expected(mode == 2 ? 1 : 10, "search");
    if (mode == 0) {
        expected.insert(expected.end(), {"nav A", "guide A", "nav B", "guide B"});
    } else if (mode == 3) {
        expected.push_back("nav A");
    } else if (mode == 4) {
        expected.insert(expected.end(), {"nav A", "guide A"});
    }
    if (mode < 2) expected.insert(expected.end(), {"home", "land"});
    check(s.events == expected);
}
void testDistanceOrdering() {
    auto history = BT::Blackboard::create();
    std::map<std::string, geometry_msgs::PoseStamped> poses;
    for (const auto& entry : std::map<std::string, double>{{"A", 10}, {"B", 1}, {"C", -1}}) {
        auto& p = poses[entry.first];
        p.header.stamp.sec = 1;
        p.pose.position.x = entry.second;
    }
    geometry_msgs::PoseStamped origin; origin.header.stamp.sec = 1;
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<maxt::ListRecordedTargetsNode>("ListRecordedTargets",
        [&](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<maxt::ListRecordedTargetsNode>(
                name, config, history, [&] { return origin; });
        });
    auto bb = BT::Blackboard::create();
    auto tree = factory.createTreeFromText(R"(
        <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
          <ListRecordedTargets sort_by_distance="{sort}" target_ids="{ids}" target_count="{count}"/>
        </BehaviorTree></root>)", bb);
    history->set("target_local_poses", poses);
    bb->set("sort", true);
    check(tree.tickRoot() == BT::NodeStatus::SUCCESS);
    check(bb->get<std::vector<std::string>>("ids") == std::vector<std::string>({"B", "C", "A"}));
    check(bb->get<int>("count") == 3);
    origin.pose.position.x = 9;
    check(tree.tickRoot() == BT::NodeStatus::SUCCESS);
    check(bb->get<std::vector<std::string>>("ids") == std::vector<std::string>({"A", "B", "C"}));
    bb->set("sort", false);
    check(tree.tickRoot() == BT::NodeStatus::SUCCESS);
    check(bb->get<std::vector<std::string>>("ids") == std::vector<std::string>({"A", "B", "C"}));
    bb->set("sort", true);
    history->set("processed_qr", std::vector<std::string>{"A"});
    check(tree.tickRoot() == BT::NodeStatus::SUCCESS);
    check(bb->get<std::vector<std::string>>("ids") == std::vector<std::string>({"B", "C"}));
    origin.header.stamp.sec = 0;
    check(tree.tickRoot() == BT::NodeStatus::FAILURE);
    origin.header.stamp.sec = 1;
    poses["B"].pose.position.x = std::numeric_limits<double>::quiet_NaN();
    history->set("target_local_poses", poses);
    check(tree.tickRoot() == BT::NodeStatus::FAILURE);
    poses["B"].pose.position.x = 1;
    poses["B"].header.frame_id = "other";
    history->set("target_local_poses", poses);
    check(tree.tickRoot() == BT::NodeStatus::FAILURE);
    history->set("target_local_poses", std::map<std::string, geometry_msgs::PoseStamped>{});
    check(tree.tickRoot() == BT::NodeStatus::SUCCESS);
    check(bb->get<int>("count") == 0);
}
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
        return std::make_unique<maxt::ListRecordedTargetsNode>(n, c, s.history, [] {
            geometry_msgs::PoseStamped pose; pose.header.stamp.sec = 1; return pose; }); });
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
    testDistanceOrdering();
    const std::string xml = argc > 1 ? argv[1] : "config/mission/subtree/RollbackTargetTree.xml";
    for (int i = 0; i < 6; ++i) run(i, xml);
    const auto dir = xml.substr(0, xml.find_last_of('/') + 1);
    for (int i = 0; i < 5; ++i) runTraverse(i, dir + "../traverse_guidance_mission.xml");
    std::cout << "Distance ordering: PASS; RollbackTargetTree: 6 scenarios PASS; TraverseGuidanceMission: 5 scenarios PASS\n";
}
