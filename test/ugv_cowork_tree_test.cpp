#include <behaviortree_cpp_v3/bt_factory.h>
#include <maxt_pkg/maxt_nodes/capture_pose_node.hpp>
#include <maxt_pkg/maxt_nodes/connect_check_node.hpp>
#include <maxt_pkg/maxt_nodes/found_check_node.hpp>
#include <maxt_pkg/maxt_nodes/get_region_waypoint_node.hpp>
#include <maxt_pkg/maxt_nodes/guidance_node.hpp>
#include <maxt_pkg/maxt_nodes/nav_node.hpp>
#include <maxt_pkg/maxt_nodes/traverse_path_generator_node.hpp>
#include <maxt_pkg/maxt_nodes/takeoff_node.hpp>
#include <maxt_pkg/maxt_nodes/touch_down_node.hpp>
#include <maxt_pkg/maxt_nodes/ugv_cowork_node.hpp>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

struct Scenario {
    std::string name;
    int detections{0};
    int search_halts{0};
    int tracking_halts{0};
    int ugv_ticks{0};
    int tracking_starts{0};
    bool returned{false};
    bool landed{false};
};

// 使用真实端口和规划节点，替换有外部设备依赖的叶节点，验证任务树的控制语义。
class Leaf : public BT::StatefulActionNode {
public:
    Leaf(const std::string& name, const BT::NodeConfiguration& config,
         const std::string& kind, Scenario& scenario)
        : BT::StatefulActionNode(name, config), kind_(kind), scenario_(scenario) {}

    BT::NodeStatus onStart() override {
        ticks_ = 0;
        if (kind_ == "CapturePose") {
            setOutput("x", 1.25);
            setOutput("y", -0.75);
            setOutput("z", 4.5);
        } else if (kind_ == "FoundCheck") {
            ++scenario_.detections;
            return scenario_.name != "not_found" && scenario_.detections >= 3
                ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        } else if (kind_ == "Guidance") {
            tracking_ = getInput<double>("tracking_duration").value() > 0;
            if (tracking_) ++scenario_.tracking_starts;
            return BT::NodeStatus::RUNNING;
        } else if (kind_ == "Nav" && name() == "ReturnHome") {
            check(getInput<double>("x").value() == 1.25, "lost takeoff X");
            check(getInput<double>("y").value() == -0.75, "lost takeoff Y");
            check(getInput<double>("z").value() == 4.5, "lost return altitude");
            scenario_.returned = true;
            return scenario_.name == "return_failed" ? BT::NodeStatus::FAILURE
                                                       : BT::NodeStatus::SUCCESS;
        } else if (kind_ == "Nav") {
            check(getInput<double>("z").value() == 4.5, "lost subtree altitude");
            return BT::NodeStatus::RUNNING;
        } else if (kind_ == "UgvCowork") {
            check(getInput<bool>("follow_drone").value(), "follow mode missing");
            return BT::NodeStatus::RUNNING;
        } else if (kind_ == "TouchDown") {
            check(scenario_.returned, "landing before return");
            scenario_.landed = true;
        }
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning() override {
        ++ticks_;
        if (kind_ == "Nav") {
            if (scenario_.name == "search_failed") return BT::NodeStatus::FAILURE;
            return ticks_ >= 4 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
        }
        if (kind_ == "Guidance") {
            if (!tracking_) {
                return scenario_.name == "alignment_failed" ? BT::NodeStatus::FAILURE
                                                              : BT::NodeStatus::SUCCESS;
            }
            if (scenario_.name == "guidance_failed") return BT::NodeStatus::FAILURE;
            if (scenario_.name == "guidance_early_success" && scenario_.tracking_starts == 1)
                return BT::NodeStatus::SUCCESS;
            return BT::NodeStatus::RUNNING;
        }
        if (kind_ == "UgvCowork") {
            ++scenario_.ugv_ticks;
            if (scenario_.name == "ugv_failed") return BT::NodeStatus::FAILURE;
            return ticks_ >= 5 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        if (kind_ == "Nav") ++scenario_.search_halts;
        if (kind_ == "Guidance" && tracking_) ++scenario_.tracking_halts;
    }

private:
    std::string kind_;
    Scenario& scenario_;
    int ticks_{0};
    bool tracking_{false};
};

template <typename Node>
void registerLeaf(BT::BehaviorTreeFactory& factory, const std::string& id, Scenario& scenario) {
    factory.registerBuilder({BT::NodeType::ACTION, id, Node::providedPorts(), ""},
        [id, &scenario](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<Leaf>(name, config, id, scenario);
        });
}

void run(const std::string& xml, const std::string& name, const std::string& expected) {
    Scenario scenario;
    scenario.name = name;
    BT::BehaviorTreeFactory factory;
    registerLeaf<maxt::CapturePoseNode>(factory, "CapturePose", scenario);
    registerLeaf<maxt::ConnectCheckNode>(factory, "ConnectCheck", scenario);
    registerLeaf<maxt::TakeoffNode>(factory, "Takeoff", scenario);
    registerLeaf<maxt::FoundCheckNode>(factory, "FoundCheck", scenario);
    registerLeaf<maxt::NavNode>(factory, "Nav", scenario);
    registerLeaf<maxt::GuidanceNode>(factory, "Guidance", scenario);
    registerLeaf<maxt::UgvCoworkNode>(factory, "UgvCowork", scenario);
    registerLeaf<maxt::TouchDownNode>(factory, "TouchDown", scenario);
    factory.registerNodeType<maxt::TraversePathGeneratorNode>("TraversePathGenerator");
    factory.registerNodeType<maxt::GetRegionWaypointNode>("GetRegionWaypoint");
    auto blackboard = BT::Blackboard::create();
    auto tree = factory.createTreeFromFile(xml, blackboard);
    auto status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < 300 && status == BT::NodeStatus::RUNNING; ++i)
        status = tree.tickRoot();
    check(status == (name == "return_failed" ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS),
          name + ": wrong root status");
    check(blackboard->get<std::string>("cowork_result") == expected, name + ": wrong result");
    check(blackboard->get<std::string>("region_search_result") ==
              (name == "search_failed" ? "failed" : name == "not_found" ? "complete" : "found"),
          name + ": wrong subtree result");
    check(!blackboard->getAny("region_waypoints") &&
          !blackboard->getAny("region_waypoint_index"), "subtree internals leaked");
    check(scenario.returned && scenario.landed == (name != "return_failed"), name + ": return/landing missing");
    if (expected == "ugv_reached") {
        check(scenario.ugv_ticks >= 5, name + ": Guidance ended cowork early");
        check(scenario.tracking_halts == 1, name + ": tracking not halted on arrival");
    }
    if (name == "found") check(scenario.search_halts == 1, "search navigation not halted");
    if (name == "not_found") check(scenario.ugv_ticks == 0, "UGV started without detection");
    std::cout << "PASS " << name << '\n';
}
} // namespace

int main(int argc, char** argv) {
    try {
        ros::Time::init();
        check(argc == 2, "expected mission XML path");
        run(argv[1], "found", "ugv_reached");
        run(argv[1], "not_found", "not_found");
        run(argv[1], "alignment_failed", "alignment_failed");
        run(argv[1], "ugv_failed", "cowork_failed");
        run(argv[1], "guidance_failed", "cowork_failed");
        run(argv[1], "search_failed", "search_failed");
        run(argv[1], "return_failed", "ugv_reached");
        run(argv[1], "guidance_early_success", "ugv_reached");
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
