#include <behaviortree_cpp_v3/bt_factory.h>
#include <maxt_pkg/maxt_nodes/nav_node.hpp>
#include <maxt_pkg/maxt_nodes/nav_raw_node.hpp>
#include <maxt_pkg/maxt_nodes/la_lo_nav_node.hpp>
#include <maxt_pkg/maxt_nodes/azimuth_fix_nav_node.hpp>
#include <maxt_pkg/maxt_nodes/guidance_node.hpp>
#include <maxt_pkg/maxt_nodes/found_check_node.hpp>
#include <maxt_pkg/maxt_nodes/get_region_waypoint_node.hpp>
#include <maxt_pkg/maxt_nodes/traverse_path_generator_node.hpp>
#include <maxt_pkg/maxt_nodes/takeoff_node.hpp>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
void check(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}
struct Scenario {
    std::string mode;
    int legs{0}, halts{0}, guidance{0};
    bool searching{false}, landed{false};
    std::vector<std::string> events;
};
class Leaf : public BT::StatefulActionNode {
public:
    Leaf(const std::string& name, const BT::NodeConfiguration& config,
         std::string kind, Scenario& s)
        : BT::StatefulActionNode(name, config), kind_(std::move(kind)), s_(s) {}
    BT::NodeStatus onStart() override {
        ticks_ = 0;
        if (kind_ == "FoundCheck") {
            const bool detect = s_.mode != "none" && s_.searching &&
                (s_.legs == 1 || (s_.mode == "repeat" && s_.legs == 3) ||
                 (s_.mode == "sticky" && s_.halts > 0));
            return detect ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        if (name() == "SearchLeg") {
            ++s_.legs;
            s_.searching = true;
            s_.events.push_back("search");
            return BT::NodeStatus::RUNNING;
        }
        if (name().find("ReturnBreakpoint") == 0) {
            check(getInput<double>("x").value() == 1.25 &&
                  getInput<double>("y").value() == -2.5 &&
                  getInput<double>("z").value() == 4.75, "wrong breakpoint coordinates");
            s_.events.push_back(name());
            if ((s_.mode == "before_failed" && name() == "ReturnBreakpointBeforeGuidance") ||
                (s_.mode == "after_failed" && name() == "ReturnBreakpointAfterGuidance"))
                return BT::NodeStatus::FAILURE;
            return BT::NodeStatus::RUNNING;
        }
        if (kind_ == "Guidance") {
            ++s_.guidance;
            s_.events.push_back("guidance");
            return s_.mode == "guidance_failed" ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
        }
        if (kind_ == "Takeoff") {
            setOutput("home_x", 0.0); setOutput("home_y", 0.0);
        }
        if (kind_ == "BasePointAzimuthFix") {
            setOutput("bp_x_raw", 0.0); setOutput("bp_y_raw", 0.0); setOutput("bp_z_raw", 6.0);
        }
        if (kind_ == "TouchDown") s_.landed = true;
        return BT::NodeStatus::SUCCESS;
    }
    BT::NodeStatus onRunning() override {
        if (++ticks_ < 3) return BT::NodeStatus::RUNNING;
        if (name() == "SearchLeg") s_.searching = false;
        return BT::NodeStatus::SUCCESS;
    }
    void onHalted() override {
        if (name() != "SearchLeg") return;
        ++s_.halts; s_.searching = false;
        s_.events.push_back("halt");
        setOutput("breakpoint_x", 1.25); setOutput("breakpoint_y", -2.5);
        setOutput("breakpoint_z", 4.75);
        setOutput("breakpoint_valid", s_.mode != "invalid");
    }
private:
    std::string kind_;
    Scenario& s_;
    int ticks_{0};
};
void run(const std::string& path, const std::string& mode) {
    Scenario s; s.mode = mode;
    BT::BehaviorTreeFactory factory;
    const auto add = [&](const std::string& id, BT::PortsList ports) {
        factory.registerBuilder({BT::NodeType::ACTION, id, ports, ""},
            [&, id](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<Leaf>(name, config, id, s);
            });
    };
    add("Nav", maxt::NavNode::providedPorts());
    add("NavRaw", maxt::NavRawNode::providedPorts());
    add("LaLoNav", maxt::LaLoNavNode::providedPorts());
    add("AzimuthFixNav", maxt::AzimuthFixNavNode::providedPorts());
    add("Guidance", maxt::GuidanceNode::providedPorts());
    add("FoundCheck", maxt::FoundCheckNode::providedPorts());
    add("Takeoff", maxt::TakeoffNode::providedPorts());
    add("ConnectCheck", {}); add("ReHeading", {}); add("TouchDown", {});
    add("Hover", {BT::InputPort<double>("hover_duration")});
    add("BasePointAzimuthFix", {BT::OutputPort<double>("bp_x_raw"),
        BT::OutputPort<double>("bp_y_raw"), BT::OutputPort<double>("bp_z_raw")});
    factory.registerNodeType<maxt::TraversePathGeneratorNode>("TraversePathGenerator");
    factory.registerNodeType<maxt::GetRegionWaypointNode>("GetRegionWaypoint");
    auto tree = factory.createTreeFromFile(path);
    auto status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < 3000 && status == BT::NodeStatus::RUNNING; ++i) status = tree.tickRoot();
    const bool failed = mode == "invalid" || mode.find("failed") != std::string::npos;
    check(status == (failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS), mode + ": root status");
    check(s.landed == !failed, mode + ": landing status");
    const int expected = mode == "none" ? 0 : mode == "repeat" ? 2 : 1;
    check(s.halts == expected, mode + ": incorrect interruption count");
    if (!failed) check(s.legs == (path.find("sim_geo") != std::string::npos ? 11 : 12),
                       mode + ": remaining waypoints skipped or repeated");
    if (!failed && expected) {
        check(s.guidance == expected, mode + ": incorrect guidance count");
        for (size_t i = 0; i < s.events.size(); ++i) {
            if (s.events[i] != "halt") continue;
            check(i + 4 < s.events.size(), "no next waypoint after handling");
            check(s.events[i+1] == "ReturnBreakpointBeforeGuidance" &&
                  s.events[i+2] == "guidance" &&
                  s.events[i+3] == "ReturnBreakpointAfterGuidance" &&
                  s.events[i+4] == "search", "incorrect breakpoint/guidance/next-waypoint order");
        }
    }
    if (mode == "invalid" || mode == "before_failed") check(s.guidance == 0, "guidance started before valid return");
    if (failed) check(s.legs == 1, "advanced waypoint after failed handling");
    std::cout << "PASS " << path << " " << mode << '\n';
}
}
int main(int argc, char** argv) {
    try {
        ros::Time::init();
        check(argc == 3, "expected two mission paths");
        for (int i = 1; i < argc; ++i)
            for (const auto& mode : {"none", "found", "sticky", "repeat", "invalid", "before_failed", "guidance_failed", "after_failed"})
                run(argv[i], mode);
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
