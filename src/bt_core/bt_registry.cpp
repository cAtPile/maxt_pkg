#include <maxt_pkg/maxt_core.hpp>
#include <maxt_pkg/maxt_nodes/play_audio_node.hpp>
#include <maxt_pkg/maxt_nodes/traverse_path_generator_node.hpp>
#include <maxt_pkg/maxt_nodes/polygon_coverage_wpg.hpp>
#include <maxt_pkg/maxt_nodes/get_region_waypoint_node.hpp>
#include <maxt_pkg/maxt_nodes/guidance_node.hpp>
#include <maxt_pkg/maxt_nodes/ugv_cowork_node.hpp>
#include <maxt_pkg/maxt_nodes/capture_pose_node.hpp>
#include <maxt_pkg/maxt_nodes/send_geo_pose_node.hpp>
#ifdef MAXT_ENABLE_QR_SURVEY
#include <maxt_pkg/maxt_nodes/qr_survey_setup_node.hpp>
#include <maxt_pkg/maxt_nodes/qr_capture_node.hpp>
#include <maxt_pkg/maxt_nodes/qr_finish_node.hpp>
#include <maxt_pkg/maxt_nodes/qr_hold_node.hpp>
#endif

#include <maxt_pkg/maxt_nodes/takeoff_node.hpp>

#include <maxt_pkg/maxt_nodes/waitstep_node.hpp>
#include <maxt_pkg/maxt_nodes/hover_node.hpp>
#include <maxt_pkg/maxt_nodes/round_node.hpp>
#include <maxt_pkg/maxt_nodes/connect_check_node.hpp>
#include <maxt_pkg/maxt_nodes/touch_down_node.hpp>

#include <maxt_pkg/maxt_nodes/nav/nav_node.hpp>
#include <maxt_pkg/maxt_nodes/nav/nav_raw_node.hpp>
#include <maxt_pkg/maxt_nodes/re_heading_node.hpp>
#include <maxt_pkg/maxt_nodes/nav/geo_to_local.hpp>
#include <maxt_pkg/maxt_nodes/nav/nav_geo_node.hpp>
#include <maxt_pkg/maxt_nodes/base_point_azimuth_fix_node.hpp>
#include <maxt_pkg/maxt_nodes/nav/nav_azimuth_node.hpp>
#include <maxt_pkg/maxt_nodes/found_check_node.hpp>
#include <maxt_pkg/maxt_nodes/target_check_node.hpp>
#include <maxt_pkg/maxt_nodes/recorded_targets_node.hpp>
#include <maxt_pkg/maxt_nodes/list_recorded_targets_node.hpp>

namespace maxt {

void MaxtCore::registerNodes() {
#ifdef MAXT_ENABLE_QR_SURVEY
    auto qr = std::make_shared<QrSurveyContext>(nh_);
    factory_.registerBuilder<QrSurveySetup>("QrSurveySetup",
        [this, qr](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<QrSurveySetup>(name, config, nh_, qr);
        });
    factory_.registerBuilder<QrCapture>("QrCapture",
        [this, qr](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<QrCapture>(name, config, *mav_, nh_, qr);
        });
    factory_.registerBuilder<QrFinish>("QrFinish",
        [this, qr](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<QrFinish>(name, config, *mav_, qr);
        });
    factory_.registerBuilder<QrHold>("QrHold",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<QrHold>(name, config, *mav_);
        });
#endif
    factory_.registerBuilder<SendGeoPoseNode>("SendGeoPose",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<SendGeoPoseNode>(name, config, nh_);
        });
    factory_.registerBuilder<BasePointAzimuthFixNode>("BasePointAzimuthFix",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<BasePointAzimuthFixNode>(name, config, *mav_);
        });
    factory_.registerBuilder<NavAzimuth>("NavAzimuth",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<NavAzimuth>(name, config, *mav_, nh_);
        });
    factory_.registerBuilder<PlayAudioNode>("PlayAudio",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<PlayAudioNode>(name, config, *mav_, nh_);
        });
    factory_.registerBuilder<CapturePoseNode>("CapturePose",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<CapturePoseNode>(name, config, *mav_);
        });
    factory_.registerBuilder<UgvCoworkNode>("UgvCowork",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<UgvCoworkNode>(name, config, *mav_);
        });
    factory_.registerBuilder<FoundCheckNode>("FoundCheck",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<FoundCheckNode>(name, config, nh_, mission_blackboard_);
        });
    factory_.registerBuilder<TargetCheckNode>("TargetCheck",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<TargetCheckNode>(name, config, nh_, *mav_, mission_blackboard_);
        });
    factory_.registerBuilder<ListRecordedTargetsNode>("ListRecordedTargets",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<ListRecordedTargetsNode>(
                name, config, mission_blackboard_, [this] { return mav_->GetRawPose(); },
                [this](const std::string& parameter) {
                    std::string value;
                    nh_.getParam(parameter, value);
                    return value;
                });
        });
    factory_.registerBuilder<GetRecordedTargetNode>("GetRecordedTarget",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<GetRecordedTargetNode>(name, config, mission_blackboard_);
        });
    factory_.registerNodeType<TraversePathGeneratorNode>("TraversePathGenerator");
    factory_.registerNodeType<PolygonCoverageWpG>("PolygonCoverageWpG");
    factory_.registerNodeType<GetRegionWaypointNode>("GetRegionWaypoint");

    factory_.registerBuilder<TakeoffNode>("Takeoff", 
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<TakeoffNode>(name, config, *mav_);
        });

    factory_.registerBuilder<Geo2Local>("Geo2Local",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<Geo2Local>(name, config, *mav_);
        });

    factory_.registerBuilder<NavGeo>("NavGeo",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<NavGeo>(name, config, *mav_, nh_);
        });

    factory_.registerBuilder<ReHeadingNode>("ReHeading",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<ReHeadingNode>(name, config, *mav_, nh_);
        });

    factory_.registerBuilder<NavNode>("Nav",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<NavNode>(name, config, *mav_, nh_);
        });

    factory_.registerBuilder<NavRawNode>("NavRaw",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<NavRawNode>(name, config, *mav_, nh_);
        });

    factory_.registerBuilder<WaitStepNode>("WaitStep",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<WaitStepNode>(name, config, *mav_);
        });

    factory_.registerBuilder<HoverNode>("Hover",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<HoverNode>(name, config, *mav_);
        });
    factory_.registerBuilder<Px4HoldNode>("Px4Hold",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<Px4HoldNode>(name, config, *mav_);
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

    factory_.registerBuilder<GuidanceNode>("Guidance",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<GuidanceNode>(name, config, *mav_, mission_blackboard_);
        });

    ROS_INFO("MaxtCore: All BT Nodes registered.");
}

} // namespace maxt
