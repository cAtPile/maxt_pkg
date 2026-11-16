#include <gtest/gtest.h>
#include <maxt_pkg/qr_survey_support.hpp>
#include <set>

TEST(QrSurveyGrid, IncludesAllNineStopsInSerpentineOrder) {
    const auto points = maxt::stopSurveyGrid(20, 20, 10);
    const maxt::RegionWaypoints expected = {
        {{0,0}}, {{0,10}}, {{0,20}}, {{10,20}}, {{10,10}},
        {{10,0}}, {{20,0}}, {{20,10}}, {{20,20}}};
    EXPECT_EQ(points, expected);
}

TEST(QrSurveyGrid, UnevenDimensionsIncludeEdgesWithoutDuplicateStops) {
    const auto points = maxt::stopSurveyGrid(21, 13, 10);
    EXPECT_EQ(points.size(), 12u);
    std::set<std::array<double, 2>> unique(points.begin(), points.end());
    EXPECT_EQ(unique.size(), points.size());
    EXPECT_TRUE(unique.count({{21,13}}));
    EXPECT_TRUE(unique.count({{0,0}}));
    EXPECT_THROW(maxt::stopSurveyGrid(0,20,10), std::invalid_argument);
    EXPECT_THROW(maxt::stopSurveyGrid(20,20,0), std::invalid_argument);
    EXPECT_THROW(maxt::stopSurveyGrid(10000,10000,1), std::invalid_argument);
}

TEST(QrSurveyHold, ExplicitRawPositionTargetHasNoVelocityOrAccelerationControl) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 3;
    pose.pose.position.z = 5;
    const auto hold = maxt::qrHoldTarget(pose, 0.4);
    using T = mavros_msgs::PositionTarget;
    EXPECT_EQ(hold.position.x, 3);
    EXPECT_EQ(hold.position.z, 5);
    EXPECT_FLOAT_EQ(hold.yaw, 0.4f);
    EXPECT_EQ(hold.type_mask & (T::IGNORE_PX|T::IGNORE_PY|T::IGNORE_PZ), 0);
    EXPECT_TRUE(hold.type_mask & T::IGNORE_VZ);
    EXPECT_TRUE(hold.type_mask & T::IGNORE_AFZ);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
