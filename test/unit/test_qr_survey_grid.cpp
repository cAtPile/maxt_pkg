#include <gtest/gtest.h>
#include <maxt_pkg/qr_survey_support.hpp>
#include <set>
#include <limits>

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

TEST(QrSurveyGrid, SignedCornersCoverAllFourQuadrantsWithoutLeavingRectangle) {
    for (double sx : {-1.,1.}) for (double sy : {-1.,1.}) {
        const auto points = maxt::stopSurveyGrid(sx*5,sy*5,2);
        ASSERT_EQ(points.size(),16u);
        EXPECT_EQ(points[0], (std::array<double,2>{{0,0}}));
        EXPECT_EQ(points[1], (std::array<double,2>{{0,sy*2}}));
        EXPECT_EQ(points[3], (std::array<double,2>{{0,sy*5}}));
        EXPECT_EQ(points[4], (std::array<double,2>{{sx*2,sy*5}}));
        std::set<std::array<double,2>> unique(points.begin(),points.end());
        EXPECT_EQ(unique.size(),points.size());
        EXPECT_TRUE(unique.count({{sx*5,sy*5}}));
        for (const auto& p : points) {
            EXPECT_GE(p[0]*sx,0); EXPECT_LE(p[0]*sx,5);
            EXPECT_GE(p[1]*sy,0); EXPECT_LE(p[1]*sy,5);
        }
    }
}

TEST(QrSurveyGrid, HeadingMapsForwardAndLeftAndRightIntoRawLocalAxes) {
    const double pi = std::acos(-1.);
    const auto north = maxt::surveyOffsetToLocal(5,-5,pi/2);
    EXPECT_NEAR(north[0],5,1e-10); EXPECT_NEAR(north[1],5,1e-10);
    const auto west = maxt::surveyOffsetToLocal(5,-5,pi);
    EXPECT_NEAR(west[0],-5,1e-10); EXPECT_NEAR(west[1],5,1e-10);
    const auto south = maxt::surveyOffsetToLocal(5,-5,-pi/2);
    EXPECT_NEAR(south[0],-5,1e-10); EXPECT_NEAR(south[1],-5,1e-10);
}

TEST(QrSurveyGrid, InvalidAndOversizedSignedRangesFailBeforeAllocation) {
    EXPECT_THROW(maxt::stopSurveyGrid(5,0,1),std::invalid_argument);
    EXPECT_THROW(maxt::stopSurveyGrid(5,-5,-1),std::invalid_argument);
    EXPECT_THROW(maxt::stopSurveyGrid(std::numeric_limits<double>::infinity(),-5,1),std::invalid_argument);
    EXPECT_THROW(maxt::stopSurveyGrid(5,-5,std::numeric_limits<double>::quiet_NaN()),std::invalid_argument);
    EXPECT_THROW(maxt::stopSurveyGrid(5,-5,std::numeric_limits<double>::min()),std::invalid_argument);
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
