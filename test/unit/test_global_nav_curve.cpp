#include <gtest/gtest.h>
#include <maxt_pkg/global_nav_curve.hpp>

namespace {
using Target = mavros_msgs::GlobalPositionTarget;

Target pose(double latitude, double longitude, float altitude) {
    Target target;
    target.coordinate_frame = Target::FRAME_GLOBAL_INT;
    target.latitude = latitude;
    target.longitude = longitude;
    target.altitude = altitude;
    return target;
}

double norm(const geometry_msgs::Vector3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

TEST(GlobalNavCurve, ExactEndpointsAndZeroBoundaryDerivatives) {
    maxt::GlobalNavCurve curve;
    const auto start = pose(31.2, 121.4, 20.0f);
    const auto goal = pose(31.2001, 121.4002, 25.0f);
    curve.setTarget(start, goal, 2.5, 1.2);
    const auto first = curve.sample(0.0);
    EXPECT_NEAR(first.latitude, start.latitude, 1e-12);
    EXPECT_NEAR(first.longitude, start.longitude, 1e-12);
    EXPECT_FLOAT_EQ(first.altitude, start.altitude);
    EXPECT_NEAR(norm(first.velocity), 0.0, 1e-12);
    EXPECT_NEAR(norm(first.acceleration_or_force), 0.0, 1e-12);
    for (double t : {curve.duration(), curve.duration() + 10.0}) {
        const auto last = curve.sample(t);
        EXPECT_DOUBLE_EQ(last.latitude, goal.latitude);
        EXPECT_DOUBLE_EQ(last.longitude, goal.longitude);
        EXPECT_FLOAT_EQ(last.altitude, goal.altitude);
        EXPECT_DOUBLE_EQ(norm(last.velocity), 0.0);
        EXPECT_DOUBLE_EQ(norm(last.acceleration_or_force), 0.0);
        EXPECT_EQ(last.coordinate_frame, Target::FRAME_GLOBAL_INT);
        EXPECT_EQ(last.type_mask, Target::IGNORE_YAW | Target::IGNORE_YAW_RATE);
    }
}

TEST(GlobalNavCurve, EnuDirectionsAndVelocityLimits) {
    for (const auto& goal : {pose(0.0, 0.001, 10.0f), pose(0.001, 0.0, 10.0f),
                             pose(0.0, 0.0, 20.0f), pose(0.001, 0.001, 20.0f)}) {
        maxt::GlobalNavCurve curve;
        curve.setTarget(pose(0.0, 0.0, 10.0f), goal, 2.5, 1.2);
        const auto mid = curve.sample(curve.duration() / 2.0);
        if (goal.longitude > 0.0) EXPECT_GT(mid.velocity.x, 0.0);
        if (goal.latitude > 0.0) EXPECT_GT(mid.velocity.y, 0.0);
        if (goal.altitude > 10.0f) EXPECT_GT(mid.velocity.z, 0.0);
        for (int i = 0; i <= 100; ++i) {
            const auto target = curve.sample(curve.duration() * i / 100.0);
            EXPECT_LE(norm(target.velocity), 2.5);
            EXPECT_LE(norm(target.acceleration_or_force), 1.2);
        }
    }
}

TEST(GlobalNavCurve, CrossingAntimeridianUsesShortPathAndConsistentVelocity) {
    maxt::GlobalNavCurve curve;
    curve.setTarget(pose(30.0, 179.9999, 10.0f), pose(30.0, -179.9999, 10.0f), 2.5, 1.2);
    EXPECT_LT(curve.duration(), 30.0);
    const double time = curve.duration() / 3.0;
    const double dt = 0.01;
    const auto before = curve.sample(time - dt);
    const auto after = curve.sample(time + dt);
    const auto mid = curve.sample(time);
    double distance, azimuth, end_azimuth;
    GeographicLib::Geodesic::WGS84().Inverse(before.latitude, before.longitude,
        after.latitude, after.longitude, distance, azimuth, end_azimuth);
    EXPECT_NEAR(distance / (2.0 * dt), norm(mid.velocity), 1e-4);
    EXPECT_GT(mid.velocity.x, 0.0);
    EXPECT_NEAR(mid.velocity.y, 0.0, 1e-5);
}

TEST(GlobalNavCurve, PreservesRelativeAltitudeFrame) {
    maxt::GlobalNavCurve curve;
    auto start = pose(31.2, 121.4, 4.0f);
    auto goal = pose(31.2001, 121.4, 7.0f);
    start.coordinate_frame = Target::FRAME_GLOBAL_REL_ALT;
    goal.coordinate_frame = Target::FRAME_GLOBAL_REL_ALT;
    curve.setTarget(start, goal, 2.5, 1.2);
    for (double t : {0.0, curve.duration() / 2.0, curve.duration()}) {
        EXPECT_EQ(curve.sample(t).coordinate_frame, Target::FRAME_GLOBAL_REL_ALT);
    }
    EXPECT_FLOAT_EQ(curve.sample(0.0).altitude, 4.0f);
    EXPECT_FLOAT_EQ(curve.sample(curve.duration()).altitude, 7.0f);
}

TEST(GlobalNavCurve, CoincidentGoalRemainsStationary) {
    maxt::GlobalNavCurve curve;
    const auto goal = pose(31.2, 121.4, 20.0f);
    curve.setTarget(goal, goal, 2.5, 1.2);
    const auto target = curve.sample(curve.duration() / 2.0);
    EXPECT_NEAR(target.latitude, goal.latitude, 1e-12);
    EXPECT_NEAR(target.longitude, goal.longitude, 1e-12);
    EXPECT_FLOAT_EQ(target.altitude, goal.altitude);
    EXPECT_DOUBLE_EQ(norm(target.velocity), 0.0);
    EXPECT_DOUBLE_EQ(norm(target.acceleration_or_force), 0.0);
}
} // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
