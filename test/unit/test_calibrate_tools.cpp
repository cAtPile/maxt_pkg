#include <iostream>
#include <sstream>

#include <gtest/gtest.h>
#include <maxt_pkg/calibrate_tools.hpp>

namespace {
constexpr double kTolerance = 1e-12;

// 使用明确的直角旋转结果，避免仅靠正逆变换抵消而遗漏方向错误。
class CalibrateToolsTest : public ::testing::Test {
protected:
    void SetUp() override {
        tools.setMap(2.0, 0.0);
        tools.setFact(0.0, 3.0);
    }

    maxt::CalibrateTools tools;
};

// 验证未设置标定数据时偏航偏移为零，正向和逆向变换都保持原坐标。
TEST(CalibrateTools, DefaultIsIdentity) {
    maxt::CalibrateTools tools;
    double x, y;
    EXPECT_DOUBLE_EQ(tools.offset(), 0.0);
    tools.transRaw(2.0, -3.0, x, y);
    EXPECT_DOUBLE_EQ(x, 2.0);
    EXPECT_DOUBLE_EQ(y, -3.0);
    tools.untransRaw(2.0, -3.0, x, y);
    EXPECT_DOUBLE_EQ(x, 2.0);
    EXPECT_DOUBLE_EQ(y, -3.0);
}

// 验证修改 map 或 fact 后会立即更新偏航偏移，后续旋转使用最新的标定结果。
TEST_F(CalibrateToolsTest, CalibrationUpdatesAfterEitherSetter) {
    EXPECT_NEAR(tools.offset(), std::acos(-1.0) / 2.0, kTolerance);
    tools.setMap(0.0, 2.0);
    EXPECT_NEAR(tools.offset(), 0.0, kTolerance);
    tools.setFact(3.0, 0.0);
    EXPECT_NEAR(tools.offset(), -std::acos(-1.0) / 2.0, kTolerance);
    double x, y;
    tools.transRaw(2.0, 3.0, x, y);
    EXPECT_NEAR(x, 3.0, kTolerance);
    EXPECT_NEAR(y, -2.0, kTolerance);
}

// 验证标量坐标正向旋转 90 度的方向正确，逆变换后可恢复原坐标。
TEST_F(CalibrateToolsTest, ScalarRotationAndInverse) {
    double x, y;
    tools.transRaw(2.0, 3.0, x, y);
    EXPECT_NEAR(x, -3.0, kTolerance);
    EXPECT_NEAR(y, 2.0, kTolerance);
    tools.untransRaw(x, y, x, y);
    EXPECT_NEAR(x, 2.0, kTolerance);
    EXPECT_NEAR(y, 3.0, kTolerance);
}

// 验证目标消息的位置、速度和加速度 XY 正向旋转正确，位置和速度可逆变换恢复。
// 同时确认高度、消息头、坐标系、掩码和偏航字段不变，逆变换保持加速度不变。
TEST_F(CalibrateToolsTest, PositionTargetRotatesSupportedFields) {
    mavros_msgs::PositionTarget target;
    target.header.frame_id = "world";
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    target.position.x = 2.0;
    target.position.y = 3.0;
    target.position.z = 4.0;
    target.velocity.x = -5.0;
    target.velocity.y = 6.0;
    target.velocity.z = 7.0;
    target.acceleration_or_force.x = 8.0;
    target.acceleration_or_force.y = -9.0;
    target.acceleration_or_force.z = 10.0;
    target.yaw = 0.4;
    target.yaw_rate = 0.5;
    const auto original = target;

    tools.transRaw(target);
    EXPECT_NEAR(target.position.x, -3.0, kTolerance);
    EXPECT_NEAR(target.position.y, 2.0, kTolerance);
    EXPECT_NEAR(target.velocity.x, -6.0, kTolerance);
    EXPECT_NEAR(target.velocity.y, -5.0, kTolerance);
    EXPECT_NEAR(target.acceleration_or_force.x, 9.0, kTolerance);
    EXPECT_NEAR(target.acceleration_or_force.y, 8.0, kTolerance);
    const auto acceleration = target.acceleration_or_force;

    tools.untransRaw(target);
    EXPECT_NEAR(target.position.x, original.position.x, kTolerance);
    EXPECT_NEAR(target.position.y, original.position.y, kTolerance);
    EXPECT_NEAR(target.velocity.x, original.velocity.x, kTolerance);
    EXPECT_NEAR(target.velocity.y, original.velocity.y, kTolerance);
    // 当前逆变换只处理位置和速度，加速度保持调用前的值。
    EXPECT_EQ(target.acceleration_or_force, acceleration);
    EXPECT_DOUBLE_EQ(target.position.z, original.position.z);
    EXPECT_DOUBLE_EQ(target.velocity.z, original.velocity.z);
    EXPECT_EQ(target.header, original.header);
    EXPECT_EQ(target.coordinate_frame, original.coordinate_frame);
    EXPECT_EQ(target.type_mask, original.type_mask);
    EXPECT_DOUBLE_EQ(target.yaw, original.yaw);
    EXPECT_DOUBLE_EQ(target.yaw_rate, original.yaw_rate);
}

// 验证位姿消息的 XY 正向旋转及逆变换正确，并保持高度、姿态和消息头不变。
TEST_F(CalibrateToolsTest, PosePreservesHeightOrientationAndHeader) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 2.0;
    pose.pose.position.y = 3.0;
    pose.pose.position.z = 4.0;
    pose.pose.orientation.z = 0.6;
    pose.pose.orientation.w = 0.8;
    const auto original = pose;

    tools.transPose(pose);
    EXPECT_NEAR(pose.pose.position.x, -3.0, kTolerance);
    EXPECT_NEAR(pose.pose.position.y, 2.0, kTolerance);
    tools.untransPose(pose);
    EXPECT_NEAR(pose.pose.position.x, original.pose.position.x, kTolerance);
    EXPECT_NEAR(pose.pose.position.y, original.pose.position.y, kTolerance);
    EXPECT_DOUBLE_EQ(pose.pose.position.z, original.pose.position.z);
    EXPECT_EQ(pose.pose.orientation, original.pose.orientation);
    EXPECT_EQ(pose.header, original.header);
}

// 验证速度消息的 XY 线速度逆向旋转正确，并保持 Z 线速度、角速度和消息头不变。
TEST_F(CalibrateToolsTest, TwistInversePreservesOtherFields) {
    geometry_msgs::TwistStamped twist;
    twist.header.frame_id = "map";
    twist.twist.linear.x = 2.0;
    twist.twist.linear.y = 3.0;
    twist.twist.linear.z = 4.0;
    twist.twist.angular.x = 0.1;
    twist.twist.angular.y = 0.2;
    twist.twist.angular.z = 0.3;
    const auto original = twist;

    tools.untransTwist(twist);
    EXPECT_NEAR(twist.twist.linear.x, 3.0, kTolerance);
    EXPECT_NEAR(twist.twist.linear.y, -2.0, kTolerance);
    EXPECT_DOUBLE_EQ(twist.twist.linear.z, original.twist.linear.z);
    EXPECT_EQ(twist.twist.angular, original.twist.angular);
    EXPECT_EQ(twist.header, original.header);
}

// 验证标定信息输出包含 map、fact 和角度值，便于人工检查当前标定结果。
TEST_F(CalibrateToolsTest, PrintReportsCalibration) {
    std::ostringstream output;
    tools.print(output);
    EXPECT_NE(output.str().find("map: (2, 0)"), std::string::npos);
    EXPECT_NE(output.str().find("fact: (0, 3)"), std::string::npos);
    EXPECT_NE(output.str().find("90 deg"), std::string::npos);
}
}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
