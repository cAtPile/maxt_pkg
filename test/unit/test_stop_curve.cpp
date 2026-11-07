#include <gtest/gtest.h>
#include <maxt_pkg/quintic_curve.hpp>
#include <algorithm>
#include <cmath>

// 覆盖零速、低速和高速停止；验证实际五次曲线不反向、终点静止，
// 且使用自适应时长后峰值减速度不超过约定限制。
TEST(StopCurve, StopsWithoutReversalAndRespectsAcceleration) {
    for (double velocity : {0.0, 0.1, -0.5, 3.0, -10.0}) {
        const double limit = 1.0;
        const double duration = std::max(1.0, 1.5 * std::abs(velocity) / limit);
        const double start = 7.0;
        const double end = start + 0.5 * velocity * duration;
        double c[6];
        maxt::QuinticCurve::solveAxis(start, velocity, 0.0, end, 0.0, 0.0, duration, c);
        EXPECT_NEAR(maxt::QuinticCurve::evalPos(c, duration), end, 1e-9);
        EXPECT_NEAR(maxt::QuinticCurve::evalVel(c, duration), 0.0, 1e-9);
        EXPECT_NEAR(maxt::QuinticCurve::evalAcc(c, duration), 0.0, 1e-9);
        for (int i = 0; i <= 1000; ++i) {
            const double t = duration * i / 1000.0;
            const double speed = maxt::QuinticCurve::evalVel(c, t);
            EXPECT_GE(speed * velocity, -1e-9);
            EXPECT_LE(std::abs(speed), std::abs(velocity) + 1e-9);
            EXPECT_LE(std::abs(maxt::QuinticCurve::evalAcc(c, t)), limit + 1e-9);
        }
    }
}
