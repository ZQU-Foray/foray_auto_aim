#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "foray_auto_aim/aimer.hpp"

using namespace foray_auto_aim;

//1
TEST(AimAt, 左前方四十五度) {
    Eigen::Vector3d position(1.0, 1.0, 0.0);

    const auto command = aim_at(position);

    constexpr double kTolerance = 1e-9;
    EXPECT_NEAR(command.yaw, M_PI / 4.0, kTolerance);
    EXPECT_NEAR(command.pitch, 0.0, kTolerance);
}
//2
TEST(AimAt, 正前方) {
    Eigen::Vector3d position(1.0, 0.0, 0.0);

    const auto command = aim_at(position);

    constexpr double kTolerance = 1e-9;
    EXPECT_NEAR(command.yaw, 0.0, kTolerance);
    EXPECT_NEAR(command.pitch, 0.0, kTolerance);
}
//3
TEST(AimAt, 前方上面四十五度) {
    Eigen::Vector3d position(1.0, 0.0, 1.0);

    const auto command = aim_at(position);

    constexpr double kTolerance = 1e-9;
    EXPECT_NEAR(command.yaw, 0.0, kTolerance);
    EXPECT_NEAR(command.pitch, -M_PI / 4.0, kTolerance);
}
//4
TEST(AimAt, 右前方四十五度) {
    Eigen::Vector3d position(1.0, -1.0, 0.0);

    const auto command = aim_at(position);

    constexpr double kTolerance = 1e-9;
    EXPECT_NEAR(command.yaw, -M_PI / 4.0, kTolerance);
    EXPECT_NEAR(command.pitch, 0.0, kTolerance);
}
