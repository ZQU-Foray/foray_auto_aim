#include <Eigen/Dense>
#include <cmath>
#include <gtest/gtest.h>

#include "foray_auto_aim/aimer.hpp"

using namespace foray_auto_aim;

namespace {

// 步兵/哨兵 17mm 弹速上限 25 m/s
// 当前实现为直瞄，不使用该参数；此处仅按接口传入
constexpr double kBulletSpeed = 25.0;

constexpr double kTolerance = 1e-9;

} // namespace

// 1 左前方 45°
TEST(AimAt, 左前方四十五度) {
    const Eigen::Vector3d position(1.0, 1.0, 0.0);

    const auto command = aim_at(position, kBulletSpeed);

    EXPECT_NEAR(command.yaw, M_PI / 4.0, kTolerance);
    EXPECT_NEAR(command.pitch, 0.0, kTolerance);
    EXPECT_TRUE(command.valid);
}

// 2 正前方
TEST(AimAt, 正前方) {
    const Eigen::Vector3d position(1.0, 0.0, 0.0);

    const auto command = aim_at(position, kBulletSpeed);

    EXPECT_NEAR(command.yaw, 0.0, kTolerance);
    EXPECT_NEAR(command.pitch, 0.0, kTolerance);
    EXPECT_TRUE(command.valid);
}

// 3 前上方 45°：pitch 向下为正 → 抬头为负
TEST(AimAt, 前上方四十五度) {
    const Eigen::Vector3d position(1.0, 0.0, 1.0);

    const auto command = aim_at(position, kBulletSpeed);

    EXPECT_NEAR(command.yaw, 0.0, kTolerance);
    EXPECT_NEAR(command.pitch, -M_PI / 4.0, kTolerance);
    EXPECT_TRUE(command.valid);
}

// 4 右前方 45°：yaw 向左为正 → 偏右为负
TEST(AimAt, 右前方四十五度) {
    const Eigen::Vector3d position(1.0, -1.0, 0.0);

    const auto command = aim_at(position, kBulletSpeed);

    EXPECT_NEAR(command.yaw, -M_PI / 4.0, kTolerance);
    EXPECT_NEAR(command.pitch, 0.0, kTolerance);
    EXPECT_TRUE(command.valid);
}

// 5 边界：目标几乎在正上方 → 水平距离退化 → 不可信
TEST(AimAt, 目标在正上方时不可信) {
    const Eigen::Vector3d position(0.0, 0.0, 5.0);

    const auto command = aim_at(position, kBulletSpeed);

    EXPECT_FALSE(command.valid);
}
