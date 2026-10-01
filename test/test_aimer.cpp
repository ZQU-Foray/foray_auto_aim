#include <Eigen/Dense>
#include <cmath>
#include <gtest/gtest.h>

#include "foray_auto_aim/aimer.hpp"

using namespace foray_auto_aim;

// 1
TEST(AimAt, 左前方四十五度) {
    Eigen::Vector3d target(1.0, 1.0, 0.0);
    constexpr double bullet_speed = 25.0;
    const auto command = aim_at(target, bullet_speed);

    constexpr double kTolerance = 1e-9;
    EXPECT_NEAR(command.yaw, M_PI / 4.0, kTolerance); // 断言 paw
    EXPECT_LT(command.pitch, 0.0);                    // 断言pitch < 0
    EXPECT_GT(command.pitch, -5e-2); // 断言pitch 大于一个范围值域  小角度抬起
}
// 2
TEST(AimAt, 正前方) {
    Eigen::Vector3d target(1.0, 0.0, 0.0);
    constexpr double bullet_speed = 25.0;
    const auto command = aim_at(target, bullet_speed);

    constexpr double kTolerance = 1e-9;
    EXPECT_NEAR(command.yaw, 0.0, kTolerance);
    EXPECT_LT(command.pitch, 0.0);   // 断言pitch < 0
    EXPECT_GT(command.pitch, -5e-2); // 断言pitch 大于一个范围值域  小角度抬起
}
// 3
TEST(AimAt, 前方上面四十五度) {
    Eigen::Vector3d target(1.0, 0.0, 1.0);
    constexpr double bullet_speed = 25.0;
    const auto command = aim_at(target, bullet_speed);

    constexpr double kTolerance = 1e-9;
    EXPECT_NEAR(command.yaw, 0.0, kTolerance);
    EXPECT_LT(command.pitch, -M_PI / 4); // 断言pitch 比直瞄(45°)更抬 —— 弹道补偿生效
    EXPECT_GT(command.pitch, -M_PI / 2); // 断言pitch 但不会抬到 90°
}
// 4
TEST(AimAt, 右前方四十五度) {
    Eigen::Vector3d target(1.0, -1.0, 0.0);
    constexpr double bullet_speed = 25.0;
    const auto command = aim_at(target, bullet_speed);

    constexpr double kTolerance = 1e-9;
    EXPECT_NEAR(command.yaw, -M_PI / 4.0, kTolerance);
    EXPECT_LT(command.pitch, 0.0);   // 断言pitch < 0
    EXPECT_GT(command.pitch, -5e-2); // 断言pitch 大于一个范围值域  小角度抬起
}
// 5
TEST(AimAt, 命中自洽) {
    constexpr double bullet_speed = 25.0;
    const Eigen::Vector3d target(3.0, 1.0, 0.5);

    const auto command = aim_at(target, bullet_speed);
    ASSERT_TRUE(command.valid);

    const double d = std::sqrt(target.x() * target.x() + target.y() * target.y());
    const double hit = predict_hit_height(d, -command.pitch, bullet_speed); // ⭐ pitch 转回仰角
    EXPECT_NEAR(hit, target.z(), 5e-3);                                     // 落点 = 目标高度
}
// 6
TEST(AimAt, 弹速过低时不可信) {
    const Eigen::Vector3d target(5.0, 0.0, 0.0);
    const auto command = aim_at(target, 1.0); // 5 米飞 5 秒，下坠 122 米 → 无解
    EXPECT_FALSE(command.valid);
}
// 7
TEST(AimAt, 水平距离为0几乎在上方或者下方) {
    EXPECT_FALSE(aim_at({0, 0, 5}, 25.0).valid);
}
