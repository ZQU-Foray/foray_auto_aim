#include "foray_auto_aim/aimer.hpp"

#include <cmath>
#include <limits>

namespace foray_auto_aim {

constexpr double kGravity = 9.8;            // 标准重力加速度（m/s²）
constexpr double kConvergeTolerance = 5e-3; // 5 mm：数值收敛，远小于小装甲板尺寸 0.135 m
constexpr double kAcceptableError = 7e-2; // 0.07 m：小装甲板半宽，落点在此内视为能命中
constexpr int kMaxIterations = 10; // 上限：硬实时环不允许无界循环；每次仅几次三角函数

double predict_hit_height(double d, double elevation, double bullet_speed) {
    const double t = d / (bullet_speed * std::cos(elevation));
    return d * std::tan(elevation) - 0.5 * kGravity * t * t;
}

GimbalCommand aim_at(const Eigen::Vector3d& xyz_in_gimbal, double bullet_speed) {
    const double x = xyz_in_gimbal.x();
    const double y = xyz_in_gimbal.y();
    const double z = xyz_in_gimbal.z();

    const double d = std::sqrt(x * x + y * y);
    // 水平距离趋近 0 几乎在上方或者下方 (实际不会发生 此处作为防御)
    if (d < 1e-6) {
        return {0.0, 0.0, false}; // 不接管云台（valid = false）
    }

    double elevation = -std::atan2(z, d); // 初始直瞄：仰角向上为正
    double best_elevation = elevation;
    double best_error =
        std::numeric_limits<double>::infinity(); // 初始化一个大的误差使得后面循环成立

    for (int i = 0; i < kMaxIterations; i++) {
        const double z_hit = predict_hit_height(d, elevation, bullet_speed);
        double error = z - z_hit;

        if (std::abs(error) < std::abs(best_error)) { // 记录最好的一组
            best_error = error;
            best_elevation = elevation;
        }
        if (std::abs(error) < kConvergeTolerance)
            break; // 达到收敛阈值即停（5 mm 远小于装甲板尺寸，够用）

        elevation += error / d; // 一阶修正：水平距离 d 上，仰角改 Δθ → 落点高度改 ≈ d·Δθ
    }

    GimbalCommand command;
    command.yaw = std::atan2(y, x);  // 弹道只影响竖直方向，yaw 不受重力影响
    command.pitch = -best_elevation; // ⭐ 输出约定：向下为正
    command.valid = std::abs(best_error) < kAcceptableError;

    return command;
}

} // namespace foray_auto_aim
