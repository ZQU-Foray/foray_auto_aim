#include "foray_auto_aim/aimer.hpp"

#include <cmath>

namespace foray_auto_aim {

GimbalCommand aim_at(const Eigen::Vector3d& xyz_in_gimbal, double bullet_speed) {
    // 弹道补偿待云台就绪后实现 该参数属接口预留
    (void)bullet_speed;

    const double x = xyz_in_gimbal.x();
    const double y = xyz_in_gimbal.y();
    const double z = xyz_in_gimbal.z();

    const double horizontal_distance = std::sqrt(x * x + y * y);

    // 水平距离趋零（目标几乎在正上方/正下方） 无意义
    // 1e-6 m 数值下限 视为不可信
    if (horizontal_distance < 1e-6) {
        return {0.0, 0.0, false};
    }

    GimbalCommand command;
    command.yaw = std::atan2(y, x);
    // pitch 约定向下为正，而输入 z 轴朝上 取负
    command.pitch = -std::atan2(z, horizontal_distance);
    command.valid = true; // 解算可信度判定待实现
    return command;
}

} // namespace foray_auto_aim
