#include "foray_auto_aim/aimer.hpp"

#include <cmath>

namespace foray_auto_aim {

GimbalCommand aim_at(const Eigen::Vector3d& xyz_in_gimbal) {
    const double x = xyz_in_gimbal.x();
    const double y = xyz_in_gimbal.y();
    const double z = xyz_in_gimbal.z();

    const double horizontal_distance = std::sqrt(x * x + y * y);

    GimbalCommand command;
    command.yaw = std::atan2(y, x);
    // pitch 约定向下为正，而输入 z 轴朝上 → 取负
    command.pitch = -std::atan2(z, horizontal_distance);
    return command;
}

} // namespace foray_auto_aim
