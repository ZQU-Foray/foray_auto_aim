#ifndef FORAY_AUTO_AIM_AIMER_HPP
#define FORAY_AUTO_AIM_AIMER_HPP

#include "foray_auto_aim/gimbal_command.hpp"
#include <Eigen/Dense>

namespace foray_auto_aim {

/// @brief 输入目标在云台系的位置，解算出云台对应指向的角度(直接输出 不含有弹道补偿)
/// @param xyz_in_gimbal 目标位置，云台系（x 前 / y 左 / z 上），单位：米
/// @return 云台角指令；yaw 向左为正、pitch 向下为正，单位：弧度
GimbalCommand aim_at(const Eigen::Vector3d& xyz_in_gimbal);

} // namespace foray_auto_aim

#endif // FORAY_AUTO_AIM_AIMER_HPP
