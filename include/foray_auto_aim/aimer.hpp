#ifndef FORAY_AUTO_AIM_AIMER_HPP
#define FORAY_AUTO_AIM_AIMER_HPP

#include "foray_auto_aim/gimbal_command.hpp"
#include <Eigen/Dense>

namespace foray_auto_aim {

/// @brief 输入目标在云台系的位置，解算出云台对应指向的角度
/// @param xyz_in_gimbal 目标位置，云台系（x 前 / y 左 / z 上），单位：米
/// @param bullet_speed 子弹速度m/s
/// @return 云台角指令；yaw 向左为正、pitch 向下为正，单位：弧度
GimbalCommand aim_at(const Eigen::Vector3d& xyz_in_gimbal, double bullet_speed);

/// @brief 前向模型：给定仰角，预测弹丸到达水平距离 d 时的高度
/// @param horizontal_distance 水平距离（m）
/// @param elevation 仰角（rad，向上为正）
/// @param bullet_speed 弹速（m/s）
/// @return 落点高度（m）
double predict_hit_height(double horizontal_distance, double elevation, double bullet_speed);

} // namespace foray_auto_aim

#endif // FORAY_AUTO_AIM_AIMER_HPP
