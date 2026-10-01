#ifndef FORAY_AUTO_AIM_AIMER_HPP
#define FORAY_AUTO_AIM_AIMER_HPP

#include "foray_auto_aim/gimbal_command.hpp"
#include <Eigen/Dense>

namespace foray_auto_aim {

/// @brief 由目标在云台系下的位置解算云台指向角（当前为直瞄几何解算）
/// @param xyz_in_gimbal 目标位置，云台系（x 前 / y 左 / z 上），单位：米
/// @param bullet_speed 弹速（m/s）—— 弹道补偿待云台就绪后实现，当前实现为直瞄，暂不使用该参数
/// @return 云台角指令；yaw 向左为正、pitch 向下为正，单位：弧度
GimbalCommand aim_at(const Eigen::Vector3d& xyz_in_gimbal, double bullet_speed);

} // namespace foray_auto_aim

#endif // FORAY_AUTO_AIM_AIMER_HPP
