#ifndef FORAY_AUTO_AIM_GIMBAL_COMMAND_HPP
#define FORAY_AUTO_AIM_GIMBAL_COMMAND_HPP

namespace foray_auto_aim {

/// 云台角指令
/// 约定 yaw 向左为正 pitch 向下为正 与 DJI NED 不同 坐标系 x 前 y 左 z 上
struct GimbalCommand {
    double yaw;   ///< 偏航角 rad 向左为正
    double pitch; ///< 俯仰角 rad 向下为正
    bool valid;   ///< false 解算不可信 上层不得据此接管云台
};

} // namespace foray_auto_aim

#endif // FORAY_AUTO_AIM_GIMBAL_COMMAND_HPP
