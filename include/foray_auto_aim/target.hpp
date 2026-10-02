#ifndef FORAY_AUTO_AIM_TARGET_HPP
#define FORAY_AUTO_AIM_TARGET_HPP

#include <Eigen/Dense>

namespace foray_auto_aim {

/// 整车状态11维度 参考sp
/// v vx y vy z vz a w r l h
/// a 车体的朝向角度 w 角速度 板到中心半径 l 板间半径差 h 板间高度差

struct TargetState {
    Eigen::Vector3d xyz;  ///< 车体中心 米
    Eigen::Vector3d vxyz; ///< 中心速度 米每秒
    double a;             ///< 朝向 弧度
    double w;             ///< 角速度 弧度每秒
    double r;             ///< 板到中心半径 米
    double l;             ///< 板间半径差 米 靶车等距时为 0
    double h;             ///< 板间高度差 米 靶车等高时为 0
};

/// 整车状态估计器 EKF 匀速模型 装甲板位置观测
class Target {
public:
    /// @param xyz0 初始中心位置 米
    /// @param a0 初始朝向 弧度 由第一次观测折算
    /// @param r0 板到中心半径 米
    /// @param p0_diag 初始协方差对角 11 维
    /// @param armor_num 装甲板数量 靶车 4
    Target(const Eigen::Vector3d& xyz0, double a0, double r0, 
            const Eigen::Matrix<double,11,1>& p0_diag, int armor_num);
    
    /// @brief 预测到当前时刻
    /// @param dt 距上次的时间 秒
    void predict(double dt);
    
    /// @brief 用一次装甲板观测修正
    /// @param xyz_measured 云台系下该装甲板的位置 米
    /// @param armor_id 装甲板编号 0-3
    void update(const Eigen::Vector3d& xyz_measured, int armor_id);

    /// @brief 当前估计
    TargetState state() const;

private:
    int armor_num_;  ///< 装甲板数量
    Eigen::Matrix<double, 11, 1> x_; ///< 状态 固定尺寸 无堆分配
    Eigen::Matrix<double, 11, 11> P_;  ///< 状态协方差 固定尺寸 无堆分配
    
    /// @brief 由状态计算第id块板子的位置
    Eigen::Vector3d h_armor_xyz(const Eigen::Matrix<double, 11, 1>& x, int armor_id) const;

};

}

#endif
