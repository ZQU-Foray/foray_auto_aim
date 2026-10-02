#include "foray_auto_aim/target.hpp"

#include <cmath>

namespace foray_auto_aim {

namespace {

// 过程噪声 加速度方差和角加速度方差
// 测试的靶车不转动 角度速度极小 防止偏移(靶车测试阶段)
constexpr double v1 = 100.0; // 加速度方差
constexpr double v2 = 1e-6;  // 角速度方差

// 观测噪声 位置误差约 1cm → (0.01)² = 1e-4
// 现在测试的相机不同 待新相机标定后替换为实测值
constexpr double kMeasVar = 1e-4;

double limit_rad(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

} // namespace

Target::Target(const Eigen::Vector3d& xyz0, double a0, double r0,
               const Eigen::Matrix<double, 11, 1>& p0_diag, int armor_num)
    : armor_num_(armor_num) {
    x_.setZero();
    P_ = p0_diag.asDiagonal();
    x_[0] = xyz0.x();
    x_[2] = xyz0.y();
    x_[4] = xyz0.z();
    x_[6] = a0;
    x_[8] = r0;
    // 因为现在是靶车测试 1 3 5 7 为速度和角速度 暂时不考虑
    // 靶车的装甲板相对 h l 为0
}

void Target::predict(double dt) {
    // 1 F矩阵
    const double a = dt * dt * dt * dt / 4;
    const double b = dt * dt * dt / 2;
    const double c = dt * dt;
    // 参考sp的写法但是跑了clang之后 没那么好看
    Eigen::Matrix<double, 11, 11> F{
        {1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 1, dt, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};

    // Q
    Eigen::MatrixXd Q{{a * v1, b * v1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {b * v1, c * v1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, a * v1, b * v1, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, b * v1, c * v1, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, a * v1, b * v1, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, b * v1, c * v1, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, a * v2, b * v2, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, b * v2, c * v2, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

    // 预测
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

TargetState Target::state() const {
    TargetState s;
    s.xyz = Eigen::Vector3d(x_[0], x_[2], x_[4]);
    s.vxyz = Eigen::Vector3d(x_[1], x_[3], x_[5]);
    s.a = x_[6];
    s.w = x_[7];
    s.r = x_[8];
    s.l = x_[9];
    s.h = x_[10];
    return s;
}

// 计算出装甲板中心的坐标（考虑长短轴） 后续加入具体 l和h
Eigen::Vector3d Target::h_armor_xyz(const Eigen::Matrix<double, 11, 1>& x, int armor_id) const {
    const double angle =
        limit_rad(x[6] + armor_id * 2 * M_PI /
                             armor_num_); // angle 可以理解是朝向 分为0123个板子的 0 90 180 270
    bool use_l_h = (armor_num_ == 4) && (armor_id == 1 || armor_id == 3); // 判断是否需要使用l 和 h
    const double r = (use_l_h) ? x[8] + x[9] : x[8];
    const double armor_x = x[0] - r * std::cos(angle); // 计算整车的x坐标 根据装甲板的中心x推
    const double armor_y = x[2] - r * std::sin(angle);      // 同理计算整车y
    const double armor_z = (use_l_h) ? x[4] + x[10] : x[4]; // 长边稍微高一些

    return {armor_x, armor_y, armor_z};
}

} // namespace foray_auto_aim