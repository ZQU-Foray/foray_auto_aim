#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "foray_auto_aim/target.hpp"

using namespace foray_auto_aim;

namespace {

constexpr double kR = 0.2; // 板到中心半径 米
constexpr int kArmorNum = 4;
constexpr double kSigma = 0.01; // 观测噪声标准差 米（1 cm）不是实际相机的 只是用于测试
constexpr int kFrames = 200;
constexpr double kDt = 0.01; // 100 Hz

// 重新计算一遍与c文件的计算实现隔离再验证
Eigen::Vector3d true_armor_xyz(const Eigen::Vector3d& center, double yaw, int id) {
    const double angle = yaw + id * 2.0 * M_PI / kArmorNum;
    return {center.x() - kR * std::cos(angle), center.y() - kR * std::sin(angle), center.z()};
}

} // namespace

TEST(Target, 静止的目标收敛并且误差小于观测误差) {
    // 车静止朝向 0
    const Eigen::Vector3d center{5.0, 0.0, 0.5};
    const double yaw = 0.0;
    const int id = 0;
    // 故意给偏离才能测出滤波效果
    const Eigen::Vector3d init_guess = center + Eigen::Vector3d(0.5, -0.3, 0.0);
    // 按照靶车的来 但可能不准确 以后续实际测试再调试数值 先保证ekf的逻辑和处理没问题
    // 顺序 x vx y vy z vz a w r l h
    // 位置 σ=0.5m 速度 σ=1m/s 朝向 σ=0.2rad 角速度 σ=0.1
    // r l h 按靶车标称已知 给极小方差
    Eigen::Matrix<double, 11, 1> p0_diag{{0.25}, {1.0},  {0.25}, {1.0},  {0.25}, {1.0},
                                         {0.04}, {0.01}, {1e-6}, {1e-6}, {1e-6}};
    Target target(init_guess, yaw, kR, p0_diag, kArmorNum); // 构造

    // 每一次跑 噪声都是这个固定的随机值
    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, kSigma);

    double sum_meas_err = 0.0; // 观测误差平方和
    double sum_est_err = 0.0;  // 估计误差平方和

    for (int i = 0; i < kFrames; ++i) {
        // 观测  =  真实位置加上噪声
        Eigen::Vector3d z = true_armor_xyz(center, yaw, id);
        z.x() += noise(gen);
        z.y() += noise(gen);
        z.z() += noise(gen);

        target.predict(kDt);  // 更新预测
        target.update(z, id); // 更新观测

        sum_meas_err += (z - true_armor_xyz(center, yaw, id))
                            .squaredNorm(); // z是全部加上观测噪声的读数
                                            // 另外一个是没有加噪声直接推出的装甲板数字
        sum_est_err +=
            (target.state().xyz - center).squaredNorm(); // 滤波器输出预测的中心减去真实中心
    }

    EXPECT_LT(std::sqrt(sum_est_err / kFrames),
              std::sqrt(sum_meas_err / kFrames)); // 估计的误差小于观测的误差
}
