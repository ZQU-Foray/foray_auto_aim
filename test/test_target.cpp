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
constexpr double kDt = 0.01; // 100 Hz 预测的hz 后面主程序需要调整

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

TEST(Target, 匀速的目标能估计出速度) {
    const Eigen::Vector3d center0{5.0, 0.0, 0.5};
    const Eigen::Vector3d v_true{0.0, 1.0, 0.0}; // y方向 1m/s移动
    const double yaw = 0.0;
    const int id = 0;

    const Eigen::Vector3d init_guess = center0 + Eigen::Vector3d(0.5, -0.3, 0.0);
    // 顺序 x vx y vy z vz a w r l h
    Eigen::Matrix<double, 11, 1> p0_diag{{0.25}, {1.0},  {0.25}, {1.0},  {0.25}, {1.0},
                                         {0.04}, {0.01}, {1e-6}, {1e-6}, {1e-6}};

    Target target(init_guess, yaw, kR, p0_diag, kArmorNum);

    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, kSigma);

    double sum_meas_err = 0.0;
    double sum_est_err = 0.0;
    double sum_v_err = 0.0;
    int n_tail = 0;

    for (int i = 0; i < kFrames; ++i) {
        // 真值 不加噪声
        Eigen::Vector3d center_now = {5.0, center0[1] + v_true[1] * (i * kDt),
                                      0.5}; // 模拟y方向上匀速 中心随着帧移动
        // 观测 装甲板加上噪声
        Eigen::Vector3d z = true_armor_xyz(center_now, yaw, id);
        z.x() += noise(gen);
        z.y() += noise(gen);
        z.z() += noise(gen);

        target.predict(kDt);  // 当前时刻的预测
        target.update(z, id); // 更新当前时刻的滤波后的值

        // 为了观测实验的结果只看后半段落
        if (i < kFrames / 2) {
            continue;
        }

        sum_meas_err += (z - true_armor_xyz(center_now, yaw, id)).squaredNorm();
        sum_est_err += (target.state().xyz - center_now).squaredNorm();
        sum_v_err = (target.state().vxyz - v_true).squaredNorm();

        ++n_tail;
    }

    const double v_rmse = std::sqrt(sum_v_err / n_tail);
    std::cout << "速度 RMSE = " << v_rmse << " m/s\n";
    EXPECT_LT(v_rmse, 0.05);
    EXPECT_LT(std::sqrt(sum_est_err / n_tail), std::sqrt(sum_meas_err / n_tail));
}
