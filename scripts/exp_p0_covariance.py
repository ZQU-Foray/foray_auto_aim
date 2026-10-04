#!/usr/bin/env python3
"""初值协方差对照实验：全维 1.0 vs 逐维给定。

用途：复现 `src/target.cpp` 里 11 维 EKF 的行为，证明"初值方差必须逐维给"。
     数字来源：2026-10-03 调试时的对照实验（现在固化成可复现脚本）。

跑法：python3 scripts/exp_p0_covariance.py
"""

import numpy as np

# ---- 与 src/target.cpp 保持一致的参数 ----
DT = 0.01  # 100 Hz
FRAMES = 200
SIGMA = 0.01  # 观测噪声标准差 米（1 cm）
V1 = 1.0  # 加速度方差（靶车不机动，比 sp 默认 100 小）
V2 = 1e-6  # 角加速度方差（靶车不转）
R_VAR = 1e-4  # 观测噪声方差 (0.01)^2
R_ARMOR = 0.2  # 板到中心半径 米
ARMOR_NUM = 4
CENTER = np.array([5.0, 0.0, 0.5])  # 静止靶车中心
YAW = 0.0
ARMOR_ID = 0
INIT_OFFSET = np.array([0.5, -0.3, 0.0])  # 初值故意给偏


def limit_rad(a):
    while a > np.pi:
        a -= 2 * np.pi
    while a < -np.pi:
        a += 2 * np.pi
    return a


def transition(dt):
    F = np.eye(11)
    for i in (0, 2, 4, 6):
        F[i, i + 1] = dt
    return F


def process_noise(dt):
    a, b, c = dt**4 / 4, dt**3 / 2, dt**2
    Q = np.zeros((11, 11))
    for i in (0, 2, 4):
        Q[i, i], Q[i, i + 1] = a * V1, b * V1
        Q[i + 1, i], Q[i + 1, i + 1] = b * V1, c * V1
    Q[6, 6], Q[6, 7] = a * V2, b * V2
    Q[7, 6], Q[7, 7] = b * V2, c * V2
    return Q


def h_armor(x, armor_id):
    """由状态算第 armor_id 块装甲板的位置（对应 h_armor_xyz）"""
    angle = limit_rad(x[6] + armor_id * 2 * np.pi / ARMOR_NUM)
    r = x[8]
    return np.array([x[0] - r * np.cos(angle), x[2] - r * np.sin(angle), x[4]])


def h_jacobian(x, armor_id):
    """观测雅可比 3x11（对应 h_jacobian）"""
    angle = limit_rad(x[6] + armor_id * 2 * np.pi / ARMOR_NUM)
    r = x[8]
    H = np.zeros((3, 11))
    H[0, 0] = H[1, 2] = H[2, 4] = 1.0
    H[0, 6], H[1, 6] = r * np.sin(angle), -r * np.cos(angle)
    H[0, 8], H[1, 8] = -np.cos(angle), -np.sin(angle)
    return H


def run(p0_diag, seed=42):
    """跑一次仿真，返回 (观测 RMSE, 估计 RMSE)"""
    F, Q = transition(DT), process_noise(DT)
    R = np.eye(3) * R_VAR
    I = np.eye(11)

    x = np.zeros(11)
    x[0], x[2], x[4] = CENTER + INIT_OFFSET  # 初值（故意偏）
    x[6], x[8] = YAW, R_ARMOR
    P = np.diag(p0_diag)

    rng = np.random.default_rng(seed)
    s_meas = s_est = 0.0
    for _ in range(FRAMES):
        truth = h_armor(
            np.array(
                [*CENTER[:1], 0, *CENTER[1:2], 0, CENTER[2], 0, YAW, 0, R_ARMOR, 0, 0]
            ),
            ARMOR_ID,
        )
        z = truth + rng.normal(0, SIGMA, 3)

        x = F @ x  # predict
        P = F @ P @ F.T + Q

        H = h_jacobian(x, ARMOR_ID)  # update
        K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
        x = x + K @ (z - h_armor(x, ARMOR_ID))
        x[6] = limit_rad(x[6])
        P = (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T

        s_meas += np.sum((z - truth) ** 2)
        s_est += np.sum((np.array([x[0], x[2], x[4]]) - CENTER) ** 2)
    return np.sqrt(s_meas / FRAMES), np.sqrt(s_est / FRAMES)


def main():
    p0_flat = np.ones(11)  # ① 全维都给 1.0（错的）
    p0_per_dim = np.array(
        [
            0.25,
            1.0,
            0.25,
            1.0,
            0.25,
            1.0,  # ② 逐维给（正确）
            0.04,
            0.01,
            1e-6,
            1e-6,
            1e-6,
        ]
    )

    m1, e1 = run(p0_flat)
    _, e2 = run(p0_per_dim)

    print(f"观测 RMSE（参考）      = {m1:.4f} m")
    print(f"① 初值全给 1.0         → 估计 RMSE = {e1:.4f} m  ← 朝向/半径被当未知，跑偏")
    print(f"② 逐维给（r/l/h 已知） → 估计 RMSE = {e2:.4f} m  ← 优于观测")
    print()
    print("结论：初值协方差必须逐维给；r/l/h 按靶车标称视为已知。")


if __name__ == "__main__":
    main()
