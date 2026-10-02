#ifndef FORAY_AUTO_AIM_DETECTOR_HPP
#define FORAY_AUTO_AIM_DETECTOR_HPP

#include <cstdint>
#include <opencv2/core.hpp>
#include <vector>

namespace foray_auto_aim {

/// 装甲板规格 决定 PnP 用的真实尺寸
enum class ArmorType {
    Big,   ///< 大装甲板 英雄
    Small, ///< 小装甲板 步兵 哨兵
    Unknown, ///< 没有检测到
};

/// 一块装甲板的检测结果 像素级 未解算
/// 坐标系 图像像素坐标 单位 像素
struct ArmorDetection {
    std::vector<cv::Point2f> corners; ///< 4 个角点 顺序与 solver pnp 的点一致 后续关键点顺序看模型
    uint8_t id;                       ///< 机器人编号 1-5 未知用 0
    ArmorType type;                   ///< 装甲板规格
    double confidence;                ///< 置信度 0-1
};

/// @brief 由一帧图像检测装甲板 模型由视觉组长训练 部署在本仓
/// @param bgr_img BGR 图像
/// @return 本帧装甲板列表 空列表表示本帧无目标 
std::vector<ArmorDetection> detect(const cv::Mat& bgr_img);

} // namespace foray_auto_aim

#endif // FORAY_AUTO_AIM_DETECTOR_HPP
