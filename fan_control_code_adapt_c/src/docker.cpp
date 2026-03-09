//
// Created by mijiao on 2026/3/6.
//

#include "docker.h"
#include <iostream>

Docker::Docker(GyroScope& gyro, AttitudePDController &controller) : gyro_(gyro), controller_(controller) {
}

bool Docker::docking(CooperationDockData cooperation_dock_data) {
    RigidPose dock_pose, self_pose;
    bool dock_success = Nokov_GetPoseById(1, dock_pose);
    bool self_success = Nokov_GetPoseById(2, self_pose);
    if (!dock_success || !self_success) {
        std::cout << "[Dock Controller] Failed to get new dock pose" << std::endl;
        return false;
    }

    // 计算自身 yaw 角 (基于 gyro_util，0度为 X 轴，逆时针为正)
    Eigen::Quaterniond q_self(self_pose.qw, self_pose.qx, self_pose.qy, self_pose.qz);
    Eigen::Vector3d self_euler = gyro_util::eulerZYX_degFromQuat(q_self);
    double self_yaw_deg = self_euler.z();

    // 双方坐标连线向量
    double dx = dock_pose.x - self_pose.x;
    double dy = dock_pose.y - self_pose.y;

    // 根据要求：正方向在xy平面上，0度方向为y轴。
    // 标准 atan2(dy, dx) 是以 X 轴为 0 度。减去 90 度即可将 0 度基准对齐到 Y 轴。
    double target_yaw_rad = std::atan2(dy, dx) - M_PI / 2.0;
    double target_yaw_deg = gyro_util::wrapDeg180(target_yaw_rad * 180.0 / M_PI);

    // 计算 yaw 的角度差值 (相对控制量)
    double yaw_diff_deg = gyro_util::wrapDeg180(target_yaw_deg - self_yaw_deg);

    // 降低日志输出频率（每3秒输出一次）
    static int log_counter = 0;
    if (++log_counter >= 150) { // 50Hz * 3秒 = 150次
        std::cout << "[Dock Controller] target yaw: " << target_yaw_deg << " deg" << std::endl;
        std::cout << "[Dock Controller] yaw diff: " << yaw_diff_deg << " deg" << std::endl;
        log_counter = 0;
    }

    controller_.setAttitudeInBalancing({0.0, 0.0, gyro_.getAttitude().z + yaw_diff_deg});
    return true;
}
