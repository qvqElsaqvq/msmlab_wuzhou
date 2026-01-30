//
// Created by msmlab on 2025/11/14.
//

#include "gyro_scope.h"
#include <iostream>

GyroScope::GyroScope(msmserial::MsMSerial& msm_serial)
: ser_(msm_serial)
{
    std::cout << "[GyroScope] init" << std::endl;

    // 注册回调：0x05 为陀螺仪数据
    ser_.registerCallback(0x05, [this](const GyroScopeData& msg)
    {
        // 原始数据缩放：你当前逻辑是 /100.0
        setAngularVelocity(msg.wx / 100.0, msg.wy / 100.0, msg.wz / 100.0);
        setAttitude(msg.roll / 100.0, msg.pitch / 100.0, msg.yaw / 100.0);
    });
}

GyroScope::Vec3 GyroScope::getAngularVelocity() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return latestAngularVel_;
}

GyroScope::Vec3 GyroScope::getAttitude() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return latestAttitude_;
}

void GyroScope::setAngularVelocity(double wx, double wy, double wz)
{
    std::lock_guard<std::mutex> lk(mtx_);
    latestAngularVel_.x = wx;
    latestAngularVel_.y = wy;
    latestAngularVel_.z = wz;
}

void GyroScope::setAttitude(double roll, double pitch, double yaw)
{
    std::lock_guard<std::mutex> lk(mtx_);
    latestAttitude_.x = roll;
    latestAttitude_.y = pitch;
    latestAttitude_.z = yaw;
}

Eigen::Quaterniond GyroScope::getQuaternion() const
{
    Vec3 att;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        att = latestAttitude_;
    }
    // ZYX（yaw->pitch->roll），输入为 deg
    return gyro_util::quatFromEulerZYX_deg(att.x, att.y, att.z);
}

// 修正/规范：计算「动捕坐标系 -> 陀螺仪坐标系」转换四元数
// 约定：Q 表示“世界->机体”的姿态（或同一物理意义），则转换满足：Q_M2G = Q_G * inv(Q_M)
Eigen::Quaterniond GyroScope::calculateMocapToGyroQuat(const RigidPose& pose) {
    // 1) mocap 四元数（w,x,y,z）
    Eigen::Quaterniond mocap_quat(pose.qw, pose.qx, pose.qy, pose.qz);
    mocap_quat.normalize();

    // 2) gyro 四元数（由 latestAttitude_ 转出来）
    Eigen::Quaterniond gyro_quat = getQuaternion();
    gyro_quat.normalize();

    // 3) Q_M2G = Q_G * inv(Q_M)
    Eigen::Quaterniond mocap_to_gyro_quat = gyro_quat * mocap_quat.inverse();
    mocap_to_gyro_quat.normalize();

    return mocap_to_gyro_quat;
}
Eigen::Quaterniond GyroScope::averageQuaternions(
    const std::vector<Eigen::Quaterniond>& quats)
{
    // Markley quaternion averaging
    // Reference:
    // F. L. Markley et al., "Averaging Quaternions", Journal of Guidance, Control, and Dynamics

    if (quats.empty()) {
        // 安全兜底：没有样本时返回单位四元数
        return Eigen::Quaterniond::Identity();
    }

    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

    for (const auto& q_in : quats) {
        Eigen::Quaterniond q = q_in.normalized();

        // 统一到同一半球，避免 q 与 -q 抵消
        if (q.w() < 0.0) {
            q.coeffs() *= -1.0;   // coeffs 顺序是 [x y z w]
        }

        // 注意：这里用 [w x y z] 形式
        Eigen::Vector4d v;
        v << q.w(), q.x(), q.y(), q.z();

        A += v * v.transpose();
    }

    // 求最大特征值对应的特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(A);
    Eigen::Vector4d v_max = solver.eigenvectors().col(3);

    Eigen::Quaterniond q_avg(v_max(0), v_max(1), v_max(2), v_max(3));
    q_avg.normalize();
    return q_avg;
}
void GyroScope::updateMocapExtrinsic(const RigidPose& pose, bool received_pose)
{
    if (!received_pose || extrinsic_ready_) {
        return;
    }

    // 1) Mocap 姿态
    Eigen::Quaterniond qM(pose.qw, pose.qx, pose.qy, pose.qz);
    qM.normalize();

    // 2) Gyro 姿态（由陀螺 rpy 转来）
    Eigen::Quaterniond qG = getQuaternion();
    qG.normalize();

    // 3) 单帧外参：q_M2G_i = qG * inv(qM)
    Eigen::Quaterniond q_i = qG * qM.inverse();
    q_i.normalize();

    q_samples_.push_back(q_i);

    // 4) 达到采样数 → 固定外参
    if ((int)q_samples_.size() >= calib_required_N_) {
        q_M2G_ = averageQuaternions(q_samples_);
        extrinsic_ready_ = true;

        std::cout << "[GyroScope] Mocap extrinsic calibrated. q_M2G (w,x,y,z)=("
                  << q_M2G_.w() << ", "
                  << q_M2G_.x() << ", "
                  << q_M2G_.y() << ", "
                  << q_M2G_.z() << ")\n";
    }
}

bool GyroScope::isMocapExtrinsicReady() const
{
    return extrinsic_ready_;
}

Eigen::Quaterniond GyroScope::getMocapExtrinsicQuat() const
{
    return q_M2G_;
}


