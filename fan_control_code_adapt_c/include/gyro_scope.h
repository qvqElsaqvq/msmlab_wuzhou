#ifndef FAN_CONTROL_CODE_ADAPT_C_GYRO_SCOPE_H
#define FAN_CONTROL_CODE_ADAPT_C_GYRO_SCOPE_H

#include <mutex>
#include <cmath>
#include <Eigen/Dense>
#include "serial.h"
#include "nokov_bridge.h"

/**
 * 统一约定：
 * - 欧拉角输入/输出均为角度制（deg）
 * - 欧拉角旋转顺序为 ZYX：yaw -> pitch -> roll
 * - quatFromEulerZYX_deg(roll, pitch, yaw) 生成的四元数与 eulerZYX_degFromQuat() 完全互逆（在无奇异情况下）
 */
namespace gyro_util {

// wrap 到 [-180, 180)
static inline double wrapDeg180(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg >= 180.0) deg -= 360.0;
    if (deg <  -180.0) deg += 360.0;
    return deg;
}

static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
static inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

// ZYX（yaw->pitch->roll），输入 roll/pitch/yaw（deg） -> Quaternion
static inline Eigen::Quaterniond quatFromEulerZYX_deg(double roll_deg,
                                                      double pitch_deg,
                                                      double yaw_deg) {
    const double r = deg2rad(roll_deg);
    const double p = deg2rad(pitch_deg);
    const double y = deg2rad(yaw_deg);

    const Eigen::AngleAxisd Rz(y, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd Ry(p, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd Rx(r, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = Rz * Ry * Rx;   // ZYX
    q.normalize();
    return q;
}

// Quaternion -> ZYX 欧拉角（deg）
// 返回向量顺序为 [roll, pitch, yaw]（deg），并 wrap 到 [-180,180)
    static inline Eigen::Vector3d eulerZYX_degFromQuat(const Eigen::Quaterniond& q_in) {
    const Eigen::Quaterniond qn = q_in.normalized();
    const double w = qn.w(), x = qn.x(), y = qn.y(), z = qn.z();

    // ZYX (yaw-pitch-roll)
    const double sinp = 2.0 * (w*y - z*x);
    const double pitch = std::asin(std::clamp(sinp, -1.0, 1.0));  // [-pi/2, pi/2]

    const double yaw  = std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
    const double roll = std::atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y));

    double roll_deg  = wrapDeg180(rad2deg(roll));
    double pitch_deg = wrapDeg180(rad2deg(pitch));
    double yaw_deg   = wrapDeg180(rad2deg(yaw));

    return Eigen::Vector3d(roll_deg, pitch_deg, yaw_deg);
}

} // namespace gyro_util


class GyroScope {
public:
    // ===== 现有接口 =====
    explicit GyroScope(msmserial::MsMSerial &msm_serial);

    struct Vec3 {
        double x{}, y{}, z{};
        Vec3() = default;
        Vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
    };

    [[nodiscard]] Vec3 getAngularVelocity() const;
    [[nodiscard]] Vec3 getAttitude() const;

    void setAngularVelocity(double wx, double wy, double wz);
    void setAttitude(double roll, double pitch, double yaw);

    Eigen::Quaterniond getQuaternion() const;
    Eigen::Quaterniond calculateMocapToGyroQuat(const RigidPose& pose);

    // ===== 外参标定接口（★ 新增，main 需要）=====
    void updateMocapExtrinsic(const RigidPose& pose, bool received_pose);
    bool isMocapExtrinsicReady() const;
    Eigen::Quaterniond getMocapExtrinsicQuat() const;

    // ===== 工具：四元数平均 =====
    static Eigen::Quaterniond averageQuaternions(
        const std::vector<Eigen::Quaterniond>& quats
    );

private:
    msmserial::MsMSerial& ser_;
    mutable std::mutex mtx_;

    Vec3 latestAngularVel_{};
    Vec3 latestAttitude_{};

    // ===== 外参标定内部状态（★ 新增）=====
    bool extrinsic_ready_ = false;
    Eigen::Quaterniond q_M2G_ = Eigen::Quaterniond::Identity();
    std::vector<Eigen::Quaterniond> q_samples_;
    int calib_required_N_ = 120;
};



#endif // FAN_CONTROL_CODE_ADAPT_C_GYRO_SCOPE_H
