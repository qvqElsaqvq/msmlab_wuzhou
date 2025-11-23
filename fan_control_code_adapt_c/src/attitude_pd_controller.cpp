//
// Created by msmlab on 2025/11/14.
//

#include "attitude_pd_controller.h"

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;

AttitudePDController::AttitudePDController(GyroScope& gyro, Fan& fan): gyro_(gyro), fan_(fan)
{
    std::cout << "[AttitudePDController] init" << std::endl;

    /* ====== 增益 ====== */
    Kp_ = Vec3(4000, 4000, 3700);
    Kd_ = Vec3(0, 100, 80);

    /* 外环：角度 → 角速度参考 */
    Kp_anging_ = Vec3(0.42, 0.42, 0.0);
    Kd_anging_ = Vec3(0.04, 0.04, 0.0);

    /* 内环：角速度 */
    Kp_rating_ = Vec3(41, 41, 0);
    Kd_rating_ = Vec3(0, 0, 0);
    Ki_rating_ = Vec3(0.16, 0.16, 0.0);

    dt_ = 0.02; // 50 Hz

    angleTarget_ = Vec3::Zero();
    qTarget_ = Quat::Identity();

    slew_ = 8.0;

    torque_x = 0.0;
    torque_y = 0.0;
    torque_z = 0.0;
}

Vec3 AttitudePDController::computeControl(const Quat& qCurrent, const Vec3& angleCurrentDeg, const Vec3& wBodyDeg)
{
    // 欧拉角误差（单位：deg）
    Vec3 angErrDeg = angleTarget_ - angleCurrentDeg;
    Vec3 wDps = wBodyDeg;

    /* 外环：角度 → 角速度参考 */
    Vec3 vRef = Kp_anging_.cwiseProduct(angErrDeg) - Kd_anging_.cwiseProduct(wDps);
    vRef = vRef.cwiseMax(Vec3(-1.0, -1.0, -3.0)).cwiseMin(Vec3(1.0, 1.0, 3.0));

    Vec3 er  = vRef - wDps;

    /* 积分 + 抗饱和 */
    intRate_ += er * dt_;
    Vec3 intMax(25, 25, 20);
    intRate_ = intRate_.cwiseMax(-intMax).cwiseMin(intMax);

    Vec3 edot = (er - prevEr_) / dt_;
    prevEr_   = er;

    /* 内环 PID */
    Vec3 tau;
    tau[0] = Kp_rating_[0] * er[0] + Ki_rating_[0] * intRate_[0] + Kd_rating_[0] * edot[0];
    tau[1] = Kp_rating_[1] * er[1] + Ki_rating_[1] * intRate_[1] + Kd_rating_[1] * edot[1];
    tau[2] = 0.0; // Z 后面单独算
    return tau;
}

void AttitudePDController::setAttitudeInBalancing(const Vec3& eulerAngleDeg)
{
    /*
     * 调平过程中使用的不进死循环的控制
     * X/Y：双环PID（欧拉角）
     * Z：滑膜PD（四元数）
     */

    angleTarget_ = eulerAngleDeg;
    qTarget_ = eulerZYXToQuat(eulerAngleDeg);

    /* 读当前姿态 */
    auto av = gyro_.getAngularVelocity();  // °/s
    auto at = gyro_.getAttitude();  // °
    Vec3 angleCurrentDeg(at.x, at.y, at.z);
    Vec3 wBodyDeg(av.x, av.y, av.z);

    Quat qCurrent = eulerZYXToQuat(angleCurrentDeg);

    /* X/Y 双环 PID */
    Vec3 tauCmd = computeControl(qCurrent, angleCurrentDeg, wBodyDeg);

    /* Z 滑模 PD */
    Quat qe = quatError(qTarget_, qCurrent);
    double e0 = qe.w();
    Vec3 eVec= qe.vec();  // [ex,ey,ez]
    double errDeg = rad2deg(2.0 * std::atan2(eVec.norm(), std::fabs(e0)));

    double kpz, kdz;
    if (errDeg >= 30.0)      kpz = 0.1;
    else if (errDeg <= 1.0)  kpz = 1.0;
    else                     kpz = 0.1 + (1.0 - 0.1) * (30.0 - errDeg) / 29.0;

    double wmag = wBodyDeg.norm();
    kdz = 1.0 + 0.25 * std::min(wmag / 6.0, 1.0) + kpz * 0.2;

    double Kpz = Kp_[2] * kpz;
    double Kdz = Kd_[2] * kdz;
    double tauZ = -Kpz * eVec.z() - Kdz * wBodyDeg.z();

    tauCmd[2] = tauZ;

    torque_x = tauCmd[0];
    torque_y = tauCmd[1];
    torque_z = tauCmd[2];
    // std::cout << "torque_x=" << torque_x << ", torque_y=" << torque_y << ", torque_z=" << torque_z << std::endl;

    // 软饱和
    // double tx = 0.5;
    // double ty = -0.5;
    double tx = std::tanh(tauCmd[0] / 1000.0);
    double ty = std::tanh(tauCmd[1] / 1000.0);
    double tz = std::tanh(tauCmd[2] / 1500.0);
    // std::cout << "tx: " << tx << ", ty: " << ty << ", tz: " << tz << std::endl;

    // 下发力矩
    fan_.sendTorque(static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz));
}

Vec3 AttitudePDController::getTorque()
{
    Vec3 current_torque;
    current_torque.x() = torque_x;
    current_torque.y() = torque_y;
    current_torque.z() = torque_z;
    return current_torque;
}
