//
// Created by msmlab on 2025/11/14.
//

#include "attitude_pd_controller.h"

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;

Vec3 AttitudePDController::computeControlBeforeBalancing(const Quat& qCurrent,
                                       const Vec3& angleCurrentDeg,
                                       const Vec3& wBodyDeg)
{
    Vec3 angErrDeg = angleTarget_ - angleCurrentDeg;
    Vec3 wDps      = wBodyDeg;

    /* 外环：角度 → 角速度参考 */
    Vec3 vRef = Kp_anging_.cwiseProduct(angErrDeg) -
                Kd_anging_.cwiseProduct(wDps);
    vRef = vRef.cwiseMax(Vec3(-1, -1, -3)).cwiseMin(Vec3(1, 1, 3));

    Vec3 er  = vRef - wDps;

    /* 积分 + 抗饱和 */
    intRate_ += er * dt_;
    Vec3 intMax(25, 25, 20);
    intRate_ = intRate_.cwiseMax(-intMax).cwiseMin(intMax);

    Vec3 edot = (er - prevEr_) / dt_;
    prevEr_   = er;

    /* 内环 PID */
    Vec3 tau;
    tau[0] = Kp_rating_[0] * er[0] + Ki_rating_[0] * intRate_[0] +
             Kd_rating_[0] * edot[0];
    tau[1] = Kp_rating_[1] * er[1] + Ki_rating_[1] * intRate_[1] +
             Kd_rating_[1] * edot[1];
    tau[2] = 0; // Z 后面单独算
    return tau;
}

void AttitudePDController::setAttitudeInBalancing(const Vec3& eulerAngleDeg)
{
    angleTarget_ = eulerAngleDeg;
    qTarget_     = eulerZYXToQuat(eulerAngleDeg);

    /* 读当前姿态 */
    auto av = gyro_.getAngularVelocity();   // °/s
    auto at = gyro_.getAttitude();          // °
    Vec3 angleCurrentDeg(at.x, at.y, at.z);
    Vec3 wBodyDeg(av.x, av.y, av.z);

    Quat qCurrent = eulerZYXToQuat(angleCurrentDeg);

    /* X/Y 双环 PID */
    Vec3 tauCmd = computeControlBeforeBalancing(qCurrent, angleCurrentDeg, wBodyDeg);

    /* Z 滑模 PD */
    Quat qe   = quatError(qTarget_, qCurrent);
    double e0 = qe.w();
    Vec3  eVec= qe.vec();          // [ex,ey,ez]
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

    /* 软饱和 tanh */
    double tx = std::tanh(tauCmd[0] / 120.0);
    double ty = std::tanh(tauCmd[1] / 120.0);
    double tz = std::tanh(tauCmd[2] / 300.0);

    /* 下发 */
    fan_.setTorque(static_cast<float>(tx),
                   static_cast<float>(ty),
                   static_cast<float>(tz));
}