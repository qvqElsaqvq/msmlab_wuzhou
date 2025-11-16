//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_MASS_CENTER_BALANCER_H
#define FAN_CONTROL_CODE_ADAPT_C_MASS_CENTER_BALANCER_H

#include <array>
#include <vector>
#include <deque>
#include <chrono>
#include <cmath>
#include <optional>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <ctime>

#include "attitude_pd_controller.h"
#include "gyro_scope.h"
#include "leadscrew_controller.h"
#include "wheel.h"

struct Attitude { double roll{}, pitch{}, yaw{}; };
struct AngularVel { double wx{}, wy{}, wz{}; };
struct WheelStatus { int id{}; double rpm{}; };
struct Torque { double tx{}, ty{}, tz{}; };

class MassCenterBalancer {
public:
    using Vec3 = Eigen::Vector3d;
    using Quat = Eigen::Quaterniond;

    explicit MassCenterBalancer(GyroScope& gyro, Fan& fan, LeadScrewController& lscrew_controller, Wheel& wheel, AttitudePDController& attitude_controller);

    /// XY 调平总入口
    void balance_xy_fan();
    /**
     * @brief XY调平控制主循环,目标：通过旋翼闭环把姿态收敛到 [0,0,0]，在稳态下采样控制输出(Tx,Ty)，
     * @brief 将其与质量块移动步数建立比例关系，移动X/Y质量块；循环直至稳态输出足够小。
     * @brief 稳态判据：角度误差连续 dwell_time 内均 < tol_deg
     * @brief 采样窗口：sample_time 内平均 |Tx|, |Ty|
     * @brief 步长换算：steps = clip(k_xy * torque, [min_step, max_step])，方向取 “反向补偿”
     */
    void balance_both_axes_fan();
    /**
     * @brief Z调平控制主循环
     * @brief 假定 X/Y 已调平,将姿态分别设为 [0,15,0] [0,20,0] [0,30,0]，待稳态后采样所需的维持力矩；
     * @brief 将该回升力矩与 Z 轴质量块位移建立保守比例关系，移动后等待；
     * @brief 循环至回升力矩低于阈值。全程限制跨越稳定边界：方向翻转则自动减半步长
     */
    void balance_z_axes_fan();
    /// 启动丝杆电机
    void activate_motors();
    /// 关闭丝杆电机
    void deactivate_motors();
    /**
     * @brief 等待稳态 + 采样
     * @brief 等到 roll/pitch/yaw 连续 dwell_time 内都满足误差 < tol_deg；
     * @brief 然后在 sample_time 内采样控制器输出的 Tx、Ty（通过 compute_control 即时计算），返回 (Tx_mean, Ty_mean)
     */
    void wait_steady_and_sample_outputs();

private:
    GyroScope gyro_;
    LeadScrewController leadscrew_;
    Wheel wheel_;
    Fan fan_;
    AttitudePDController controller_;

    // 配置参数
    const std::array<uint8_t,3> axis_ids_{0x04,0x05,0x06};
    double tol_deg_;
    /// 稳态保持计时时间
    double dwell_time_;
    /// 采样时间
    double sample_time_;
    /// 控制循环频率
    double settle_wait_;
    /// 等待的时间阈值
    double waiting_time_;

    double last_roll_, last_pitch_, last_yaw_;

    /* 力矩平均值 */
    double tx_mean_;
    double ty_mean_;

    /* 时间相关起始点，未计时为-1 */
    double t_enter_;  // 判断稳态进入时间
    double sample_t_enter_;  // 开始采样进入时间
    double waiting_t_enter_;  // 判断进入等待的时间
    double x_err_angle_t_enter_;
    double y_err_angle_t_enter_;
    double z_err_angle_t_enter_;

    /* xy轴调平相关 */
    double x_target_angle_;
    double x_err_angle_mean_;
    double x_err_angle_t_threshold_;
    double y_target_angle_;
    double y_err_angle_mean_;
    double y_err_angle_t_threshold_;
    double z_target_angle_;
    double z_err_angle_mean_;
    double z_err_angle_t_threshold_;

    int z_period_cnt_; // 调平判断周期计数
    int z_period_threshold_; // 调平判断周期数阈值
    int xy_balancing_cnt_;
    int xy_balancing_cnt_threshold_;

    /* 调平相关标志位 */
    bool if_in_steady_state_;  // 是否在稳态状态
    bool waiting_after_moving_;  // 每次移动质量块后等待5s
    bool if_begin_sampling_; // 是否开始采样
    bool if_end_sampling_;  // 是否结束采样
    bool if_finish_balancing_;  // 调平是否结束

    /* z轴调平相关 */
    bool if_return_zero_;
    bool if_finish_testing_ty_;
    bool is_balancing_z_;
    bool pitch_metric_finish_; // 回升力矩是否已经消除
    bool if_15_ok_;
    bool if_20_ok_;

    std::vector<double> x_err_angle_;
    std::vector<double> y_err_angle_;
    std::vector<double> z_err_angle_;

    /// x力矩采样值
    std::vector<double> xs_;
    /// y力矩采样值
    std::vector<double> ys_;
};

#endif //FAN_CONTROL_CODE_ADAPT_C_MASS_CENTER_BALANCER_H