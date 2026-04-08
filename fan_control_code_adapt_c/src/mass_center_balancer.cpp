//
// Created by msmlab on 2025/11/14.
//

#include "mass_center_balancer.h"

using namespace std::chrono;

MassCenterBalancer::MassCenterBalancer(GyroScope &gyro, Fan &fan, LeadScrewController &lscrew_controller,
                                       Wheel &wheel, AttitudePDController &attitude_controller) : gyro_(gyro),
    leadscrew_(lscrew_controller), wheel_(wheel), fan_(fan), controller_(attitude_controller) {
    std::cout << "[MassCenterBalancer] init" << std::endl;

    tol_deg_ = 0.1;
    dwell_time_ = 6.0;
    sample_time_ = 6.0;
    settle_wait_ = 0.02;
    waiting_time_ = 6.0;

    t_enter_ = -1;
    sample_t_enter_ = -1;
    waiting_t_enter_ = -1;

    z_period_cnt_ = 0;
    xy_balancing_cnt_ = 0;
    z_period_threshold_ = 3;
    xy_balancing_cnt_threshold_ = 3;

    last_roll_ = 0.0;
    last_pitch_ = 0.0;
    last_yaw_ = 0.0;

    tx_mean_ = 0.0;
    ty_mean_ = 0.0;

    x_err_angle_t_enter_ = -1;
    y_err_angle_t_enter_ = -1;
    z_err_angle_t_enter_ = -1;
    x_target_angle_ = 0.0;
    y_target_angle_ = 0.0;
    z_target_angle_ = 0.0;
    x_err_angle_mean_ = 0.0;
    y_err_angle_mean_ = 0.0;
    z_err_angle_mean_ = 0.0;
    x_err_angle_t_threshold_ = 10.0;
    y_err_angle_t_threshold_ = 10.0;
    z_err_angle_t_threshold_ = 10.0;
    x_err_angle_.resize(10);
    y_err_angle_.resize(10);
    z_err_angle_.resize(10);
    xs_.resize(100);
    ys_.resize(100);

    if_in_steady_state_ = false;
    waiting_after_moving_ = false;
    if_begin_sampling_ = false;
    if_end_sampling_ = false;
    if_finish_balancing_ = false;
    flag_x_ = false;
    flag_y_ = false;
    if_set_balancing_ = false;

    if_return_zero_ = false;
    if_finish_testing_ty_ = false;
    is_balancing_z_ = false;
    pitch_metric_finish_ = false;
    if_15_ok_ = false;
    if_20_ok_ = false;
    first_balancing_sleep_cnt = 0;
    if_first_balancing = true;
    first_balancing_sleep_time = 40;
    z_change_attitude_sleep_cnt = 0;
    if_in_z_changing_attitude = false;
    z_change_attitude_sleep_time = 30;

    // 初始化状态触发标志
    steady_state_triggered_ = false;
    sampling_triggered_ = false;
    xy_balance_complete_triggered_ = false;
    z_angle_adjust_triggered_ = false;
    z_15_complete_triggered_ = false;
    z_20_complete_triggered_ = false;
    z_complete_triggered_ = false;
}

void MassCenterBalancer::balance_both_axes_fan() {
    if_set_balancing_ = true;

    int prev_dir_x = 0, prev_dir_y = 0; // 预设质量块的移动方向

    /* 扭矩→步长比例与阈值（按需现场标定） */
    // 回升力矩较大时的扭矩→步长比例
    int kx_max = 300, ky_max = 300; // step/Nm（示例系数，后续可在现场用相同口径微调）
    int min_step_max = 20, max_step_max = 5000;
    // 回升力矩较小时的扭矩→步长比例
    int kx_min = 200, ky_min = 100;
    int min_step_min = 2, max_step_min = 1000; // step/Nm（示例系数，后续可在现场用相同口径微调）

    // 分段：1.5-3.0, 3.0-10.0+
    double torque_done = 1.5; // 稳态输出阈值（Nm），小于则认定该轴已足够好
    double torque_xy_threshold = 7.0;

    tol_deg_ = 0.1;
    dwell_time_ = 6.0; // 稳态保持计时时间
    sample_time_ = 8.0; // 采样时间
    settle_wait_ = 0.02; // 控制循环频率

    // 1. 设定目标姿态 [0,0,0] → 等稳态并采样控制输出
    controller_.setAttitudeInBalancing({0.0, 0.0, 0.0});

    if (if_first_balancing) {
        // std::cout << "首次执行等待气浮台移动至指定角度附近, first_balancing_sleep_cnt=" << first_balancing_sleep_cnt << std::endl;
        first_balancing_sleep_cnt++;
        if (first_balancing_sleep_cnt >= first_balancing_sleep_time / settle_wait_) {
            if_first_balancing = false;
        }
    } else {
        wait_steady_and_sample_outputs();

        if (if_end_sampling_) {
            // 采样完成信息由 getStageStatus 返回
            xy_balance_complete_triggered_ = true;

            // 2. 终止判据
            if (std::abs(tx_mean_) <= torque_done && std::abs(ty_mean_) <= torque_done) {
                flag_y_ = true;
                flag_x_ = true;
                is_balancing_z_ = true;
                return;
            }

            // 3. 计算丝杠移动步长与方向（反向补偿：输出>0 → 质量块朝负向移动）
            double kx, ky;
            int min_step_x, min_step_y, max_step_x, max_step_y;
            if (std::abs(tx_mean_) >= torque_xy_threshold) {
                kx = kx_max;
                min_step_x = min_step_max;
                max_step_x = max_step_max;
            } else {
                kx = kx_min;
                min_step_x = min_step_min;
                max_step_x = max_step_min;
            }
            if (std::abs(ty_mean_) >= torque_xy_threshold) {
                ky = ky_max;
                min_step_y = min_step_max;
                max_step_y = max_step_max;
            } else {
                ky = ky_min;
                min_step_y = min_step_min;
                max_step_y = max_step_min;
            }

            std::vector<int16_t> steps;
            steps.reserve(2);

            int dir_x = (tx_mean_ > 0) ? -1 : +1;
            int mag_x = std::clamp(int(std::abs(tx_mean_) * kx), min_step_x, max_step_x);
            if (prev_dir_x != 0 && dir_x != prev_dir_x)
                mag_x = std::max(min_step_x, int(mag_x / 3));

            steps.push_back(dir_x * mag_x);
            prev_dir_x = dir_x;

            int dir_y = (ty_mean_ < 0) ? +1 : -1;
            int mag_y = std::clamp(int(std::abs(ty_mean_) * ky * 0.75),
                                   min_step_y, max_step_y);
            if (prev_dir_y != 0 && dir_y != prev_dir_y)
                mag_y = std::max(min_step_y, int(mag_y / 3));

            steps.push_back(dir_y * mag_y);
            prev_dir_y = dir_y;

            leadscrew_.moveTo(steps);
            // 质量块移动由 ControlModeManager 统一输出，这里不重复输出
            if_in_steady_state_ = false;
            waiting_after_moving_ = true;
            waiting_t_enter_ = clock();
            x_err_angle_.clear();
            y_err_angle_.clear();
            x_err_angle_t_enter_ = -1;
            y_err_angle_t_enter_ = -1;
        }
    }
}

void MassCenterBalancer::balance_z_axes_fan() {
    /* 扭矩→步长比例与阈值（按需现场标定） */
    int kz;
    int min_step;
    int max_step;
    // 回升力矩较大时的扭矩→步长比例
    int kz_l = 800; // step/N·m
    int min_step_l = 100;
    int max_step_l = 4000;
    // 回升力矩较小时的扭矩→步长比例
    int kz_s = 400; // step/N·m（保守）
    int min_step_s = 5;
    int max_step_s = 2000;

    double torque_z_threshold = 7.0;
    double torque_done = 3.0; // 允许的回升力矩上限（Nm）
    double pitch_metric = 0.0;
    double prev_dir_z = 0; // 方向
    int raw_sign = 0;

    tol_deg_ = 0.2;
    dwell_time_ = 6.0; // 稳态保持计时时间
    sample_time_ = 6.0; // 采样时间
    settle_wait_ = 0.02; // 控制循环频率

    if (if_in_z_changing_attitude) {
        controller_.setAttitudeInBalancing({0.0, z_target_angle_, 0.0});

        auto att = gyro_.getAttitude(); // 获取当前rpy
        double roll = att.x;
        double pitch = att.y;
        double yaw = att.z;
        // std::cout << "rpy: " << roll << ", " << pitch << ", " << yaw << std::endl;

        z_change_attitude_sleep_cnt++;
        if (z_change_attitude_sleep_cnt >= z_change_attitude_sleep_time / settle_wait_) {
            if_in_z_changing_attitude = false;
            z_change_attitude_sleep_cnt = 0;
            z_err_angle_t_enter_ = -1;
        }
    }
    else {
        // 1. 设定目标姿态 [0,0,0] → 等稳态并采样控制输出
        if (!if_return_zero_) {
            z_target_angle_ = 0.0;
            // std::cout << "z_target_angle: " << z_target_angle_ << std::endl;

            controller_.setAttitudeInBalancing({0.0, z_target_angle_, 0.0});
            auto att = gyro_.getAttitude(); // 获取当前rpy
            double roll = att.x;
            double pitch = att.y;
            double yaw = att.z;

            if (abs(yaw) < 0.5 && abs(pitch) < 0.5 && abs(roll) < 0.5) {
                if_return_zero_ = true; // 已经回到 0,0,0
                if_in_steady_state_ = false;
                z_angle_adjust_triggered_ = true;
                if_in_z_changing_attitude = true;
                z_change_attitude_sleep_cnt = 0;
                z_change_attitude_sleep_time = 45;
            }
        }
        if (if_return_zero_ && !if_finish_testing_ty_) // 已经回到 0,0,0
        {
            // std::cout << "------------------------------" << std::endl;
            // 2) 试探姿态: [0,15,0] [0,20,0] [0,30,0]
            if (!if_15_ok_)
                z_target_angle_ = 15.0;
            else if (!if_20_ok_)
                z_target_angle_ = 20.0;   // 如果你本来就“废弃 20°”，也可以留着
            else if (!pitch_metric_finish_)
                z_target_angle_ = 25.0;


            controller_.setAttitudeInBalancing({0.0, z_target_angle_, 0.0});

            wait_steady_and_sample_outputs();

            if (if_end_sampling_) {
                pitch_metric = ty_mean_; // 维持该倾角所需的回升力矩（取 Y 轴）
                if_finish_testing_ty_ = true;
                if_in_steady_state_ = false;
            }
        }

        if (if_finish_testing_ty_ && !pitch_metric_finish_) // 回升力矩获取完毕
        {
            // 3) 终止判据
            if (pitch_metric <= torque_done && pitch_metric >= 0) {
                if_return_zero_ = true;
                if (!if_15_ok_) {
                    if_15_ok_ = true;
                    if_finish_testing_ty_ = false;
                    z_15_complete_triggered_ = true;
                    z_target_angle_ = 25.0;

                    if_in_z_changing_attitude = true;
                    z_change_attitude_sleep_cnt = 0;
                    z_change_attitude_sleep_time = 25;

                    if_20_ok_ = true;

                    return;
                } else if (!if_20_ok_) { // 已废弃20° 调平
                    if_20_ok_ = true;
                    if_finish_testing_ty_ = false;
                    z_20_complete_triggered_ = true;
                    z_target_angle_ = 25.0;

                    if_in_z_changing_attitude = true;
                    z_change_attitude_sleep_cnt = 0;
                    z_change_attitude_sleep_time = 20;

                    return;
                } else if (!pitch_metric_finish_) {
                    if_finish_testing_ty_ = false;
                    if_finish_balancing_ = true;
                    if_set_balancing_ = false;
                    pitch_metric_finish_ = true;
                    z_complete_triggered_ = true;
                    return;
                }
            }

            // 4) 方向与步长（保守，不跨边界：方向翻转自动减半）
            if (pitch_metric > 0)
                raw_sign = +1;
            else if (pitch_metric < 0)
                raw_sign = -1;
            else
                raw_sign = 0;

            if (fabs(pitch_metric) >= torque_z_threshold) {
                kz = kz_l;
                min_step = min_step_l;
                max_step = max_step_l;
            } else {
                kz = kz_s;
                min_step = min_step_s;
                max_step = max_step_s;
            }
            int step_mag = std::clamp(int(std::abs(pitch_metric) * kz * 0.8),
                                      min_step, max_step); // 0.8 保守系数
            if (prev_dir_z != 0 && raw_sign != prev_dir_z)
                step_mag = std::max(min_step, int(step_mag / 3));

            int step_z = raw_sign * step_mag;
            std::vector<int16_t> action;
            action.push_back(step_z);
            leadscrew_.moveTo(action);
            prev_dir_z = raw_sign;

            // 标志位重置
            if_return_zero_ = true; //每次调之间不需要返回 0，0，0
            if_finish_testing_ty_ = false;
            if_in_steady_state_ = false;
            waiting_after_moving_ = true;
            waiting_t_enter_ = clock();
        }
    }

}

void MassCenterBalancer::wait_steady_and_sample_outputs() {
    // 等待进入稳态
    if (!if_in_steady_state_) {
        // std::cout << "-----------------------" << std::endl;
        if_end_sampling_ = false;
        if_begin_sampling_ = false;

        if (waiting_after_moving_) {
            if ((clock() - waiting_t_enter_) / CLOCKS_PER_SEC * 20 >= waiting_time_) {
                waiting_after_moving_ = false;
                t_enter_ = -1;
                x_err_angle_.clear();
                y_err_angle_.clear();
                x_err_angle_t_enter_ = -1;
                y_err_angle_t_enter_ = -1;
            } else
                return;
        } else {
            auto att = gyro_.getAttitude(); // 获取当前rpy
            double roll = att.x;
            double pitch = att.y;
            double yaw = att.z;

            // std::cout << "rpy: " << roll << ", " << pitch << ", " << yaw << std::endl;

            double target_angle_roll = 0.0;
            double target_angle_pitch = 0.0;
            double target_angle_yaw = 0.0;
            double angle_threshold = 0.5;

            if (is_balancing_z_) {
                target_angle_pitch = z_target_angle_;
                angle_threshold = 0.3;
            }

            bool err_ok = std::abs(roll - last_roll_) < tol_deg_ && std::abs(pitch - last_pitch_) < tol_deg_ &&
                          std::abs(yaw - last_yaw_) < tol_deg_ && abs(roll - target_angle_roll) < 0.7 &&
                          abs(pitch - target_angle_pitch) < angle_threshold && abs(yaw - target_angle_yaw) < 2.0;
            last_roll_ = roll;
            last_pitch_ = pitch;
            last_yaw_ = yaw;
            // std::cout << "err_ok:" << err_ok << std::endl;
            if (err_ok) {
                if (t_enter_ == -1) {
                    z_err_angle_.clear();
                    z_err_angle_t_enter_ = -1;
                    t_enter_ = clock();
                    steady_state_triggered_ = true;
                }
                if ((clock() - t_enter_) / CLOCKS_PER_SEC * 20 >= dwell_time_) {
                    if_in_steady_state_ = true;
                    if_begin_sampling_ = true;
                    sampling_triggered_ = true;
                }
            } else {
                t_enter_ = -1;
                // std::cout << "`````````````````稳态计时重置``````````````````" << std::endl;
                // std::cout << "is_balancing_z: " << is_balancing_z_ << std::endl;

                if (is_balancing_z_) // 调 Z 轴过程中
                {
                    // std::cout << "z_err_angle_t_enter: " << z_err_angle_t_enter_ << std::endl;
                    // std::cout << "z_target_angle_: " << z_target_angle_ << std::endl;
                    if (z_err_angle_t_enter_ == -1 && fabs(z_target_angle_ - pitch) >= 0.2) {
                        z_err_angle_t_enter_ = clock();
                        z_err_angle_.clear();
                    }
                    if (z_err_angle_t_enter_ != -1) {
                        // std::cout << "z_err_angle: "<< z_target_angle_ - pitch << std::endl;
                        z_err_angle_.push_back(z_target_angle_ - pitch);
                        if ((clock() - z_err_angle_t_enter_) / CLOCKS_PER_SEC * 20 >= z_err_angle_t_threshold_) {
                            if (z_err_angle_.size() > 0) {
                                z_err_angle_mean_ = std::accumulate(z_err_angle_.begin(), z_err_angle_.end(), 0.0)
                                                    / z_err_angle_.size();
                            } else
                                z_err_angle_mean_ = 0.0;
                            // std::cout << "z_err_angle_mean: " << z_err_angle_mean_ << std::endl;
                            if (fabs(z_err_angle_mean_) >= 0.2) // 移动质量块
                            {
                                std::vector<int16_t> action;
                                int step = int(z_err_angle_mean_ * 6000);
                                if (step > 0) // 限幅
                                    step = std::min(10000, step);
                                else if (step < 0)
                                    step = std::max(-10000, step);
                                action.push_back(step);
                                leadscrew_.moveTo(action);
                                waiting_after_moving_ = true;
                                waiting_t_enter_ = clock();
                            }
                            z_err_angle_.clear();
                            z_err_angle_t_enter_ = -1;
                        }
                    }
                } else //调 XY 轴过程中
                {
                    // X
                    if (x_err_angle_t_enter_ == -1 && abs(x_target_angle_ - roll) >= 0.7) {
                        x_err_angle_t_enter_ = clock();
                        x_err_angle_.clear();
                    }
                    if (x_err_angle_t_enter_ != -1) {
                        // std::cout << "x_err_angle: " << std::dec << x_target_angle_ - roll << std::endl;
                        x_err_angle_.push_back(x_target_angle_ - roll);
                        if ((clock() - x_err_angle_t_enter_) / CLOCKS_PER_SEC * 20 >= x_err_angle_t_threshold_) {
                            if (x_err_angle_.size() > 0)
                                x_err_angle_mean_ = std::accumulate(x_err_angle_.begin(), x_err_angle_.end(), 0.0)
                                                    / x_err_angle_.size();
                            else
                                x_err_angle_mean_ = 0.0;
                            if (abs(x_err_angle_mean_) >= 0.5) // 直接移动质量块
                            {
                                std::vector<int16_t> action;
                                action.push_back(int(x_err_angle_mean_ * (-12000)));
                                action.push_back(0);
                                leadscrew_.moveTo(action);

                                waiting_after_moving_ = true;
                                waiting_t_enter_ = clock();
                                x_err_angle_.clear();
                                y_err_angle_.clear();
                                x_err_angle_t_enter_ = -1;
                                y_err_angle_t_enter_ = -1;
                                return;
                            }
                            x_err_angle_.clear();
                            x_err_angle_t_enter_ = -1;
                        }
                    }
                    // Y
                    if (y_err_angle_t_enter_ == -1 && abs(y_target_angle_ - pitch) >= 0.4) {
                        y_err_angle_t_enter_ = clock();
                        y_err_angle_.clear();
                    }
                    if (y_err_angle_t_enter_ != -1) {
                        // std::cout << "y_err_angle: " << std::dec << y_target_angle_ - pitch << std::endl;
                        y_err_angle_.push_back(y_target_angle_ - pitch);
                        if ((clock() - y_err_angle_t_enter_) / CLOCKS_PER_SEC * 20 >= y_err_angle_t_threshold_) {
                            if (y_err_angle_.size() > 0)
                                y_err_angle_mean_ = std::accumulate(y_err_angle_.begin(), y_err_angle_.end(), 0.0)
                                                    / y_err_angle_.size();
                            else
                                y_err_angle_mean_ = 0.0;
                            if (abs(y_err_angle_mean_) >= 0.5) // 直接移动质量块
                            {
                                std::vector<int16_t> action;
                                action.push_back(0);
                                action.push_back(int(y_err_angle_mean_ * 12000));
                                leadscrew_.moveTo(action);

                                waiting_after_moving_ = true;
                                waiting_t_enter_ = clock();
                                x_err_angle_.clear();
                                y_err_angle_.clear();
                                x_err_angle_t_enter_ = -1;
                                y_err_angle_t_enter_ = -1;
                                return;
                            }
                            y_err_angle_.clear();
                            y_err_angle_t_enter_ = -1;
                        }
                    }
                }
            }
        }
    }

    if (if_begin_sampling_) {
        sample_t_enter_ = clock();
        last_roll_ = 0.0;
        last_pitch_ = 0.0;
        last_yaw_ = 0.0;
        xs_.clear();
        ys_.clear();
        if_begin_sampling_ = false;
    }
    // 稳态采样控制输出（平均值）
    if (if_in_steady_state_) {
        if ((clock() - sample_t_enter_) / CLOCKS_PER_SEC * 20 < sample_time_) {
            // 这里直接读控制器上一次输出的扭矩
            auto tau = controller_.getTorque();
            xs_.push_back(tau.x() / 10.0);
            ys_.push_back(tau.y() / 10.0);
        } else {
            tx_mean_ = std::accumulate(xs_.begin(), xs_.end(), 0.0) / xs_.size();
            ty_mean_ = std::accumulate(ys_.begin(), ys_.end(), 0.0) / ys_.size();
            xs_.clear();
            ys_.clear();
            if_end_sampling_ = true;
        }
    }
}

void MassCenterBalancer::balance_axes() {
    if (!(flag_x_ && flag_y_))
        balance_both_axes_fan();
    else if (flag_x_ && flag_y_ && !if_finish_balancing_)
        balance_z_axes_fan();
}

BalancingStageStatus MassCenterBalancer::getStageStatus() const {
    BalancingStageStatus status;
    status.has_new_info = false;

    // 确定当前调平阶段
    if (!flag_x_ || !flag_y_) {
        status.stage_name = "XY轴调平";
    } else if (!if_finish_balancing_) {
        status.stage_name = "Z轴调平";
    } else {
        status.stage_name = "调平完成";
        return status;
    }

    // 确定当前状态
    if (if_first_balancing) {
        status.state = "首次调平等待";
        return status;
    }

    if (if_in_steady_state_) {
        status.state = "采样力矩中";
    } else if (waiting_after_moving_) {
        status.state = "等待质量块移动后稳定";
    } else if (if_in_z_changing_attitude) {
        status.state = "Z轴角度调整中";
    } else {
        status.state = "等待稳态";
    }

    // 检查是否有新的状态信息需要输出
    if (steady_state_triggered_) {
        status.detail = "检测到稳态，开始计时";
        status.has_new_info = true;
        steady_state_triggered_ = false;
    } else if (sampling_triggered_) {
        status.detail = "稳态确认，开始采样力矩";
        status.has_new_info = true;
        sampling_triggered_ = false;
    } else if (xy_balance_complete_triggered_) {
        char buf[100];
        snprintf(buf, sizeof(buf), "采样完成 - 平均力矩: Tx=%.2f Ty=%.2f", tx_mean_, ty_mean_);
        status.detail = buf;
        status.has_new_info = true;
        xy_balance_complete_triggered_ = false;
    } else if (z_angle_adjust_triggered_) {
        status.detail = "回到0,0,0，准备Z轴调平";
        status.has_new_info = true;
        z_angle_adjust_triggered_ = false;
    } else if (z_15_complete_triggered_) {
        status.detail = "15度调平完成";
        status.has_new_info = true;
        z_15_complete_triggered_ = false;
    } else if (z_20_complete_triggered_) {
        status.detail = "20度调平完成";
        status.has_new_info = true;
        z_20_complete_triggered_ = false;
    } else if (z_complete_triggered_) {
        status.detail = "Z轴调平完成";
        status.has_new_info = true;
        z_complete_triggered_ = false;
    }

    return status;
}

void MassCenterBalancer::reset_balance() {
    /* Z轴标志位复位 */
    pitch_metric_finish_ = false; // 回升力矩是否已经消除
    if_return_zero_ = false;
    if_finish_testing_ty_ = false;
    if_in_steady_state_ = false;
    is_balancing_z_ = false;
    if_15_ok_ = false;
    if_20_ok_ = false;
}
