//
// Created by msmlab on 2025/11/14.
//

#include "control_mode_manager.h"
#include "data_collector.h"
#include "config_manager.h"
#include "Utility.h"
#include <iostream>

ControlModeManager::ControlModeManager(CallBack& mqtt_callback,
                                     AttitudePDController& attitude_controller,
                                     MassCenterBalancer& balancer,
                                     Docker& docker,
                                     Fan& fan,
                                     Wheel& wheel,
                                     LeadScrewController& leadscrew)
    : mqtt_callback_(mqtt_callback)
    , attitude_controller_(attitude_controller)
    , balancer_(balancer)
    , docker_(docker)
    , fan_(fan)
    , wheel_(wheel)
    , leadscrew_(leadscrew) {
}

ControlMode ControlModeManager::update(const SensorData& sensor_data) {
    // 处理MQTT命令
    processMqttCommands();

    // 更新电源状态
    power_off_ = mqtt_callback_.getIfPowerOff();
    wheel_.setIfPowerOff(power_off_);
    leadscrew_.setIfPowerOff(power_off_);

    // Fan电源状态：关机或调平完成后都应停止
    bool fan_should_off = power_off_ || fan_power_off_after_balance_;
    fan_.setIfPowerOff(fan_should_off);

    // 调试输出（仅当状态改变时）
    static bool last_fan_off = false;
    static bool initialized = false;
    if (!initialized) {
        last_fan_off = fan_should_off;  // 初始化时同步状态
        initialized = true;
        std::cout << "[ControlModeManager] Fan电源状态初始化: " << (fan_should_off ? "关闭" : "开启")
                  << " (power_off_=" << power_off_ << ", fan_power_off_after_balance_=" << fan_power_off_after_balance_ << ")" << std::endl;
    } else if (fan_should_off != last_fan_off) {
        std::cout << "[ControlModeManager] Fan电源状态变更: " << (fan_should_off ? "关闭" : "开启")
                  << " (power_off_=" << power_off_ << ", fan_power_off_after_balance_=" << fan_power_off_after_balance_ << ")" << std::endl;
        last_fan_off = fan_should_off;
    }
    // 如果关机，切换到空闲模式并跳过控制执行
    if (power_off_) {
        if (current_mode_ != ControlMode::IDLE) {
            switchToMode(ControlMode::IDLE, nullptr);
        }
        return current_mode_;
    }
    
    // 根据当前模式执行控制
    switch (current_mode_) {
        case ControlMode::IDLE:
            executeIdleMode();
            break;
            
        case ControlMode::DIRECT_TORQUE:
            executeDirectTorqueMode();
            break;
            
        case ControlMode::VELOCITY_CONTROL:
            executeVelocityControlMode();
            break;
            
        case ControlMode::BALANCING:
            executeBalancingMode();
            break;
            
        case ControlMode::ATTITUDE_CONTROL:
            executeAttitudeControlMode(sensor_data);
            break;
            
        case ControlMode::DOCKING:
            executeDockingMode();
            break;
            
        case ControlMode::EMERGENCY_STOP:
            executeEmergencyStopMode();
            break;
    }
    
    return current_mode_;
}

void ControlModeManager::processMqttCommands() {
    // 检查是否需要切换模式
    
    // 1) 直接力矩控制
    if (mqtt_callback_.getIfReceiveFanTorque()) {
        ControlCommand cmd;
        cmd.mode = ControlMode::DIRECT_TORQUE;
        auto torque_data = mqtt_callback_.getFanTorqueData();
        cmd.torque.tx = torque_data.tx;
        cmd.torque.ty = torque_data.ty;
        cmd.torque.tz = torque_data.tz;
        
        switchToMode(ControlMode::DIRECT_TORQUE, &cmd);
        return;
    }
    
    // 2) 速度控制
    if (mqtt_callback_.getIfReceiveFanVelocity()) {
        ControlCommand cmd;
        cmd.mode = ControlMode::VELOCITY_CONTROL;
        auto vel_data = mqtt_callback_.getFanVelData();
        cmd.velocity.wx = vel_data.wx;
        cmd.velocity.wy = vel_data.wy;
        cmd.velocity.wz = vel_data.wz;
        
        switchToMode(ControlMode::VELOCITY_CONTROL, &cmd);
        return;
    }
    
    // 3) 自动调平
    if (mqtt_callback_.getIfNeedBalancing()) {
        // 重置调平完成标志，允许重新调平
        mqtt_callback_.setFlagBalance(false);
        ControlCommand cmd;
        cmd.mode = ControlMode::BALANCING;
        cmd.need_balancing = true;

        switchToMode(ControlMode::BALANCING, &cmd);
        return;
    }
    
    // 4) 姿态控制
    if (mqtt_callback_.getIfReceiveAttitudeControl()) {
        ControlCommand cmd;
        cmd.mode = ControlMode::ATTITUDE_CONTROL;
        auto attitude_data = mqtt_callback_.getAttitudeData();
        cmd.attitude.roll = attitude_data.roll;
        cmd.attitude.pitch = attitude_data.pitch;
        cmd.attitude.yaw = attitude_data.yaw;
        
        switchToMode(ControlMode::ATTITUDE_CONTROL, &cmd);
        return;
    }
    
    // 5) 对接控制
    if (mqtt_callback_.getIfReceiveCoopDock()) {
        ControlCommand cmd;
        cmd.mode = ControlMode::DOCKING;
        auto dock_data = mqtt_callback_.getCoopDockData();
        cmd.docking.dock_device_id = dock_data.dock_device_id;
        cmd.docking.self_device_id = dock_data.self_device_id;
        
        switchToMode(ControlMode::DOCKING, &cmd);
        return;
    }
    
    // 6) 如果没有特定命令，保持当前模式
}

bool ControlModeManager::switchToMode(ControlMode mode, const ControlCommand* command) {
    if (mode == current_mode_) {
        // 已经是该模式，只更新命令
        if (command) {
            current_command_ = *command;
        }
        return true;
    }
    
    // 清理当前模式状态
    cleanupControlState();
    
    // 记录模式切换
    std::cout << "[ControlModeManager] Switching from " 
              << static_cast<int>(current_mode_) 
              << " to " << static_cast<int>(mode) << std::endl;
    
    // 更新模式和命令
    current_mode_ = mode;
    if (command) {
        current_command_ = *command;
        last_command_ = *command;
    } else {
        // 使用默认命令
        current_command_.mode = mode;
    }
    
    // 模式特定的初始化
    switch (mode) {
        case ControlMode::BALANCING:
            // 调平模式需要Fan，确保Fan已启用
            fan_power_off_after_balance_ = false;
            fan_.setIfPowerOff(false);
            // 退出可能的自动调平/闭环姿态控制状态
            current_balance_status_ = false;
            balancer_.reset_balance();
            attitude_controller_.setIfFinishBalancing(false);
            balancing_in_progress_ = true;
            break;

        case ControlMode::DIRECT_TORQUE:
        case ControlMode::VELOCITY_CONTROL:
        case ControlMode::ATTITUDE_CONTROL:
        case ControlMode::DOCKING:
            // 这些模式需要Fan，确保Fan已启用
            fan_power_off_after_balance_ = false;
            fan_.setIfPowerOff(false);
            // 退出可能的自动调平状态
            current_balance_status_ = false;
            balancer_.reset_balance();
            attitude_controller_.setIfFinishBalancing(false);
            break;

        case ControlMode::EMERGENCY_STOP:
            emergencyStop();
            break;

        default:
            break;
    }
    
    return true;
}

void ControlModeManager::cleanupControlState() {
    // 清理所有控制状态
    balancing_in_progress_ = false;
    current_balance_status_ = false;
    if_finish_balancing_ = false;
    mocap_offset_initialized_ = false;
}

void ControlModeManager::executeIdleMode() {
    // 空闲模式：不执行任何控制
    // 确保所有执行器都被设置为安全状态
    fan_.sendTorque(0, 0, 0);
}

void ControlModeManager::executeDirectTorqueMode() {
    // 发送直接力矩指令
    fan_.sendTorque(
        current_command_.torque.tx,
        current_command_.torque.ty,
        current_command_.torque.tz
    );
}

void ControlModeManager::executeVelocityControlMode() {
    // 速度闭环控制
    Eigen::Vector3d wTarget(
        current_command_.velocity.wx,
        current_command_.velocity.wy,
        current_command_.velocity.wz
    );
    attitude_controller_.setAngularVelocityInControl(wTarget);
}

void ControlModeManager::executeBalancingMode() {
    // 检查是否关机，如果是则不执行调平
    if (power_off_) {
        return;
    }

    if (!current_balance_status_) {
        std::cout << "=================== 自动调平开始 ===================\n";
        const auto& config = ConfigManager::getInstance().getConfig();
        std::vector<int16_t> action{static_cast<int16_t>(config.lead_screw_z_init_pos)};
        leadscrew_.moveTo(action); // 先降 Z 轴质量块
        std::cout << std::dec << "[调平阶段1] Z轴质量块移动步长: " << config.lead_screw_z_init_pos << std::endl;
        current_balance_status_ = true;
        balancing_in_progress_ = true;
    }

    // 执行调平
    balancer_.balance_axes();
    if_finish_balancing_ = balancer_.getIfFinishBalancing();
    attitude_controller_.setIfFinishBalancing(if_finish_balancing_);

    // 调平状态信息
    auto stage_status = balancer_.getStageStatus();

    // 获取当前力矩和陀螺姿态
    auto current_torque = attitude_controller_.getTorque();
    auto current_attitude = attitude_controller_.getCurrentGyroAttitude();

    const auto& mass_steps = leadscrew_.getCurrentPositions();

    // 统一输出调平状态（每3秒输出一次）
    static int balance_log_counter = 0;
    if (++balance_log_counter >= 150) { // 50Hz * 3秒 = 150次
        // 输出实时状态
        std::cout << std::dec;
        std::cout << "[" << stage_status.stage_name << "] "
                  << stage_status.state << " | "
                  << "姿态(roll/pitch/yaw): (" << current_attitude.x() << ", "
                  << current_attitude.y() << ", " << current_attitude.z() << ") | "
                  << "力矩(Tx/Ty/Tz): (" << current_torque.x() << ", "
                  << current_torque.y() << ", " << current_torque.z() << ") | "
                  << "质量块步长(X/Y/Z): (" << mass_steps[0] << ", "
                  << mass_steps[1] << ", " << mass_steps[2] << ")" << std::endl;
        balance_log_counter = 0;
    }

    // 立即输出新的状态信息（如果有的话）
    if (stage_status.has_new_info) {
        std::cout << "[" << stage_status.stage_name << "] " << stage_status.detail << std::endl;
    }

    if (if_finish_balancing_) {
        std::cout << std::dec;
        std::cout << "[调平完成] 自动调平算法执行完成\n";
        std::cout << "[调平完成] 最终姿态(roll/pitch/yaw): (" << current_attitude.x() << ", "
                  << current_attitude.y() << ", " << current_attitude.z() << ")\n";
        std::cout << "[调平完成] 最终力矩(Tx/Ty/Tz): (" << current_torque.x() << ", "
                  << current_torque.y() << ", " << current_torque.z() << ")\n";
        std::cout << "[调平完成] 最终质量块步长(X/Y/Z): (" << mass_steps[0] << ", "
                  << mass_steps[1] << ", " << mass_steps[2] << ")\n";
        mqtt_callback_.setFlagBalance(true);
        current_balance_status_ = false;
        balancing_in_progress_ = false;
        balancer_.reset_balance();

        // 调平完成后停止发送力矩指令（掐断与STM32通信）
        // 方式：设置fan_power_off_after_balance_标志，update函数会据此停止通信
        fan_power_off_after_balance_ = true;
        fan_.setIfPowerOff(true);
        std::cout << "[调平完成] 已停止发送力矩指令（掐断STM32通信）\n";

        // 调平完成后切换到空闲模式（但只在未关机时）
        if (!power_off_) {
            switchToMode(ControlMode::IDLE);
        }
    }
}

void ControlModeManager::executeAttitudeControlMode(const SensorData& sensor_data) {
    // 使用主循环中的姿态控制逻辑
    
    if (sensor_data.mocap.valid) {
        // 动捕有效：使用“动捕绝对目标 + 动捕->陀螺偏置(roll/pitch固定 + yaw慢更新)”生成陀螺闭环目标。
        // 目标不再依赖当前陀螺姿态 qG_cur，可避免目标随漂移跑动导致的低频自激摆动。

        // 1) 动捕当前姿态（平台等价：pitch 反号）
        Eigen::Quaterniond qM_raw = sensor_data.mocap.quaternion.normalized();
        Eigen::Vector3d rpyM_plat = gyro_util::eulerZYX_degFromQuat(qM_raw); // [roll,pitch,yaw]
        rpyM_plat.y() = -rpyM_plat.y();
        rpyM_plat.z() = gyro_util::wrapDeg180(rpyM_plat.z());

        // 保存原始动捕当前姿态用于显示（动捕系）
        Eigen::Vector3d current_mocap_raw = gyro_util::eulerZYX_degFromQuat(qM_raw);

        // 2) 当前陀螺姿态（使用 raw_attitude，避免被动捕覆盖导致逻辑不一致）
        Eigen::Vector3d rpyG_raw = sensor_data.gyro.raw_attitude; // [roll,pitch,yaw]
        rpyG_raw.z() = gyro_util::wrapDeg180(rpyG_raw.z());

        // 3) 初始化偏置（首次进入/或切换模式后）
        if (!mocap_offset_initialized_) {
            mocap_roll_offset_deg_  = gyro_util::wrapDeg180(rpyG_raw.x() - rpyM_plat.x());
            mocap_pitch_offset_deg_ = gyro_util::wrapDeg180(rpyG_raw.y() - rpyM_plat.y());
            mocap_yaw_offset_deg_   = gyro_util::wrapDeg180(rpyG_raw.z() - rpyM_plat.z());
            mocap_offset_initialized_ = true;
        }

        // 4) 偏置慢更新：用动捕提供绝对姿态约束，抵消陀螺积分漂移与安装误差残差。
        // 说明：
        // - roll/pitch 漂移通常较小，但在“动捕看仍有残差”时，可用慢更新把残差吃掉（避免卡在 0.5~1deg 附近）。
        // - yaw 漂移更明显，因此单独设置更慢/更稳的时间常数。
        //
        // 50Hz 下经验值：
        // - rp_alpha = 0.003  => tau ~ 0.02/0.003 ≈ 6.7s（对 0.5~1deg 残差收敛较快，仍足够平滑）
        // - yaw_alpha = 0.002 => tau ~ 10s（抑制 yaw 漂移，同时避免引入动捕帧间抖动）
        const double rp_alpha = 0.003;
        const double yaw_alpha = 0.002;

        const double roll_err  = gyro_util::wrapDeg180((rpyG_raw.x() - rpyM_plat.x()) - mocap_roll_offset_deg_);
        const double pitch_err = gyro_util::wrapDeg180((rpyG_raw.y() - rpyM_plat.y()) - mocap_pitch_offset_deg_);
        const double yaw_err   = gyro_util::wrapDeg180((rpyG_raw.z() - rpyM_plat.z()) - mocap_yaw_offset_deg_);

        mocap_roll_offset_deg_  = gyro_util::wrapDeg180(mocap_roll_offset_deg_  + rp_alpha  * roll_err);
        mocap_pitch_offset_deg_ = gyro_util::wrapDeg180(mocap_pitch_offset_deg_ + rp_alpha  * pitch_err);
        mocap_yaw_offset_deg_   = gyro_util::wrapDeg180(mocap_yaw_offset_deg_   + yaw_alpha * yaw_err);

        // 5) 动捕绝对目标（平台等价：pitch 反号） -> 陀螺目标（加偏置）
        Attitude target_plat{
            current_command_.attitude.roll,
            -current_command_.attitude.pitch,
            current_command_.attitude.yaw
        };

        target_attitude_.roll  = gyro_util::wrapDeg180(target_plat.roll  + mocap_roll_offset_deg_);
        target_attitude_.pitch = gyro_util::wrapDeg180(target_plat.pitch + mocap_pitch_offset_deg_);
        target_attitude_.yaw   = gyro_util::wrapDeg180(target_plat.yaw   + mocap_yaw_offset_deg_);

        const auto& config = ConfigManager::getInstance().getConfig();
        const auto angle_diff = [](double tar, double cur) {
            return gyro_util::wrapDeg180(tar - cur);
        };

        if (std::abs(angle_diff(target_attitude_.roll, rpyG_raw.x())) < config.angle_tolerance_deg &&
            std::abs(angle_diff(target_attitude_.pitch, rpyG_raw.y())) < config.angle_tolerance_deg &&
            std::abs(angle_diff(target_attitude_.yaw, rpyG_raw.z())) < config.yaw_tolerance_deg) {
            attitude_controller_.setIfFinishBalancing(true);
        }

        // 降低日志输出频率（每3秒输出一次）
        static int log_counter = 0;
        if (++log_counter >= 150) { // 50Hz * 3秒 = 150次
            // 显示用户设定的目标姿态（动捕系）
            std::cout << "[姿态控制] 目标(动捕系): roll=" << current_command_.attitude.roll
                      << ", pitch=" << current_command_.attitude.pitch
                      << ", yaw=" << current_command_.attitude.yaw << std::endl;
            // 显示当前动捕姿态
            std::cout << "[姿态控制] 当前(动捕系): roll=" << current_mocap_raw.x()
                      << ", pitch=" << current_mocap_raw.y()
                      << ", yaw=" << current_mocap_raw.z() << std::endl;
            std::cout << "[姿态控制] 当前(陀螺系): roll=" << gyro_util::wrapDeg180(rpyG_raw.x())
                      << ", pitch=" << gyro_util::wrapDeg180(rpyG_raw.y())
                      << ", yaw=" << gyro_util::wrapDeg180(rpyG_raw.z()) << std::endl;
            std::cout << "[姿态控制] 目标(陀螺系): roll=" << target_attitude_.roll
                      << ", pitch=" << target_attitude_.pitch
                      << ", yaw=" << target_attitude_.yaw
                      << " | offsets(r,p,y)=(" << mocap_roll_offset_deg_
                      << "," << mocap_pitch_offset_deg_
                      << "," << mocap_yaw_offset_deg_ << ")" << std::endl;
            log_counter = 0;
        }
    } else {
        // 没有动捕数据，使用直接目标
        target_attitude_.roll = current_command_.attitude.roll;
        target_attitude_.pitch = current_command_.attitude.pitch;
        target_attitude_.yaw = current_command_.attitude.yaw;
        attitude_controller_.setIfFinishBalancing(true);
    }
    
    // 执行姿态控制任务
    Eigen::Vector3d euler_target(
        target_attitude_.roll,
        target_attitude_.pitch,
        target_attitude_.yaw
    );
    attitude_controller_.setAttitudeInBalancing(euler_target);
}

void ControlModeManager::executeDockingMode() {
    CooperationDockData dock_data;
    dock_data.dock_device_id = current_command_.docking.dock_device_id;
    dock_data.self_device_id = current_command_.docking.self_device_id;
    dock_data.cmd_type = 0x17; // 合作目标交会对接
    
    bool status = docker_.docking(dock_data);
    if (!status) {
        std::cout << "[Cooperation Docker] Docking error, failed to get pose!" << std::endl;
    }
}

void ControlModeManager::executeEmergencyStopMode() {
    // 紧急停止：停止所有执行器
    fan_.sendTorque(0, 0, 0);
    // 可以添加其他停止逻辑
}

void ControlModeManager::emergencyStop() {
    switchToMode(ControlMode::EMERGENCY_STOP);
}

ControlMode ControlModeManager::getCurrentMode() const {
    return current_mode_;
}

const ControlCommand& ControlModeManager::getCurrentCommand() const {
    return current_command_;
}

bool ControlModeManager::isBalancing() const {
    return current_mode_ == ControlMode::BALANCING || balancing_in_progress_;
}

bool ControlModeManager::isControllingAttitude() const {
    return current_mode_ == ControlMode::ATTITUDE_CONTROL;
}

Eigen::Vector3d ControlModeManager::convertMocapToPlatformAttitude(const Eigen::Vector3d& mocap_attitude) const {
    // 动捕到平台姿态转换：pitch 反号
    return Eigen::Vector3d(
        mocap_attitude.x(),  // roll 不变
        -mocap_attitude.y(), // pitch 反号
        mocap_attitude.z()   // yaw 不变
    );
}
