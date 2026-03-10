//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_CONTROL_MODE_MANAGER_H
#define FAN_CONTROL_CODE_ADAPT_C_CONTROL_MODE_MANAGER_H

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <functional>
#include "attitude_pd_controller.h"
#include "mass_center_balancer.h"
#include "fan.h"
#include "wheel.h"
#include "leadscrew_controller.h"
#include "docker.h"
#include "MQTT_server.h"

// 前向声明SensorData
struct SensorData;

/**
 * @brief 控制模式枚举
 */
enum class ControlMode {
    IDLE,               // 空闲模式
    DIRECT_TORQUE,      // 直接力矩控制
    VELOCITY_CONTROL,   // 速度闭环控制
    BALANCING,          // 自动调平
    ATTITUDE_CONTROL,   // 姿态闭环控制
    DOCKING,            // 对接控制
    EMERGENCY_STOP      // 紧急停止
};

/**
 * @brief 控制命令
 */
struct ControlCommand {
    ControlMode mode = ControlMode::IDLE;
    
    // 直接力矩控制参数
    struct {
        double tx = 0.0, ty = 0.0, tz = 0.0;
    } torque;
    
    // 速度控制参数
    struct {
        double wx = 0.0, wy = 0.0, wz = 0.0;
    } velocity;
    
    // 姿态控制参数
    struct {
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
    } attitude;
    
    // 对接控制参数
    struct {
        uint8_t dock_device_id = 0;
        uint8_t self_device_id = 0;
    } docking;
    
    // 其他参数
    bool power_off = false;
    bool need_balancing = false;
    bool need_fan_calibration = false;
};

/**
 * @brief 控制模式管理器
 * 
 * 负责管理不同的控制模式，处理控制命令的切换和执行
 */
class ControlModeManager {
public:
    /**
     * @brief 构造函数
     */
    ControlModeManager(CallBack& mqtt_callback,
                      AttitudePDController& attitude_controller,
                      MassCenterBalancer& balancer,
                      Docker& docker,
                      Fan& fan,
                      Wheel& wheel,
                      LeadScrewController& leadscrew);

    /**
     * @brief 更新控制模式
     * @param sensor_data 传感器数据
     * @return 当前控制模式
     */
    ControlMode update(const SensorData& sensor_data);

    /**
     * @brief 获取当前控制模式
     */
    ControlMode getCurrentMode() const;

    /**
     * @brief 切换到指定模式
     * @param mode 目标模式
     * @param command 控制命令（可选）
     * @return 是否切换成功
     */
    bool switchToMode(ControlMode mode, const ControlCommand* command = nullptr);

    /**
     * @brief 紧急停止
     */
    void emergencyStop();

    /**
     * @brief 获取当前控制命令
     */
    const ControlCommand& getCurrentCommand() const;

    /**
     * @brief 检查是否正在调平
     */
    bool isBalancing() const;

    /**
     * @brief 检查是否正在控制姿态
     */
    bool isControllingAttitude() const;

private:
    /**
     * @brief 处理MQTT命令
     */
    void processMqttCommands();

    /**
     * @brief 执行空闲模式
     */
    void executeIdleMode();

    /**
     * @brief 执行直接力矩控制模式
     */
    void executeDirectTorqueMode();

    /**
     * @brief 执行速度控制模式
     */
    void executeVelocityControlMode();

    /**
     * @brief 执行自动调平模式
     */
    void executeBalancingMode();

    /**
     * @brief 执行姿态控制模式
     */
    void executeAttitudeControlMode(const SensorData& sensor_data);

    /**
     * @brief 执行对接控制模式
     */
    void executeDockingMode();

    /**
     * @brief 执行紧急停止模式
     */
    void executeEmergencyStopMode();

    /**
     * @brief 清理控制状态
     */
    void cleanupControlState();

    /**
     * @brief 转换动捕姿态到平台姿态
     */
    Eigen::Vector3d convertMocapToPlatformAttitude(const Eigen::Vector3d& mocap_attitude) const;

    // 组件引用
    CallBack& mqtt_callback_;
    AttitudePDController& attitude_controller_;
    MassCenterBalancer& balancer_;
    Docker& docker_;
    Fan& fan_;
    Wheel& wheel_;
    LeadScrewController& leadscrew_;

    // 当前状态
    ControlMode current_mode_ = ControlMode::IDLE;
    ControlCommand current_command_;
    ControlCommand last_command_;

    // 状态标志
    bool balancing_in_progress_ = false;
    bool current_balance_status_ = false;
    bool if_finish_balancing_ = false;
    bool power_off_ = false;
    bool fan_power_off_after_balance_ = false;  // 调平完成后停止旋翼

    // 目标姿态（用于姿态控制）
    struct Attitude target_attitude_ {0.0, 0.0, 0.0};
};

#endif //FAN_CONTROL_CODE_ADAPT_C_CONTROL_MODE_MANAGER_H
