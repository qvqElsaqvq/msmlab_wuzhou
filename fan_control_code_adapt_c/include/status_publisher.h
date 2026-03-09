//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_STATUS_PUBLISHER_H
#define FAN_CONTROL_CODE_ADAPT_C_STATUS_PUBLISHER_H

#include <mqtt/async_client.h>
#include <memory>
#include <atomic>
#include "message.h"
#include "data_collector.h"
#include "control_mode_manager.h"
#include "attitude_pd_controller.h"
#include "mass_center_balancer.h"
#include "fan.h"
#include "wheel.h"

/**
 * @brief 状态发布器
 * 
 * 负责收集系统状态并定期通过MQTT发布
 */
class StatusPublisher {
public:
    /**
     * @brief 构造函数
     */
    StatusPublisher(mqtt::async_client& mqtt_client,
                   CallBack& mqtt_callback,
                   DataCollector& data_collector,
                   ControlModeManager& control_mode_manager,
                   AttitudePDController& attitude_controller,
                   MassCenterBalancer& balancer,
                   Fan& fan,
                   Wheel& wheel);

    /**
     * @brief 初始化状态发布器
     */
    void initialize();

    /**
     * @brief 更新系统状态（在每个控制循环中调用）
     */
    void update();

    /**
     * @brief 发布状态到MQTT
     * @param force 强制发布（忽略发送间隔）
     */
    void publishStatus(bool force = false);

    /**
     * @brief 获取发送间隔计数器
     */
    int getSendCounter() const;

    /**
     * @brief 重置发送计数器
     */
    void resetSendCounter();

    /**
     * @brief 设置发送间隔
     */
    void setSendInterval(int interval);

private:
    /**
     * @brief 收集当前系统状态
     */
    void collectSystemStatus();

    /**
     * @brief 构建状态消息
     */
    void buildStatusMessage(Plane& message);

    /**
     * @brief 计算校验和
     */
    uint8_t calculateChecksum(const uint8_t* data, size_t length) const;

    /**
     * @brief 填充状态数据结构
     */
    void fillStatusData(PlaneData& data);

    // MQTT相关
    mqtt::async_client& mqtt_client_;
    CallBack& mqtt_callback_;
    
    // 系统组件引用
    DataCollector& data_collector_;
    ControlModeManager& control_mode_manager_;
    AttitudePDController& attitude_controller_;
    MassCenterBalancer& balancer_;
    Fan& fan_;
    Wheel& wheel_;

    // 状态数据
    struct SystemStatus {
        // 姿态数据
        struct Attitude {
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
        } attitude;
        
        // 角速度数据
        struct AngularVelocity {
            double wx = 0.0;
            double wy = 0.0;
            double wz = 0.0;
        } angular_velocity;
        
        // 动量轮状态
        struct WheelStatus {
            uint8_t dir = 0x55;  // 旋转方向
            int16_t speed = 0;   // 转速
            int16_t current = 0; // 电流
            uint8_t fault = 0x00; // 故障码
        } wheel;
        
        // 力矩数据
        struct Torque {
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
        } torque;
        
        // 系统标志
        struct Flags {
            bool power_off = false;             // 电源状态
            bool gyro_fault = false;            // 陀螺仪故障
            bool wheel_fault = false;           // 动量轮故障
            bool balance_flag = false;          // 自动调平标志
            bool balance_set = false;           // 自动调平触发
            bool fan_calibration_flag = false;  // 旋翼校准标志
            bool fan_calibration_set = false;   // 旋翼校准触发
            bool traj_ready = true;             // 轨迹准备完成
        } flags;
        
        // 电源信息
        struct PowerInfo {
            int16_t battery_percent = 100;      // 电量百分比
            int16_t pwr_v1 = 0;                 // 通道1电压
            int16_t pwr_v2 = 0;                 // 通道2电压
            int16_t pwr_v3 = 0;                 // 通道3电压
            int16_t pwr_v4 = 0;                 // 通道4电压
            int16_t pwr_i1 = 0;                 // 通道1电流
            int16_t pwr_i2 = 0;                 // 通道2电流
            int16_t pwr_i3 = 0;                 // 通道3电流
            int16_t pwr_i4 = 0;                 // 通道4电流
        } power;
        
        // 其他信息
        uint8_t platform_type = 0xF4;           // 平台类型
        uint8_t platform_status = 0x01;         // 平台状态
        int16_t payload_mass = 0;               // 承载质量
        int16_t thrust_x = 0;                   // X方向推力
        int16_t thrust_y = 0;                   // Y方向推力
        int16_t thrust_z = 0;                   // Z方向推力
    } current_status_;

    // 发送控制
    std::atomic<int> send_counter_{0};
    std::atomic<int> send_interval_{5};  // 默认5个循环发送一次
    std::atomic<bool> initialized_{false};
};

#endif //FAN_CONTROL_CODE_ADAPT_C_STATUS_PUBLISHER_H
