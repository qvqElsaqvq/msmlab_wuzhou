//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_MQTT_SERVER_H
#define FAN_CONTROL_CODE_ADAPT_C_MQTT_SERVER_H

#include <iostream>
#include <fstream>
#include <mqtt/async_client.h>
#include <map>
#include <cstring>
#include <endian.h>
#include <atomic>

#include "message.h"
#include "serial.h"

// 上位机下发的姿态目标（单位：deg）
struct AttitudeData {
    double pitch = 0.0; // 目标俯仰角
    double roll = 0.0;  // 目标滚转角
    double yaw = 0.0;   // 目标偏航角
};

// 上位机下发的推力器力矩指令（单位由上位机协议约定，通常与 sendTorque 一致）
struct TorqueData { float tx = 0.0f, ty = 0.0f, tz = 0.0f; };

// 上位机下发的角速度指令（deg/s）
struct VelData { float wx = 0.0f, wy = 0.0f, wz = 0.0f; };

class CallBack : public virtual mqtt::callback,
                 public virtual mqtt::iaction_listener
{
public:
    explicit CallBack();

    ~CallBack() override;

    void on_failure(const mqtt::token &tok) override {
        std::cout << "Connection failed!" << std::endl;
    }

    void on_success(const mqtt::token &tok) override {
        std::cout << "Connection success!" << std::endl;
    }

    void connection_lost(const std::string &cause) override {
        std::cout << "Connection lost: " << cause << std::endl;
    }

    void message_arrived(mqtt::const_message_ptr msg) override;

    void delivery_complete(mqtt::delivery_token_ptr tok) override {
        // std::cout << "Delivery complete!" << std::endl;
    }

    // 将 payload（16 进制字符串）转为二进制缓冲区（与 message.h 的结构体对齐）
    void convert_msg(const std::string& pl, uint8_t *buffer, int& idx);

    [[nodiscard]] AttitudeData getAttitudeData() const { return attitude_data_; }

    [[nodiscard]] bool getIfNeedBalancing() const { return if_need_balancing_; }

    [[nodiscard]] bool getIfNeedFanCalibration() const { return if_need_fan_calibration_; }

    [[nodiscard]] bool getIfReceiveAttitudeControl() const{ return if_receive_attitude_basic_; }

    [[nodiscard]] bool getIfReceiveFanTorque() const { return if_receive_fan_torque_; }
    [[nodiscard]] bool getIfReceiveFanVelocity() const { return if_receive_fan_velocity_; }

    [[nodiscard]] TorqueData getFanTorqueData() const { return fan_torque_data_; }
    [[nodiscard]] VelData getFanVelData() const { return fan_vel_data_; }

    [[nodiscard]] bool getIfPowerOff() const{ return if_power_off_; }

    [[nodiscard]] bool getIfReceiveCoopDock() const { return if_receive_coop_dock_; }

    [[nodiscard]] CooperationDockData getCoopDockData() const {return coop_dock_data_;}

    [[nodiscard]] bool hasTaskCommandReceived() const { return task_command_received_.load(std::memory_order_acquire); }

    void setFlagBalance(bool flag);

    // ===== MQTT 连接配置（由 ConfigManager 或默认值初始化）=====
    std::string SERVER_ADDRESS;
    std::string CLIENT_ID;
    int QOS;
    std::string plane_data_topic;           // 气浮台 -> 上位机：状态数据
    std::string cmd_plane_basic_topic;      // 上位机 -> 气浮台：基础姿态/位姿指令
    std::string cmd_plane_trajectory_topic; // 上位机 -> 气浮台：内置轨迹指令
    std::string cmd_plane_power_topic;      // 上位机 -> 气浮台：开关机/停机指令
    std::string fan_torque_topic;           // 上位机 -> 气浮台：推力器力矩模式
    std::string fan_velocity_topic;         // 上位机 -> 气浮台：推力器角速度模式
    std::string wheel_test_topic;           // 上位机 -> 气浮台：动量轮测试指令
    std::string balance_topic;              // 上位机 -> 气浮台：自动调平指令
    std::string fan_calibration_topic;      // 上位机 -> 气浮台：旋翼校准指令
    std::string coop_dock_topic;            // 上位机 -> 气浮台：合作目标对接任务指令

private:
    // ===== MQTT 消息结构体缓存（message.h）=====
    Plane* plane_;
    CmdBasic* cmd_basic_;
    CmdTrajectory* cmd_trajectory_;
    CmdPower* cmd_power_;
    FanTest* fan_test_;
    WheelTest* wheel_test_;
    Balance* balance_;
    FanCalibration* fan_calibration_;
    CooperationDock* coop_dock_;

    /* 控制指令标志位 */
    bool flag_balance_;             // 是否已完成过至少一次自动调平（用于 reset 逻辑）
    bool if_need_balancing_;        // 当前是否需要进入调平流程

    bool if_open_;                  // 设备开关机状态（保留字段）

    bool running_;                  // 回调对象运行状态（保留字段）

    bool if_need_fan_calibration_;  // 当前是否需要进入旋翼校准流程
    bool flag_fan_calibration_;     // 是否已完成过至少一次旋翼校准（用于 reset 逻辑）

    bool if_receive_attitude_basic_; // 是否收到姿态/基础控制指令

    bool if_receive_fan_torque_;     // 是否收到推力器力矩指令
    bool if_receive_fan_velocity_;   // 是否收到推力器角速度指令

    bool if_power_off_;              // 是否收到停机指令（上层可据此停止输出）

    bool if_receive_coop_dock_;      // 是否收到合作对接任务指令

    /* 数据变量 */
    double wx_;                      // 角速度 wx（保留字段）
    double wy_;                      // 角速度 wy（保留字段）
    double wz_;                      // 角速度 wz（保留字段）
    double roll_;                    // roll（保留字段）
    double pitch_;                   // pitch（保留字段）
    double yaw_;                     // yaw（保留字段）
    double q0_;                      // 四元数 w（保留字段）
    double q1_;                      // 四元数 x（保留字段）
    double q2_;                      // 四元数 y（保留字段）
    double q3_;                      // 四元数 z（保留字段）
    std::vector<uint8_t> wheel_dirs_{0x55, 0x55, 0x55};     // 动量轮方向缓存（保留字段）
    std::vector<uint16_t> wheel_rpms_{0, 0, 0};             // 动量轮转速缓存（保留字段）
    AttitudeData attitude_data_;      // 姿态目标缓存（来自 cmd_plane_basic_topic）
    TorqueData fan_torque_data_;      // 推力器力矩目标缓存（来自 fan_torque_topic）
    VelData fan_vel_data_;            // 推力器角速度目标缓存（来自 fan_velocity_topic）
    CooperationDockData coop_dock_data_; // 合作对接任务缓存

    std::atomic<bool> task_command_received_{false};
};

#endif //FAN_CONTROL_CODE_ADAPT_C_MQTT_SERVER_H
