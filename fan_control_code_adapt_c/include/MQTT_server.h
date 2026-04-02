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

struct AttitudeData
{
    double pitch = 0.0, roll = 0.0, yaw = 0.0;
};

struct TorqueData { float tx=0.0f, ty=0.0f, tz=0.0f; };
struct VelData    { float wx=0.0f, wy=0.0f, wz=0.0f; };  // deg/s

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

    std::string SERVER_ADDRESS;
    std::string CLIENT_ID;
    int QOS;
    std::string plane_data_topic; // 平面气浮台传上位机指令格式
    std::string cmd_plane_basic_topic; // 上位机传平面气浮台基础指令
    std::string cmd_plane_trajectory_topic; // 上位机传平面气浮内置轨迹指令
    std::string cmd_plane_power_topic; // 上位机传平面气浮台开关机指令
    std::string fan_torque_topic;   // 推力器力矩模式（原 attitude/fan）
    std::string fan_velocity_topic; // 推力器速度控制模式
    std::string wheel_test_topic; // 动量轮临时测试指令
    std::string balance_topic; // 上位机传姿态气浮台调平指令
    std::string fan_calibration_topic; // 上位机传姿态气浮台旋翼校准指令
    std::string fan_calibration_topic1;
    std::string coop_dock_topic;

private:
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
    bool flag_balance_; // 自动调平指令标志，标志是否完成过至少一次自动调平
    bool if_need_balancing_;

    bool if_open_; // 开关机状态

    bool running_; // 线程是否还在运行

    bool if_need_fan_calibration_;
    bool flag_fan_calibration_; // 当前是否进行过至少一次旋翼校准

    bool if_receive_attitude_basic_;

    bool if_receive_fan_torque_;
    bool if_receive_fan_velocity_;

    bool if_power_off_;

    bool if_receive_coop_dock_;

    /* 数据变量 */
    double wx_;
    double wy_;
    double wz_;
    double roll_;
    double pitch_;
    double yaw_;
    double q0_;
    double q1_;
    double q2_;
    double q3_;
    std::vector<uint8_t> wheel_dirs_{0x55, 0x55, 0x55};
    std::vector<uint16_t> wheel_rpms_{0, 0, 0};
    AttitudeData attitude_data_;
    TorqueData fan_torque_data_;
    VelData fan_vel_data_;
    CooperationDockData coop_dock_data_;

    std::atomic<bool> task_command_received_{false};
};

#endif //FAN_CONTROL_CODE_ADAPT_C_MQTT_SERVER_H
