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

#include "message.h"
#include "serial.h"

class CallBack : public virtual mqtt::callback,
                 public virtual mqtt::iaction_listener
{
public:
    explicit CallBack();

    ~CallBack() override;

    struct AttitudeData
    {
        double pitch = 0, roll = 0, yaw = 0;
        double q0 = 0, q1 = 0, q2 = 0, q3 = 0;
    };

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

    AttitudeData getAttitudeData();

    std::string SERVER_ADDRESS;
    std::string CLIENT_ID;
    int QOS;
    std::string plane_data_topic; // 平面气浮台传上位机指令格式
    std::string cmd_plane_basic_topic; // 上位机传平面气浮台基础指令
    std::string cmd_plane_trajectory_topic; // 上位机传平面气浮内置轨迹指令
    std::string cmd_plane_power_topic; // 上位机传平面气浮台开关机指令
    std::string fan_test_topic; // 推力器临时测试指令
    std::string wheel_test_topic; // 动量轮临时测试指令
    std::string balance_topic; // 上位机传姿态气浮台调平指令
    std::string fan_calibration_topic; // 上位机传姿态气浮台旋翼校准指令

    /* 控制指令标志位 */
    bool flag_balance_; // 自动调平指令标志，标志
    bool reset_balance_; // 重置自动调平
    bool flag_attitude_euler_; // 欧拉角模式标志
    bool flag_attitude_quat_; // 四元数模式标志
    bool if_open_; // 开关机状态
    bool running_; // 线程是否还在运行

private:
    Plane* plane_;
    CmdBasic* cmd_basic_;
    CmdTrajectory* cmd_trajectory_;
    CmdPower* cmd_power_;
    FanTest* fan_test_;
    WheelTest* wheel_test_;
    Balance* balance_;
    FanCalibration* fan_calibration_;

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

    std::map<std::string, double> attitude_data; // 姿态控制指令数据
};

#endif //FAN_CONTROL_CODE_ADAPT_C_MQTT_SERVER_H