//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_MQTT_SERVER_H
#define FAN_CONTROL_CODE_ADAPT_C_MQTT_SERVER_H

#include <iostream>
#include <fstream>
#include <mqtt/async_client.h>
#include <map>

#include "message.h"
#include "cpp/INIReader.h"
#include "serial.h"

INIReader ini("config.ini");

class CallBack : public virtual mqtt::callback,
                 public virtual mqtt::iaction_listener
{
public:
    explicit CallBack();

    void on_failure(const mqtt::token &tok) override {
        std::cout << "Connection failed!" << std::endl;
    }

    void on_success(const mqtt::token &tok) override {
        std::cout << "Connection success!" << std::endl;
    }

    void connection_lost(const std::string &cause) override {
        std::cout << "Connection lost: " << cause << std::endl;
    }

    void message_arrived(mqtt::const_message_ptr msg) override {
        std::cout << "On topic: " << msg->get_topic() <<" received message: " << msg->to_string() << std::endl;
    }

    void delivery_complete(mqtt::delivery_token_ptr tok) override {
        std::cout << "Delivery complete!" << std::endl;
    }

private:
    std::string BROKER_HOST; // MQTT Broker IP
    int BROKER_PORT;
    std::string CLIENT_ID;
    int QOS;
    std::string SUB_TOPIC;
    std::string PUB_TOPIC;
};

class MQTTServer
{
public:
    explicit MQTTServer(std::string BROKER_HOST, int BROKER_PORT, std::string CLIENT_ID, std::string PUB_TOPIC,
                        std::string SUB_TOPIC);

    ~MQTTServer();

    struct AttitudeData
    {
        double pitch = 0, roll = 0, yaw = 0;
        double q0 = 0, q1 = 0, q2 = 0, q3 = 0;
    };

    void start();

    void stop();

    std::vector<uint8_t> send_data();

    void sender_thread();

    void message_arrived(mqtt::const_message_ptr msg);

private:
    mqtt::async_client client_;
    CallBack cb_;
    mqtt::connect_options connOpts_;

    std::string broker_host_;
    int broker_port_;
    std::string client_id_;
    std::string pub_topic_;
    std::string sub_topic_;

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

    /* 控制指令标志位 */
    bool flag_balance_; // 自动调平指令标志
    bool flag_attitude_euler_; // 欧拉角模式标志
    bool flag_attitude_quat_; // 四元数模式标志
    bool running_; // 线程是否还在运行

    std::map<std::string, double> attitude_data; // 姿态控制指令数据

    std::thread sender_th_; // 线程控制
};

#endif //FAN_CONTROL_CODE_ADAPT_C_MQTT_SERVER_H