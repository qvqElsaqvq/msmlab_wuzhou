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
        if (msg->get_topic() == "attitude/data")
        {
            std::cout << "On topic: " << msg->get_topic() << std::endl;
            const std::string& pl = msg->get_payload_str();
            if (pl.size() != sizeof(Plane)) {
                std::cerr << "[WARN] payload size " << pl.size()
                          << " != " << sizeof(Plane) << " bytes, drop\n";
                return;
            }

            Plane d{};
            std::memcpy(&d, pl.data(), sizeof(d));
            // 小端转主机序
            auto fix16 = [](int16_t raw) { return static_cast<int16_t>(le16toh(static_cast<uint16_t>(raw))); };
            d.head.head = fix16(d.head.head);
            d.data.wx = fix16(d.data.wx);  d.data.wy = fix16(d.data.wy);  d.data.wz = fix16(d.data.wz);
            d.data.yaw = fix16(d.data.yaw);  d.data.pitch = fix16(d.data.pitch);  d.data.roll = fix16(d.data.roll);
            d.data.wheel_current = fix16(d.data.wheel_current);  d.data.wheel_rpm = fix16(d.data.wheel_rpm);
            d.data.payload_mass  = fix16(d.data.payload_mass);
            d.data.pwr_v1 = fix16(d.data.pwr_v1);  d.data.pwr_v2 = fix16(d.data.pwr_v2);
            d.data.pwr_v3 = fix16(d.data.pwr_v3);  d.data.pwr_v4 = fix16(d.data.pwr_v4);
            d.data.pwr_i1 = fix16(d.data.pwr_i1);  d.data.pwr_i2 = fix16(d.data.pwr_i2);
            d.data.pwr_i3 = fix16(d.data.pwr_i3);  d.data.pwr_i4 = fix16(d.data.pwr_i4);
            d.data.thrust_x = fix16(d.data.thrust_x);  d.data.thrust_y = fix16(d.data.thrust_y);
            d.data.thrust_z = fix16(d.data.thrust_z);
            d.data.torque_yaw   = fix16(d.data.torque_yaw);  d.data.torque_pitch = fix16(d.data.torque_pitch);
            d.data.torque_roll  = fix16(d.data.torque_roll);
            d.tail.checksum = fix16(d.tail.checksum);

            std::cout << "----- PlaneData arrived -----\n"
                      << "device_id: " << +d.data.device_id
                      << " platform_type: 0x" << std::hex << +d.data.platform_type << std::dec
                      << " wx: " << d.data.wx / 100.0 << " wy: " << d.data.wy / 100.0
                      << " wz: " << d.data.wz / 100.0 << " yaw: " << d.data.yaw / 100.0
                      << " pitch: " << d.data.pitch / 100.0 << " roll: " << d.data.roll / 100.0
                      << " wheel_rpm: " << d.data.wheel_rpm << " battery: " << +d.data.battery_percent << "%\n";
        }
        else if(msg->get_topic() == "attitude/basic")
        {
            std::cout << "On topic: " << msg->get_topic() << std::endl;
            const std::string& pl = msg->get_payload_str();
            if (pl.size() != sizeof(CmdBasic)) {
                std::cerr << "[WARN] payload size " << pl.size()
                          << " != " << sizeof(CmdBasic) << " bytes, drop\n";
                return;
            }

            CmdBasic d{};
            std::memcpy(&d, pl.data(), sizeof(d));
            // 小端转主机序
            auto fix16 = [](int16_t raw) { return static_cast<int16_t>(le16toh(static_cast<uint16_t>(raw))); };
            d.head.head = fix16(d.head.head);
            d.data.device_id = fix16(d.data.device_id);
            d.data.cmd_type = fix16(d.data.cmd_type);
            d.data.pos_x = fix16(d.data.pos_x);
            d.data.pos_y = fix16(d.data.pos_y);
            d.data.rot_z = fix16(d.data.rot_z);
            d.data.yaw = fix16(d.data.yaw);
            d.data.pitch = fix16(d.data.pitch);
            d.data.roll = fix16(d.data.roll);
            d.tail.checksum = fix16(d.tail.checksum);

            std::cout << "----- CmdPlaneBasic arrived -----\n"
                      << "device_id: " << +d.data.device_id
                      << " cmd_type: 0x" << std::hex << +d.data.cmd_type << std::dec
                      << " pos_x: " << d.data.pos_x / 100.0 << " pos_y: " << d.data.pos_y / 100.0
                      << " rot_z: " << d.data.rot_z / 100.0 << " yaw: " << d.data.yaw / 100.0
                      << " pitch: " << d.data.pitch / 100.0 << " roll: " << d.data.roll / 100.0 << "%\n";
        }
        else if(msg->get_topic() == "attitude/trajectory")
        {
            std::cout << "On topic: " << msg->get_topic() << std::endl;
            const std::string& pl = msg->get_payload();
            std::stringstream ss(pl);
            uint8_t buffer[200] = {};
            int idx = 0;
            // std::cout << "buffer[idx]: " << std::endl;
            while (!ss.eof()) {
                int temp;
                ss >> std::hex >> temp;
                buffer[idx] = temp;
                // std::cout << std::hex << std::setfill('0') << std::setw(2) << temp << " ";
                idx += 1;
            }
            std::cout << " pl: " << pl << std::endl;
            if (idx != sizeof(CmdTrajectory)) {
                std::cerr << "[WARN] payload size " << idx
                          << " != " << sizeof(CmdTrajectory) << " bytes, drop\n";
                return;
            }

            CmdTrajectory* d = new CmdTrajectory;

            // d = (CmdTrajectory*)&buffer[0];
            std::memcpy(d, buffer, idx);

            std::cout << "----- CmdPlaneTrajectory arrived -----\n"
                      << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)d->data.device_id
                      << " cmd_type: " << std::hex << std::setfill('0') << std::setw(2)  << (int)d->data.cmd_type
                      << " traj_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)d->data.traj_id
                      << " start: " << std::hex << std::setfill('0') << std::setw(2) << (int)d->data.start << "\n";
            delete d;
        }
        else if(msg->get_topic() == "attitude/power")
        {
            std::cout << "On topic: " << msg->get_topic() << std::endl;
            const std::string& pl = msg->get_payload_str();
            if (pl.size() != sizeof(CmdPower)) {
                std::cerr << "[WARN] payload size " << pl.size()
                          << " != " << sizeof(CmdPower) << " bytes, drop\n";
                return;
            }

            CmdPower d{};
            std::memcpy(&d, pl.data(), sizeof(d));
            // 小端转主机序
            auto fix16 = [](int16_t raw) { return static_cast<int16_t>(le16toh(static_cast<uint16_t>(raw))); };
            d.head.head = fix16(d.head.head);
            d.data.device_id = fix16(d.data.device_id);
            d.data.cmd_type = fix16(d.data.cmd_type);
            d.data.cmd_data = fix16(d.data.cmd_data);
            d.tail.checksum = fix16(d.tail.checksum);

            std::cout << "----- CmdPlanePower arrived -----\n"
                      << "device_id: " << +d.data.device_id
                      << " cmd_type: 0x" << std::hex << +d.data.cmd_type << std::dec
                      << " cmd_data: 0x" << std::hex << +d.data.cmd_data << std::dec << "%\n";
        }
        else if(msg->get_topic() == "attitude/fan")
        {
            std::cout << "On topic: " << msg->get_topic() << std::endl;
            const std::string& pl = msg->get_payload_str();
            if (pl.size() != sizeof(FanTest)) {
                std::cerr << "[WARN] payload size " << pl.size()
                          << " != " << sizeof(FanTest) << " bytes, drop\n";
                return;
            }

            FanTest d{};
            std::memcpy(&d, pl.data(), sizeof(d));
            // 小端转主机序
            auto fix16 = [](int16_t raw) { return static_cast<int16_t>(le16toh(static_cast<uint16_t>(raw))); };
            d.head.head = fix16(d.head.head);
            d.data.device_id = fix16(d.data.device_id);
            d.data.cmd_type = fix16(d.data.cmd_type);
            d.data.fan_dir = fix16(d.data.fan_dir);
            d.data.torque_x = fix16(d.data.torque_x);
            d.data.torque_y = fix16(d.data.torque_y);
            d.data.torque_z = fix16(d.data.torque_z);
            d.tail.checksum = fix16(d.tail.checksum);

            std::cout << "----- CmdPlaneTrajectory arrived -----\n"
                    << "device_id: " << +d.data.device_id
                    << " cmd_type: 0x" << std::hex << +d.data.cmd_type << std::dec
                    << " fan_dir: 0x" << std::hex << +d.data.fan_dir << std::dec
                    << " torque_x: " << d.data.torque_x << std::dec
                    << " torque_y: " << d.data.torque_y << std::dec
                    << " torque_z: " << d.data.torque_z << "%\n";
        }
        // else if(msg->get_topic() == "attitude/fan")
        // {
        //     std::cout << "On topic: " << msg->get_topic() << std::endl;
        //     const std::string& pl = msg->get_payload_str();
        //     if (pl.size() != sizeof(FanTest)) {
        //         std::cerr << "[WARN] payload size " << pl.size()
        //                   << " != " << sizeof(FanTest) << " bytes, drop\n";
        //         return;
        //     }
        //
        //     FanTest d{};
        //     std::memcpy(&d, pl.data(), sizeof(d));
        //     // 小端转主机序
        //     auto fix16 = [](int16_t raw) { return static_cast<int16_t>(le16toh(static_cast<uint16_t>(raw))); };
        //     d.head.head = fix16(d.head.head);
        //     d.data.device_id = fix16(d.data.device_id);
        //     d.data.cmd_type = fix16(d.data.cmd_type);
        //     d.data.fan_dir = fix16(d.data.fan_dir);
        //     d.data.torque_x = fix16(d.data.torque_x);
        //     d.data.torque_y = fix16(d.data.torque_y);
        //     d.data.torque_z = fix16(d.data.torque_z);
        //     d.tail.checksum = fix16(d.tail.checksum);
        //
        //     std::cout << "----- CmdPlaneTrajectory arrived -----\n"
        //             << "device_id: " << +d.data.device_id
        //             << " cmd_type: 0x" << std::hex << +d.data.cmd_type << std::dec
        //             << " fan_dir: 0x" << std::hex << +d.data.fan_dir << std::dec
        //             << " torque_x: " << d.data.torque_x << std::dec
        //             << " torque_y: " << d.data.torque_y << std::dec
        //             << " torque_z: " << d.data.torque_z << "%\n";
        // }
    }

    void delivery_complete(mqtt::delivery_token_ptr tok) override {
        std::cout << "Delivery complete!" << std::endl;
    }

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

    struct AttitudeData
    {
        double pitch = 0, roll = 0, yaw = 0;
        double q0 = 0, q1 = 0, q2 = 0, q3 = 0;
    };

private:
    Plane plane_;
    CmdBasic cmd_basic_;
    CmdTrajectory cmd_trajectory_;
    CmdPower cmd_power_;
    FanTest fan_test_;
    WheelTest wheel_test_;
    Balance balance_;
    FanCalibration fan_calibration_;

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
};

#endif //FAN_CONTROL_CODE_ADAPT_C_MQTT_SERVER_H