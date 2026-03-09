//
// Created by msmlab on 2025/11/14.
//

#include "status_publisher.h"
#include "config_manager.h"
#include "Utility.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <cstring>

StatusPublisher::StatusPublisher(mqtt::async_client& mqtt_client,
                               CallBack& mqtt_callback,
                               DataCollector& data_collector,
                               ControlModeManager& control_mode_manager,
                               AttitudePDController& attitude_controller,
                               MassCenterBalancer& balancer,
                               Fan& fan,
                               Wheel& wheel)
    : mqtt_client_(mqtt_client)
    , mqtt_callback_(mqtt_callback)
    , data_collector_(data_collector)
    , control_mode_manager_(control_mode_manager)
    , attitude_controller_(attitude_controller)
    , balancer_(balancer)
    , fan_(fan)
    , wheel_(wheel) {
}

void StatusPublisher::initialize() {
    const auto& config = ConfigManager::getInstance().getConfig();
    send_interval_ = config.mqtt_send_interval;
    initialized_ = true;
}

void StatusPublisher::update() {
    if (!initialized_) {
        return;
    }
    
    // 收集系统状态
    collectSystemStatus();
    
    // 更新发送计数器并决定是否发送
    send_counter_++;
    if (send_counter_ >= send_interval_) {
        publishStatus(true);
    }
}

void StatusPublisher::collectSystemStatus() {
    // 获取传感器数据
    auto sensor_data = data_collector_.getSensorData();

    // 更新姿态：使用原始陀螺仪姿态（不被动捕覆盖）
    current_status_.attitude.roll = data_collector_.getRawGyroAttitude().x();
    current_status_.attitude.pitch = data_collector_.getRawGyroAttitude().y();
    current_status_.attitude.yaw = data_collector_.getRawGyroAttitude().z();

    current_status_.angular_velocity.wx = sensor_data.gyro.angular_velocity.x();
    current_status_.angular_velocity.wy = sensor_data.gyro.angular_velocity.y();
    current_status_.angular_velocity.wz = sensor_data.gyro.angular_velocity.z();

    // 获取动量轮状态
    auto wheel_status = wheel_.getStatus();
    current_status_.wheel.dir = wheel_status.dir;
    current_status_.wheel.speed = wheel_status.speed;
    current_status_.wheel.current = 0; // 暂时设为0，实际应从硬件读取

    // 获取力矩数据
    auto torque = attitude_controller_.getTorque();
    current_status_.torque.roll = torque.x();
    current_status_.torque.pitch = torque.y();
    current_status_.torque.yaw = torque.z();

    // 更新系统标志
    current_status_.flags.power_off = mqtt_callback_.getIfPowerOff();

    // 获取调平状态
    current_status_.flags.balance_flag = balancer_.getIfFinishBalancing();
    current_status_.flags.balance_set = balancer_.getIfInBalancing();

    // 设置其他标志（暂时硬编码）
    current_status_.flags.gyro_fault = false;
    current_status_.flags.wheel_fault = false;
    current_status_.flags.fan_calibration_flag = false;
    current_status_.flags.fan_calibration_set = false;
    current_status_.flags.traj_ready = true;

    // 设置平台状态
    if (current_status_.flags.power_off) {
        current_status_.platform_status = 0x00; // 停机
    } else {
        current_status_.platform_status = 0x01; // 运行中
    }
}

void StatusPublisher::publishStatus(bool force) {
    if (!force && send_counter_ < send_interval_) {
        return;
    }

    // 构建状态消息
    Plane message;
    buildStatusMessage(message);

    // 计算并设置校验和
    uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&message);
    size_t data_length = sizeof(message.head) + sizeof(message.data);
    uint8_t checksum = calculateChecksum(data_ptr, data_length);
    message.tail.checksum = checksum;

    try {
        // 使用 vector 作为缓冲区
        std::vector<uint8_t> payload(sizeof(message));
        std::memcpy(payload.data(), &message, sizeof(message));

        mqtt_client_.publish(
            mqtt_callback_.plane_data_topic,
            payload.data(),
            payload.size(),
            mqtt_callback_.QOS,
            false
        );

        // 重置计数器
        send_counter_ = 0;
    } catch (const mqtt::exception& e) {
        std::cerr << "[StatusPublisher] Failed to publish status: " << e.what() << std::endl;
    }
}

void StatusPublisher::buildStatusMessage(Plane& message) {
    // 设置帧头
    message.head.head = 0x4D47; // 'MG'
    
    // 填充数据
    fillStatusData(message.data);
}

void StatusPublisher::fillStatusData(PlaneData& data) {
    // 设备ID
    data.device_id = 0x05;
    
    // 平台类型
    data.platform_type = current_status_.platform_type;
    
    // 指令计数和文件数
    data.cmd_count = 0x01;
    data.file_count = 0x01;
    
    // 平台状态
    data.platform_status = current_status_.platform_status;
    
    // 陀螺仪故障码
    data.gyro_fault = current_status_.flags.gyro_fault ? 0x01 : 0x00;
    
    // 角速度数据（乘以100）
    data.wx = static_cast<int16_t>(current_status_.angular_velocity.wx * 100.0f);
    data.wy = static_cast<int16_t>(current_status_.angular_velocity.wy * 100.0f);
    data.wz = static_cast<int16_t>(current_status_.angular_velocity.wz * 100.0f);
    
    // 姿态角数据（乘以100）
    data.roll = static_cast<int16_t>(current_status_.attitude.roll * 100.0f);
    data.pitch = static_cast<int16_t>(current_status_.attitude.pitch * 100.0f);
    data.yaw = static_cast<int16_t>(current_status_.attitude.yaw * 100.0f);
    
    // 动量轮数据
    data.wheel_dir = current_status_.wheel.dir;
    data.wheel_current = current_status_.wheel.current;
    data.wheel_rpm = current_status_.wheel.speed;
    data.wheel_fault = current_status_.flags.wheel_fault ? 0x01 : 0x00;
    
    // 承载质量
    data.payload_mass = current_status_.payload_mass;
    
    // 电源数据
    data.pwr_v1 = current_status_.power.pwr_v1;
    data.pwr_v2 = current_status_.power.pwr_v2;
    data.pwr_v3 = current_status_.power.pwr_v3;
    data.pwr_v4 = current_status_.power.pwr_v4;
    data.pwr_i1 = current_status_.power.pwr_i1;
    data.pwr_i2 = current_status_.power.pwr_i2;
    data.pwr_i3 = current_status_.power.pwr_i3;
    data.pwr_i4 = current_status_.power.pwr_i4;
    
    // 电池电量
    data.battery_percent = current_status_.power.battery_percent;
    
    // 轨迹准备状态
    data.traj_ready = current_status_.flags.traj_ready ? 0x01 : 0x00;
    
    // 推力数据
    data.thrust_x = current_status_.thrust_x;
    data.thrust_y = current_status_.thrust_y;
    data.thrust_z = current_status_.thrust_z;
    
    // 力矩数据
    data.torque_roll = static_cast<int16_t>(current_status_.torque.roll * 100.0f);
    data.torque_pitch = static_cast<int16_t>(current_status_.torque.pitch * 100.0f);
    data.torque_yaw = static_cast<int16_t>(current_status_.torque.yaw * 100.0f);
    
    // 调平标志
    data.balance_flag = current_status_.flags.balance_flag ? 0x01 : 0x00;
    data.balance_set = current_status_.flags.balance_set ? 0x01 : 0x00;
    
    // 旋翼校准标志
    data.fan_calibration_flag = current_status_.flags.fan_calibration_flag ? 0x01 : 0x00;
    data.fan_calibration_set = current_status_.flags.fan_calibration_set ? 0x01 : 0x00;
    
    // 保留字段清零
    for (int i = 0; i < 7; i++) {
        data.reserved[i] = 0x00;
    }
}

uint8_t StatusPublisher::calculateChecksum(const uint8_t* data, size_t length) const {
    // 简单的异或校验
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

int StatusPublisher::getSendCounter() const {
    return send_counter_.load();
}

void StatusPublisher::resetSendCounter() {
    send_counter_ = 0;
}

void StatusPublisher::setSendInterval(int interval) {
    send_interval_ = interval;
}
