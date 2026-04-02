//
// Created by msmlab on 2025/11/14.
//

#include "data_collector.h"
#include "config_manager.h"
#include <iostream>
#include <chrono>

DataCollector::DataCollector(GyroScope& gyro, msmserial::MsMSerial& serial)
    : gyro_(gyro)
    , serial_(serial)
    , last_update_time_(std::chrono::steady_clock::now()) {
}

bool DataCollector::initialize() {
    const auto& config = ConfigManager::getInstance().getConfig();

    // 初始化IMU
    ForsenseIMU::InitSerial(config.imu_serial_port.c_str());
    std::cout << "[DataCollector] IMU initialized" << std::endl;

    // Nokov 已在 SystemController::start() 中启动，这里不需要重复启动

    return true;
}

bool DataCollector::update() {
    auto current_time = std::chrono::steady_clock::now();
    
    // 更新所有传感器数据
    updateGyroData();
    updateImuData();
    updateMocapData();
    
    // 数据融合
    fuseData();
    
    // 更新统计信息
    update_count_++;
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_update_time_).count();
    
    if (elapsed > 1000) { // 每秒计算一次频率
        update_frequency_ = update_count_ * 1000.0 / elapsed;
        update_count_ = 0;
        last_update_time_ = current_time;
    }
    
    sensor_data_.timestamp = current_time;
    return true;
}

void DataCollector::updateGyroData() {
    try {
        auto att = gyro_.getAttitude();
        auto av = gyro_.getAngularVelocity();
        auto quat = gyro_.getQuaternion();
        
        sensor_data_.gyro.attitude = Eigen::Vector3d(att.x, att.y, att.z);
        sensor_data_.gyro.raw_attitude = Eigen::Vector3d(att.x, att.y, att.z); // 保存原始值
        sensor_data_.gyro.angular_velocity = Eigen::Vector3d(av.x, av.y, av.z);
        sensor_data_.gyro.quaternion = quat;
        
        gyro_valid_ = true;
    } catch (const std::exception& e) {
        std::cerr << "[DataCollector] Failed to update gyro data: " << e.what() << std::endl;
        gyro_valid_ = false;
    }
}

void DataCollector::updateImuData() {
    try {
        ForsenseIMU::ProcessSerialData();
        auto imu_data = ForsenseIMU::GetLatestData();
        
        sensor_data_.imu.roll = imu_data.roll;
        sensor_data_.imu.pitch = imu_data.pitch;
        sensor_data_.imu.yaw = imu_data.yaw;
        sensor_data_.imu.gx = imu_data.gx;
        sensor_data_.imu.gy = imu_data.gy;
        sensor_data_.imu.gz = imu_data.gz;
        
        imu_valid_ = true;
    } catch (const std::exception& e) {
        std::cerr << "[DataCollector] Failed to update IMU data: " << e.what() << std::endl;
        imu_valid_ = false;
    }
}

void DataCollector::updateMocapData() {
    const auto& config = ConfigManager::getInstance().getConfig();
    RigidPose pose;
    
    bool received_pose = false;
    if (config.mocap_target_id >= 0) {
        received_pose = Nokov_GetPoseById(config.mocap_target_id, pose);
    } else {
        received_pose = Nokov_GetPoseByName(config.mocap_target_name, pose);
    }
    
    if (received_pose) {
        sensor_data_.mocap.valid = true;
        sensor_data_.mocap.position = Eigen::Vector3d(pose.x, pose.y, pose.z);
        sensor_data_.mocap.quaternion = Eigen::Quaterniond(pose.qw, pose.qx, pose.qy, pose.qz);
        
        sensor_data_.mocap.quaternion.normalize();
        sensor_data_.mocap.euler_angles = gyro_util::eulerZYX_degFromQuat(sensor_data_.mocap.quaternion);
        
        mocap_valid_ = true;
        
        // 动捕存在就先做外参标定
        if (!mocap_calibrated_) {
            calibrateMocapExtrinsic();
        }
    } else {
        sensor_data_.mocap.valid = false;
        mocap_valid_ = false;
    }
}

void DataCollector::calibrateMocapExtrinsic() {
    // 使用动捕数据校准陀螺仪外参
    if (sensor_data_.mocap.valid) {
        // 这里调用陀螺仪的外参标定函数
        // 注意：需要根据实际接口调整
        // gyro_.updateMocapExtrinsic(pose, true);
        mocap_calibrated_ = true;
        std::cout << "[DataCollector] Mocap extrinsic calibrated" << std::endl;
    }
}

void DataCollector::fuseData() {
    // 简单的数据融合策略：
    // 1. 优先使用动捕数据（如果有效）
    // 2. 否则使用陀螺仪数据
    // 3. IMU数据作为备用
    
    if (mocap_valid_) {
        // 使用动捕数据作为主要姿态源
        sensor_data_.gyro.attitude = sensor_data_.mocap.euler_angles;
        // 注意：动捕不提供角速度，所以角速度仍用陀螺仪
    } else if (gyro_valid_) {
        // 使用陀螺仪数据
        // 姿态数据已经是陀螺仪的
    } else if (imu_valid_) {
        // 使用IMU数据作为备用
        sensor_data_.gyro.attitude = Eigen::Vector3d(
            sensor_data_.imu.roll,
            sensor_data_.imu.pitch,
            sensor_data_.imu.yaw
        );
        sensor_data_.gyro.angular_velocity = Eigen::Vector3d(
            sensor_data_.imu.gx,
            sensor_data_.imu.gy,
            sensor_data_.imu.gz
        );
    }
}

Eigen::Vector3d DataCollector::getFusedAttitude() const {
    return sensor_data_.gyro.attitude;
}

Eigen::Vector3d DataCollector::getFusedAngularVelocity() const {
    return sensor_data_.gyro.angular_velocity;
}

const SensorData& DataCollector::getSensorData() const {
    return sensor_data_;
}

Eigen::Vector3d DataCollector::getRawGyroAttitude() const {
    return sensor_data_.gyro.raw_attitude;
}

bool DataCollector::isMocapValid() const {
    return mocap_valid_;
}

bool DataCollector::isGyroValid() const {
    return gyro_valid_;
}

bool DataCollector::isImuValid() const {
    return imu_valid_;
}

double DataCollector::getUpdateFrequency() const {
    return update_frequency_;
}
