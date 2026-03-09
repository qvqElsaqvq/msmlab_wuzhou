//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_DATA_COLLECTOR_H
#define FAN_CONTROL_CODE_ADAPT_C_DATA_COLLECTOR_H

#include <Eigen/Dense>
#include <optional>
#include <memory>
#include "gyro_scope.h"
#include "ForsenseIMU.h"
#include "nokov_bridge.h"
#include "serial.h"

/**
 * @brief 传感器数据
 */
struct SensorData {
    // 陀螺仪数据
    struct GyroData {
        Eigen::Vector3d attitude;      // roll, pitch, yaw (deg)
        Eigen::Vector3d angular_velocity; // wx, wy, wz (deg/s)
        Eigen::Quaterniond quaternion; // 四元数
    } gyro;

    // IMU数据 (Forsense)
    struct ImuData {
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        double gx = 0.0, gy = 0.0, gz = 0.0;
    } imu;

    // 动捕数据
    struct MocapData {
        bool valid = false;
        Eigen::Vector3d position;      // x, y, z (mm)
        Eigen::Quaterniond quaternion; // qw, qx, qy, qz
        Eigen::Vector3d euler_angles;  // roll, pitch, yaw (deg)
    } mocap;

    // 时间戳
    std::chrono::steady_clock::time_point timestamp;
};

/**
 * @brief 数据采集器
 * 
 * 负责从各种传感器收集数据并进行融合处理
 */
class DataCollector {
public:
    /**
     * @brief 构造函数
     */
    DataCollector(GyroScope& gyro, msmserial::MsMSerial& serial);

    /**
     * @brief 析构函数
     */
    ~DataCollector();

    /**
     * @brief 初始化数据采集器
     */
    bool initialize();

    /**
     * @brief 更新所有传感器数据
     * @return 是否成功更新
     */
    bool update();

    /**
     * @brief 获取融合后的姿态数据
     */
    Eigen::Vector3d getFusedAttitude() const;

    /**
     * @brief 获取融合后的角速度数据
     */
    Eigen::Vector3d getFusedAngularVelocity() const;

    /**
     * @brief 获取原始传感器数据
     */
    const SensorData& getSensorData() const;

    /**
     * @brief 获取动捕数据是否有效
     */
    bool isMocapValid() const;

    /**
     * @brief 获取陀螺仪数据是否有效
     */
    bool isGyroValid() const;

    /**
     * @brief 获取IMU数据是否有效
     */
    bool isImuValid() const;

    /**
     * @brief 获取数据采集频率
     */
    double getUpdateFrequency() const;

private:
    /**
     * @brief 更新陀螺仪数据
     */
    void updateGyroData();

    /**
     * @brief 更新IMU数据
     */
    void updateImuData();

    /**
     * @brief 更新动捕数据
     */
    void updateMocapData();

    /**
     * @brief 姿态数据融合算法
     */
    void fuseData();

    /**
     * @brief 校准动捕外参
     */
    void calibrateMocapExtrinsic();

    // 传感器接口引用
    GyroScope& gyro_;
    msmserial::MsMSerial& serial_;

    // 传感器数据
    SensorData sensor_data_;

    // 动捕桥接
    std::unique_ptr<NokovBridge> nokov_bridge_;

    // 统计信息
    std::chrono::steady_clock::time_point last_update_time_;
    int update_count_ = 0;
    double update_frequency_ = 0.0;

    // 状态标志
    bool gyro_valid_ = false;
    bool imu_valid_ = false;
    bool mocap_valid_ = false;
    bool mocap_calibrated_ = false;
};

#endif //FAN_CONTROL_CODE_ADAPT_C_DATA_COLLECTOR_H
