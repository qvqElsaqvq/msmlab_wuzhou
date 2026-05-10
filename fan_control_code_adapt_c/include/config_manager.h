//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_CONFIG_MANAGER_H
#define FAN_CONTROL_CODE_ADAPT_C_CONFIG_MANAGER_H

#include <string>

/**
 * @brief 系统配置参数
 */
struct SystemConfig {
    // 串口配置
    std::string serial_port = "/dev/ttyACM0";      // 主控板串口设备路径
    int serial_baudrate = 115200;                  // 主控板串口波特率
    std::string imu_serial_port = "/dev/ttyUSB0";  // IMU 串口设备路径（如单独 IMU 设备）

    // MQTT配置
    std::string mqtt_server_address = "tcp://localhost:1883"; // MQTT broker 地址（tcp://IP:PORT）
    std::string mqtt_client_id = "fan_control_client";        // MQTT 客户端 ID
    int mqtt_qos = 1;                                         // MQTT QoS（0/1/2）

    // 动捕配置
    std::string mocap_ip = "192.168.31.3";          // 动捕服务器 IP（Nokov/Vicon/Optitrack）
    std::string mocap_target_name = "WUZHOUSHANG";  // 动捕刚体/目标名称（与动捕软件一致）

    // 控制参数
    int control_loop_period_ms = 20; // 主控制循环周期（ms）
    int mqtt_send_interval = 5;      // MQTT 状态发布间隔（按控制循环计数）

    // 调平参数
    int lead_screw_z_init_pos = 3000; // 调平机构 Z 轴初始位置（步数/编码器计数等）

    // 姿态控制容差
    double angle_tolerance_deg = 1.0; // roll/pitch 容差（deg）
    double yaw_tolerance_deg = 3.0;   // yaw 容差（deg）
};

/**
 * @brief 配置管理器
 */
class ConfigManager {
public:
    /**
     * @brief 获取单例实例
     */
    static ConfigManager& getInstance();

    /**
     * @brief 加载配置文件
     * @param config_path 配置文件路径
     * @return 是否加载成功
     */
    bool loadFromFile(const std::string& config_path);

    /**
     * @brief 获取系统配置
     */
    const SystemConfig& getConfig() const;

    /**
     * @brief 保存当前配置到文件
     * @param config_path 输出文件路径
     * @return 是否保存成功
     */
    bool saveToFile(const std::string& config_path);

private:
    ConfigManager() = default;
    ~ConfigManager() = default;
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    SystemConfig config_;
};

#endif //FAN_CONTROL_CODE_ADAPT_C_CONFIG_MANAGER_H
