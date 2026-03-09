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
    std::string serial_port = "/dev/ttyACM0";
    int serial_baudrate = 115200;
    std::string imu_serial_port = "/dev/ttyUSB0";

    // MQTT配置
    std::string mqtt_server_address = "tcp://localhost:1883";
    std::string mqtt_client_id = "fan_control_client";
    int mqtt_qos = 1;

    // 动捕配置
    std::string mocap_ip = "192.168.31.3";
    std::string mocap_target_name = "WUZHOUSHANG";

    // 控制参数
    int control_loop_period_ms = 20;
    int mqtt_send_interval = 5;

    // 调平参数
    int lead_screw_z_init_pos = 3000;

    // 姿态控制容差
    double angle_tolerance_deg = 1.0;
    double yaw_tolerance_deg = 3.0;
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
