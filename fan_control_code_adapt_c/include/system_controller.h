//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_SYSTEM_CONTROLLER_H
#define FAN_CONTROL_CODE_ADAPT_C_SYSTEM_CONTROLLER_H

#include <memory>
#include <atomic>
#include <thread>
#include <chrono>
#include <fstream>
#include "serial.h"
#include "gyro_scope.h"
#include "fan.h"
#include "wheel.h"
#include "leadscrew_controller.h"
#include "attitude_pd_controller.h"
#include "mass_center_balancer.h"
#include "docker.h"
#include "MQTT_server.h"
#include "config_manager.h"
#include "data_collector.h"
#include "control_mode_manager.h"
#include "status_publisher.h"

/**
 * @brief 系统控制器
 * 
 * 主控制系统，负责协调所有模块，管理主控制循环
 */
class SystemController {
public:
    /**
     * @brief 构造函数
     */
    SystemController();

    /**
     * @brief 析构函数
     */
    ~SystemController();

    /**
     * @brief 初始化系统
     * @param config_path 配置文件路径
     * @return 是否初始化成功
     */
    bool initialize(const std::string& config_path = "config.ini");

    /**
     * @brief 启动系统
     * @return 是否启动成功
     */
    bool start();

    /**
     * @brief 停止系统
     */
    void stop();

    /**
     * @brief 主控制循环
     */
    void run();

    /**
     * @brief 检查系统是否运行中
     */
    bool isRunning() const;

    /**
     * @brief 紧急停止
     */
    void emergencyStop();

    /**
     * @brief 获取系统状态字符串
     */
    std::string getStatusString() const;

    /**
     * @brief 获取系统运行时间（秒）
     */
    double getUptime() const;

private:
    /**
     * @brief 初始化硬件接口
     */
    bool initializeHardware();

    /**
     * @brief 初始化MQTT通信
     */
    bool initializeMqtt();

    /**
     * @brief 初始化动捕系统
     */
    bool initializeMocap();

    /**
     * @brief 初始化IMU
     */
    bool initializeImu();

    bool activateHardwareAndModules();

    /**
     * @brief 清理资源
     */
    void cleanup();

    /**
     * @brief 控制循环迭代
     */
    void controlLoopIteration();

    /**
     * @brief 速率控制，保证固定频率
     */
    void rateControl();

    // 系统组件
    std::unique_ptr<msmserial::MsMSerial> serial_;
    std::unique_ptr<GyroScope> gyro_;
    std::unique_ptr<Fan> fan_;
    std::unique_ptr<Wheel> wheel_;
    std::unique_ptr<LeadScrewController> leadscrew_;
    std::unique_ptr<AttitudePDController> attitude_controller_;
    std::unique_ptr<MassCenterBalancer> balancer_;
    std::unique_ptr<Docker> docker_;
    std::unique_ptr<CallBack> mqtt_callback_;
    std::unique_ptr<mqtt::async_client> mqtt_client_;
    
    // 新模块
    std::unique_ptr<DataCollector> data_collector_;
    std::unique_ptr<ControlModeManager> control_mode_manager_;
    std::unique_ptr<StatusPublisher> status_publisher_;

    // 控制循环相关
    std::atomic<bool> running_{false};
    std::atomic<bool> emergency_stop_{false};
    std::atomic<bool> hardware_active_{false};
    std::thread control_thread_;
    std::chrono::steady_clock::time_point start_time_;
    
    // 循环控制
    std::chrono::milliseconds loop_period_{20}; // 50Hz
    std::chrono::steady_clock::time_point last_loop_time_;

    // 配置文件
    std::string config_path_;

    // 日志文件
    std::ofstream log_csv_;
    bool logging_enabled_ = false;
};

#endif //FAN_CONTROL_CODE_ADAPT_C_SYSTEM_CONTROLLER_H
