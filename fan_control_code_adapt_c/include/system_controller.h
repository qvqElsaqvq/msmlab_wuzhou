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

    /**
     * @brief 激活硬件与模块（进入可控状态前的最后一步）
     * @return 是否激活成功
     */
    bool activateHardwareAndModules();

    /**
     * @brief 清理资源
     */
    void cleanup();

    /**
     * @brief 控制循环迭代
     */
    void controlLoopIteration(const SensorData& sensor_data);

    /**
     * @brief 速率控制，保证固定频率
     */
    void rateControl();

    /**
     * @brief 系统处于等待/未激活状态时的状态上报（降低频率）
     */
    void publishWaitingStatus(const SensorData& sensor_data);

    /**
     * @brief 计算 MQTT 消息尾部校验（当前实现为简单累加校验）
     */
    uint8_t calculateChecksum(const uint8_t* data, size_t length) const;

    // 系统组件
    std::unique_ptr<msmserial::MsMSerial> serial_;           // 与 C 板串口通信对象
    std::unique_ptr<GyroScope> gyro_;                        // 陀螺仪数据接口（从串口回调获得）
    std::unique_ptr<Fan> fan_;                               // 推力器控制（力矩模式下发）
    std::unique_ptr<Wheel> wheel_;                           // 动量轮控制/状态（串口协议）
    std::unique_ptr<LeadScrewController> leadscrew_;         // 丝杆/配重机构控制（用于调平）
    std::unique_ptr<AttitudePDController> attitude_controller_; // 姿态串级 PID 控制器（输出推力器力矩）
    std::unique_ptr<MassCenterBalancer> balancer_;           // 质心调整/自动调平逻辑
    std::unique_ptr<Docker> docker_;                         // 对接/相关任务逻辑封装
    std::unique_ptr<CallBack> mqtt_callback_;                // MQTT 回调与消息解析（上位机指令入口）
    std::unique_ptr<mqtt::async_client> mqtt_client_;        // MQTT 客户端（状态发布/订阅指令）
    
    // 新模块
    std::unique_ptr<DataCollector> data_collector_;          // 传感器/动捕数据采集与缓存
    std::unique_ptr<ControlModeManager> control_mode_manager_; // 控制模式状态机与目标生成
    std::unique_ptr<StatusPublisher> status_publisher_;      // 状态数据上报（MQTT/日志）

    // 控制循环相关
    std::atomic<bool> running_{false};                       // 主循环是否运行
    std::atomic<bool> emergency_stop_{false};                // 紧急停止标志（触发后输出归零/停止）
    std::atomic<bool> hardware_active_{false};               // 硬件是否已激活（用于区分等待/运行状态）
    std::thread control_thread_;                             // 控制线程（固定周期循环）
    std::chrono::steady_clock::time_point start_time_;       // 系统启动时间（用于 uptime）
    
    // 循环控制
    std::chrono::milliseconds loop_period_{20};              // 主循环周期（默认 20ms => 50Hz）
    std::chrono::steady_clock::time_point last_loop_time_;   // 上一次循环时间点（用于 rateControl）
    std::chrono::steady_clock::time_point last_waiting_status_publish_time_{}; // 上一次等待状态上报时间
    std::chrono::milliseconds waiting_status_publish_period_{1000};            // 等待状态上报周期

    // 配置文件
    std::string config_path_;                                // 当前使用的配置文件路径

    // 日志文件
    std::ofstream log_csv_;                                  // CSV 日志输出流
    bool logging_enabled_ = false;                           // 是否启用日志落盘
};

#endif //FAN_CONTROL_CODE_ADAPT_C_SYSTEM_CONTROLLER_H
