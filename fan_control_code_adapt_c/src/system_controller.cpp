//
// Created by msmlab on 2025/11/14.
//

#include "system_controller.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <unistd.h>

SystemController::SystemController() {
    start_time_ = std::chrono::steady_clock::now();
    last_loop_time_ = std::chrono::steady_clock::now();
}

SystemController::~SystemController() {
    stop();
    cleanup();
}

bool SystemController::initialize(const std::string& config_path) {
    config_path_ = config_path;
    
    std::cout << "[SystemController] Initializing system..." << std::endl;
    
    // 1. 加载配置
    auto& config_manager = ConfigManager::getInstance();
    if (!config_manager.loadFromFile(config_path)) {
        std::cerr << "[SystemController] Failed to load config from: " << config_path << std::endl;
        return false;
    }
    
    const auto& config = config_manager.getConfig();
    loop_period_ = std::chrono::milliseconds(config.control_loop_period_ms);
    
    // 2. 初始化硬件接口
    if (!initializeHardware()) {
        std::cerr << "[SystemController] Failed to initialize hardware" << std::endl;
        return false;
    }
    
    // 3. 初始化MQTT
    if (!initializeMqtt()) {
        std::cerr << "[SystemController] Failed to initialize MQTT" << std::endl;
        return false;
    }
    
    // 4. 初始化动捕系统
    if (!initializeMocap()) {
        std::cerr << "[SystemController] Failed to initialize mocap" << std::endl;
        return false;
    }
    
    // 5. 初始化IMU
    if (!initializeImu()) {
        std::cerr << "[SystemController] Failed to initialize IMU" << std::endl;
        return false;
    }
    
    // 6. 初始化新模块
    data_collector_ = std::make_unique<DataCollector>(*gyro_, *serial_);
    if (!data_collector_->initialize()) {
        std::cerr << "[SystemController] Failed to initialize data collector" << std::endl;
        return false;
    }
    
    control_mode_manager_ = std::make_unique<ControlModeManager>(
        *mqtt_callback_,
        *attitude_controller_,
        *balancer_,
        *docker_,
        *fan_,
        *wheel_,
        *leadscrew_
    );
    
    status_publisher_ = std::make_unique<StatusPublisher>(
        *mqtt_client_,
        *mqtt_callback_,
        *data_collector_,
        *control_mode_manager_,
        *attitude_controller_,
        *balancer_,
        *fan_,
        *wheel_
    );
    status_publisher_->initialize();
    
    // 7. 启动串口
    serial_->setTxEnabled(false);
    serial_->spin(true);
    
    // 8. 初始化日志
    if (logging_enabled_) {
        log_csv_.open("attitude_compare.csv");
        if (log_csv_.is_open()) {
            log_csv_ << "ms,"
                     << "gyro_roll,gyro_pitch,gyro_yaw,"
                     << "mocap_roll,mocap_pitch,mocap_yaw,"
                     << "new_roll,new_pitch,new_yaw\n";
            log_csv_.flush();
        }
    }
    
    std::cout << "[SystemController] System initialized successfully" << std::endl;
    return true;
}

bool SystemController::initializeHardware() {
    const auto& config = ConfigManager::getInstance().getConfig();
    
    try {
        // 创建串口
        serial_ = std::make_unique<msmserial::MsMSerial>(config.serial_port, config.serial_baudrate);
        
        // 创建传感器和执行器
        gyro_ = std::make_unique<GyroScope>(*serial_);
        fan_ = std::make_unique<Fan>(*serial_);
        wheel_ = std::make_unique<Wheel>(*serial_);
        leadscrew_ = std::make_unique<LeadScrewController>(*serial_);
        
        // 创建控制器
        attitude_controller_ = std::make_unique<AttitudePDController>(*gyro_, *fan_, *wheel_);
        balancer_ = std::make_unique<MassCenterBalancer>(*gyro_, *fan_, *leadscrew_, *wheel_, *attitude_controller_);
        docker_ = std::make_unique<Docker>(*gyro_, *attitude_controller_);
        
        std::cout << "[SystemController] Hardware initialized" << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[SystemController] Hardware initialization error: " << e.what() << std::endl;
        return false;
    }
}

bool SystemController::initializeMqtt() {
    const auto& config = ConfigManager::getInstance().getConfig();
    
    try {
        mqtt_callback_ = std::make_unique<CallBack>();
        mqtt_client_ = std::make_unique<mqtt::async_client>(
            config.mqtt_server_address,
            config.mqtt_client_id
        );
        
        mqtt_client_->set_callback(*mqtt_callback_);
        
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);
        
        std::cout << "[SystemController] Connecting to MQTT..." << std::endl;
        std::cout << "[SystemController] MQTT server: " << config.mqtt_server_address
                  << " client_id: " << config.mqtt_client_id << std::endl;
        mqtt_client_->connect(connOpts)->wait();
        
        mqtt_client_->start_consuming();
        
        // 订阅主题
        mqtt_client_->subscribe(mqtt_callback_->cmd_plane_basic_topic, mqtt_callback_->QOS);
        mqtt_client_->subscribe(mqtt_callback_->cmd_plane_trajectory_topic, mqtt_callback_->QOS);
        mqtt_client_->subscribe(mqtt_callback_->cmd_plane_power_topic, mqtt_callback_->QOS);
        mqtt_client_->subscribe(mqtt_callback_->fan_torque_topic, mqtt_callback_->QOS);
        mqtt_client_->subscribe(mqtt_callback_->fan_velocity_topic, mqtt_callback_->QOS);
        mqtt_client_->subscribe(mqtt_callback_->wheel_test_topic, mqtt_callback_->QOS);
        mqtt_client_->subscribe(mqtt_callback_->balance_topic, mqtt_callback_->QOS);
        mqtt_client_->subscribe(mqtt_callback_->fan_calibration_topic, mqtt_callback_->QOS);
        mqtt_client_->subscribe(mqtt_callback_->fan_calibration_topic, mqtt_callback_->QOS);
        mqtt_client_->subscribe(mqtt_callback_->fan_calibration_topic1, mqtt_callback_->QOS);
        mqtt_client_->subscribe(mqtt_callback_->coop_dock_topic, mqtt_callback_->QOS);
        mqtt_client_->subscribe("attitude/activate", mqtt_callback_->QOS);
        
        std::cout << "[SystemController] MQTT initialized and connected" << std::endl;
        return true;
    } catch (const mqtt::exception& e) {
        std::cerr << "[SystemController] MQTT initialization error: " << e.what() << std::endl;
        return false;
    }
}

bool SystemController::initializeMocap() {
    const auto& config = ConfigManager::getInstance().getConfig();
    
    // 动捕系统在主循环中初始化，这里只返回true
    return true;
}

bool SystemController::initializeImu() {
    const auto& config = ConfigManager::getInstance().getConfig();
    
    // IMU在DataCollector中初始化，这里只返回true
    return true;
}

bool SystemController::start() {
    if (running_) {
        std::cout << "[SystemController] System is already running" << std::endl;
        return true;
    }

    running_ = true;
    emergency_stop_ = false;

    // 启动动捕系统
    const auto& config = ConfigManager::getInstance().getConfig();
    if (Nokov_Start(config.mocap_ip.c_str()) != 0) {
        std::cerr << "[SystemController] Failed to start Nokov bridge" << std::endl;
        running_ = false;
        return false;
    }
    std::cout << "[SystemController] Nokov bridge started" << std::endl;

    // 等待 Nokov SDK 稳定（添加延迟以避免 SDK 线程崩溃）
    std::cout << "[SystemController] Waiting for Nokov SDK to stabilize..." << std::endl;
    usleep(1000000); // 等待1秒
    std::cout << "[SystemController] Nokov SDK stabilized" << std::endl;

    // 启动控制线程
    control_thread_ = std::thread(&SystemController::run, this);

    std::cout << "[SystemController] System started" << std::endl;
    return true;
}

void SystemController::stop() {
    if (!running_) {
        return;
    }
    
    std::cout << "[SystemController] Stopping system..." << std::endl;
    
    running_ = false;
    emergency_stop_ = false;
    
    // 等待控制线程结束
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    
    // 停止动捕系统
    Nokov_Stop();
    std::cout << "[SystemController] Nokov bridge stopped" << std::endl;
    
    // 断开MQTT
    if (mqtt_client_) {
        try {
            mqtt_client_->stop_consuming();
            mqtt_client_->disconnect()->wait();
            std::cout << "[SystemController] MQTT disconnected" << std::endl;
        } catch (const mqtt::exception& e) {
            std::cerr << "[SystemController] MQTT disconnect error: " << e.what() << std::endl;
        }
    }
    
    std::cout << "[SystemController] System stopped" << std::endl;
}

void SystemController::run() {
    last_loop_time_ = std::chrono::steady_clock::now();

    std::cout << "[SystemController] Control loop started" << std::endl;

    while (running_ && !emergency_stop_) {
        try {
            if (mqtt_callback_ && serial_ && !serial_->isTxEnabled() && mqtt_callback_->hasTaskCommandReceived()) {
                std::cout << "[SystemController] Task command received, enabling MCU TX..." << std::endl;
                serial_->setTxEnabled(true);
            }
            controlLoopIteration();
            rateControl();
        } catch (const std::exception& e) {
            std::cerr << "[SystemController] Control loop error: " << e.what() << std::endl;
            // 继续运行，不退出
        } catch (...) {
            std::cerr << "[SystemController] Unknown exception in control loop" << std::endl;
            throw;
        }
    }

    std::cout << "[SystemController] Control loop stopped" << std::endl;
}

void SystemController::controlLoopIteration() {
    // 1. 更新传感器数据
    if (!data_collector_->update()) {
        std::cerr << "[SystemController] Failed to update sensor data" << std::endl;
        return;
    }

    // 2. 获取当前传感器数据
    auto sensor_data = data_collector_->getSensorData();

    // 3. 更新控制模式
    control_mode_manager_->update(sensor_data);

    // 4. 更新状态发布器
    status_publisher_->update();

    // 5. 记录日志（如果启用）
    if (logging_enabled_ && log_csv_.is_open()) {
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_time_).count();

        // 这里简化日志记录，实际应该记录更多数据
        log_csv_ << ms << ","
                 << sensor_data.gyro.attitude.x() << ","
                 << sensor_data.gyro.attitude.y() << ","
                 << sensor_data.gyro.attitude.z() << ","
                 << (sensor_data.mocap.valid ? sensor_data.mocap.euler_angles.x() : 0) << ","
                 << (sensor_data.mocap.valid ? sensor_data.mocap.euler_angles.y() : 0) << ","
                 << (sensor_data.mocap.valid ? sensor_data.mocap.euler_angles.z() : 0) << ","
                 << sensor_data.imu.roll << ","
                 << sensor_data.imu.pitch << ","
                 << sensor_data.imu.yaw << "\n";

        // 定期刷新
        static int flush_cnt = 0;
        if (++flush_cnt >= 50) {
            log_csv_.flush();
            flush_cnt = 0;
        }
    }
}

void SystemController::rateControl() {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = current_time - last_loop_time_;

    auto sleep_time = loop_period_ - elapsed;

    if (sleep_time > std::chrono::milliseconds(0)) {
        auto sleep_ms = std::chrono::duration_cast<std::chrono::milliseconds>(sleep_time).count();
        usleep(sleep_ms * 1000);
    } else {
        // 循环超时，警告
        auto overtime = -sleep_time;
        if (overtime > std::chrono::milliseconds(5)) {
            std::cerr << "[SystemController] Control loop overtime: "
                      << std::chrono::duration_cast<std::chrono::microseconds>(overtime).count()
                      << "us" << std::endl;
        }
    }

    last_loop_time_ = std::chrono::steady_clock::now();
}

void SystemController::cleanup() {
    // 关闭日志文件
    if (log_csv_.is_open()) {
        log_csv_.close();
    }
    
    // 清理所有资源
    serial_.reset();
    gyro_.reset();
    fan_.reset();
    wheel_.reset();
    leadscrew_.reset();
    attitude_controller_.reset();
    balancer_.reset();
    docker_.reset();
    mqtt_callback_.reset();
    mqtt_client_.reset();
    data_collector_.reset();
    control_mode_manager_.reset();
    status_publisher_.reset();
}

bool SystemController::isRunning() const {
    return running_;
}

void SystemController::emergencyStop() {
    emergency_stop_ = true;
    control_mode_manager_->emergencyStop();
    std::cout << "[SystemController] Emergency stop activated" << std::endl;
}

std::string SystemController::getStatusString() const {
    std::string status;
    
    status += "System Status:\n";
    status += "  Running: " + std::string(running_ ? "Yes" : "No") + "\n";
    status += "  Emergency Stop: " + std::string(emergency_stop_ ? "Yes" : "No") + "\n";
    status += "  Uptime: " + std::to_string(getUptime()) + " seconds\n";
    
    if (data_collector_) {
        status += "  Data Collector Frequency: " + 
                  std::to_string(data_collector_->getUpdateFrequency()) + " Hz\n";
        status += "  Mocap Valid: " + std::string(data_collector_->isMocapValid() ? "Yes" : "No") + "\n";
        status += "  Gyro Valid: " + std::string(data_collector_->isGyroValid() ? "Yes" : "No") + "\n";
        status += "  IMU Valid: " + std::string(data_collector_->isImuValid() ? "Yes" : "No") + "\n";
    }
    
    if (control_mode_manager_) {
        status += "  Control Mode: " + std::to_string(static_cast<int>(control_mode_manager_->getCurrentMode())) + "\n";
        status += "  Balancing: " + std::string(control_mode_manager_->isBalancing() ? "Yes" : "No") + "\n";
        status += "  Controlling Attitude: " + 
                  std::string(control_mode_manager_->isControllingAttitude() ? "Yes" : "No") + "\n";
    }
    
    return status;
}

double SystemController::getUptime() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_);
    return duration.count();
}
