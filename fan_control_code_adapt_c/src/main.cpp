//
// 简化的主文件，使用新的模块化架构
// 原始 main.cpp 已重构为 system_controller 和其他模块
//

#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include "system_controller.h"

// 全局系统控制器实例
std::unique_ptr<SystemController> system_controller;
std::atomic<bool> running{true};

// 信号处理函数
void signalHandler(int signal) {
    (void)signal;
    running.store(false, std::memory_order_relaxed);
}

// 打印帮助信息
void printHelp() {
    std::cout << "\n气浮台控制系统\n";
    std::cout << "================\n";
    std::cout << "用法: ./fan_control_code_adapt_c [config_file]\n";
    std::cout << "参数:\n";
    std::cout << "  config_file - 配置文件路径 (可选，默认: config.ini)\n";
    std::cout << "\n控制命令:\n";
    std::cout << "  Ctrl+C  - 优雅停止\n";
    std::cout << "  Ctrl+\\  - 强制退出\n";
    std::cout << "\n模块化架构:\n";
    std::cout << "  SystemController - 主控制系统\n";
    std::cout << "  DataCollector    - 数据采集模块\n";
    std::cout << "  ControlModeManager - 控制模式管理\n";
    std::cout << "  StatusPublisher  - 状态发布模块\n";
    std::cout << "  ConfigManager    - 配置管理模块\n";
    std::cout << std::endl;
}

int main(int argc, char* argv[]) {
    // 设置命令行参数
    std::string config_file = "config.ini";
    if (argc > 1) {
        if (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
            printHelp();
            return 0;
        }
        config_file = argv[1];
    }
    
    std::cout << "==========================================" << std::endl;
    std::cout << "气浮台控制系统 - 模块化版本" << std::endl;
    std::cout << "配置文件: " << config_file << std::endl;
    std::cout << "==========================================" << std::endl;
    
    // 注册信号处理
    std::signal(SIGINT, signalHandler);   // Ctrl+C
    std::signal(SIGTERM, signalHandler);  // kill命令
    std::signal(SIGQUIT, signalHandler);  // Ctrl+\
    
    try {
        // 创建系统控制器
        system_controller = std::make_unique<SystemController>();
        
        // 初始化系统
        std::cout << "[Main] 初始化系统..." << std::endl;
        if (!system_controller->initialize(config_file)) {
            std::cerr << "[Main] 系统初始化失败!" << std::endl;
            return 1;
        }
        
        // 启动系统
        std::cout << "[Main] 启动系统..." << std::endl;
        if (!system_controller->start()) {
            std::cerr << "[Main] 系统启动失败!" << std::endl;
            return 1;
        }
        
        std::cout << "\n[Main] 系统运行中..." << std::endl;
        std::cout << "按 Ctrl+C 停止系统" << std::endl;
        
        // 主循环（等待信号）
        while (running && system_controller->isRunning()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        // 停止系统
        std::cout << "[Main] 正在停止系统..." << std::endl;
        system_controller->stop();
        
        std::cout << "[Main] 系统已停止" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "[Main] 未处理的异常: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
