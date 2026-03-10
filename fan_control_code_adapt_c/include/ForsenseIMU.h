#pragma once
#include <cstdint>


namespace ForsenseIMU {

    // 结构体
    struct ParsedData {
        uint32_t timer; // 时间标 (us)
        float pitch;    // 俯仰角
        float roll;     // 横滚角
        float yaw;      // 航向角
        float ax;       // X轴加速度 (g)
        float ay;       // Y轴加速度 (g)
        float az;       // Z轴加速度 (g)
        float gx;       // X轴角速度 (°/s)
        float gy;       // Y轴角速度 (°/s)
        float gz;       // Z轴角速度 (°/s)
        float temp;     // 温度 (℃)
        bool  isValid;  // 是否已经成功接收到至少一帧有效数据
    };

    // 初始化串口（传入如 "COM3" 或 "\\\\.\\COM10"）
    bool InitSerial(const char* portName);

    // 核心更新函数（请在你的主循环中定时调用它，它会自动读取串口并解析）
    void ProcessSerialData();

    // 获取最新数据的接口（随时可以调用，获取当前的姿态和角速度）
    ParsedData GetLatestData();

    // 关闭串口
    void CloseSerial();
}