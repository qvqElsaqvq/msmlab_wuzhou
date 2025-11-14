//
// Created by mijiao on 2025/10/31.
//

#ifndef SATELLITE_MESSAGE_H
#define SATELLITE_MESSAGE_H

#include "serialPro/serialPro.h"

message_data Head {
    uint8_t frame_head; // 0x4D,0x14
    uint8_t command; // 0,1,2,3
};

message_data Tail {
    uint8_t checksum; //异或
};

// 平面气浮台传上位机指令格式
message_data PlaneData {
    uint8_t device_id; // 3    设备号
    uint8_t platform_type; // 4    气浮台类型 0xF1/0xF2
    uint8_t cmd_count; // 5    指令计数
    uint8_t file_count; // 6    文件数
    uint8_t platform_status; // 7    气浮台状态 00执行中 01停机
    int16_t wx; // 8~9  角速度wx ×100
    int16_t wy; // 10~11 角速度wy ×100
    int16_t wz; // 12~13 角速度wz ×100
    int16_t yaw; // 14~15 偏航角 ×100
    int16_t pitch; // 16~17 俯仰角 ×100
    int16_t roll; // 18~19 滚转角 ×100
    uint8_t gyro_fault; // 20   陀螺仪故障码 00正常01故障
    uint8_t wheel_dir; // 21   动量轮旋转方向 55正向AA反向
    int16_t wheel_current; // 22~23 动量轮电流 ×100
    int16_t wheel_rpm; // 24~25 动量轮转速 500–5000rpm
    uint8_t wheel_fault; // 26   动量轮故障码 00正常01故障
    int16_t payload_mass; // 27~28 承载质量 ×100
    uint8_t pwr_v1; // 29   电源通道1 电压×10
    uint8_t pwr_v2; // 30   电源通道2 电压×10
    uint8_t pwr_v3; // 31   电源通道3 电压×10
    uint8_t pwr_v4; // 32   电源通道4 电压×10
    uint8_t pwr_i1; // 33   电源通道1 电流×10
    uint8_t pwr_i2; // 34   电源通道2 电流×10
    uint8_t pwr_i3; // 35   电源通道3 电流×10
    uint8_t pwr_i4; // 36   电源通道4 电流×10
    uint8_t battery_percent; // 37   电源电量 %
    uint8_t traj_ready; // 38   内置轨迹准备完成 00未准备01准备完毕
    int16_t thrust_x; // 39~40 X方向推力 ×100
    int16_t thrust_y; // 41~42 Y方向推力 ×100
    int16_t thrust_z; // 43~44 Z方向推力 ×100
    int16_t torque_yaw; // 45~46 偏航力矩 ×100
    int16_t torque_pitch; // 47~48 俯仰力矩 ×100
    int16_t torque_roll; // 49~50 滚转力矩 ×100
    uint8_t reserved[10]; // 51~60 保留
};

// 上位机传平面气浮台基础指令
message_data CmdPlaneBasic {
    uint8_t device_id; // 3    设备号
    uint8_t cmd_type; // 4    指令类型 0x10
    int16_t pos_x; // 5~6  位置x ×100
    int16_t pos_y; // 7~8  位置y ×100
    int16_t rot_z; // 9~10 旋转z ×100
    int16_t yaw; // 11~12 偏航角 ×100
    int16_t pitch; // 13~14 俯仰角 ×100
    int16_t roll; // 15~16 滚转角 ×100
};

// 上位机传平面气浮内置轨迹指令
message_data CmdPlaneTrajectory {
    uint8_t device_id; // 3    设备号
    uint8_t cmd_type; // 4    指令类型 0x11
    uint8_t traj_id; // 5    内置轨迹ID 01/02/03
    uint8_t start; // 6    00不开始 01开始
};

// 上位机传平面气浮台开关机指令
message_data CmdPlanePower {
    uint8_t device_id; // 3    设备号
    uint8_t cmd_type; // 4    0x00执行中 0x01停机
};

message_data Plane {
    Head head;
    PlaneData data;
    Tail tail;
};

message_data CmdBasic {
    Head head;
    CmdPlaneBasic data;
    Tail tail;
};

message_data CmdTrajectory {
    Head head;
    CmdPlaneTrajectory data;
    Tail tail;
};

message_data CmdPower {
    Head head;
    CmdPlanePower data;
    Tail tail;
};

#endif //SATELLITE_MESSAGE_H
