//
// Created by mijiao on 2025/10/31.
//

#ifndef SATELLITE_MESSAGE_H
#define SATELLITE_MESSAGE_H

#include "serialPro/serialPro.h"

/* 平面气浮台和C板通信部分 */
message_data Head {
    uint16_t frame_head = 0x6F6E; // 0x6F,0x6E
    uint8_t command = 0;
};

message_data Tail {
    uint16_t checksum = 0xEDED; // 0xED,0xED
};

// 发送
message_data FanControl{ // 0x01
    uint8_t device_id;  // 设备号 00
    uint8_t direction;  // 后三位分别表示x，y，z的力的方向，0表示正向，1表示负向。例如：04（0000 0100）表示x负向，y，z正向
    uint16_t torque_x;  // X轴方向推力大小
    uint16_t torque_y;  // Y轴方向推力大小
    uint16_t torque_z;  // Z轴方向推力大小
};

message_data WheelControl{ // 0x02
    uint8_t device_id;  // 设备编号 01/02/03（动量轮）
    uint8_t direction;  // 动量轮旋转方向 55（正向）、AA（反向）
    uint8_t current;  // 动量轮电流 符号字符型*100
};

message_data FanCalibrationControl{ // 0x03
    uint8_t device_id;  // 设备号 1A
    uint8_t fan_set; // 旋翼校准触发位, 内容：00（不触发），01（触发）
    uint8_t fan_reset; // 内容：00（不触发），01（触发）内容：00（不重置） 01（重置）
};

message_data LeadScrewControl{ // 0x04
    uint8_t device_id;  // 设备编号 内容：0x3A
    int16_t dist_x;  // x轴丝杆电机移动步长
    int16_t dist_y;  // y轴丝杆电机移动步长
    int16_t dist_z;  // z轴丝杆电机移动步长
};

// 接收
message_data GyroScopeData{ // 0x05
    uint8_t device_id;  // 设备号 06
    int16_t wx; // 4~5  角速度wx ×100
    int16_t wy; // 6~7 角速度wy ×100
    int16_t wz; // 8~9 角速度wz ×100
    int16_t yaw; // 10~11 偏航角 ×100
    int16_t pitch; // 12~13 俯仰角 ×100
    int16_t roll; // 14~15 滚转角 ×100
};

message_data WheelData{ // 0x06
    uint8_t device_id;  // 设备编号 01/02/03（动量轮）
    uint8_t direction;  // 动量轮旋转方向 55（正向）、AA（反向）
    int16_t speed;  // 动量轮转速 0x01F4-0x1388 （对应500-5000rpm）
};

message_data PowerData{ // 0x07
    uint8_t device_id; // 设备编号 内容：05
    uint8_t V_channel_1;
    uint8_t V_channel_2;
    uint8_t V_channel_3;
    uint8_t V_channel_4;
    uint8_t A_channel_1;
    uint8_t A_channel_2;
    uint8_t A_channel_3;
    uint8_t A_channel_4;
};

message_data FanCalibrationData{ // 0x08
    uint8_t device_id; // 设备编号 内容：1B
    uint8_t fan_flag; // 旋翼校准标志位 内容：00（未完成），01（已完成）
    uint8_t fan_set; // 旋翼校准触发位 内容：00（未触发），01（触发中）
};

message_data LeadScrewAlarm{ // 0x09
    uint8_t device_id; // 设备编号 内容：1F
    uint8_t alarm_x; // x轴丝杆撞边报警 内容：00（未触发），01（右边），02（左边）
    uint8_t alarm_y; // y轴丝杆撞边报警
    uint8_t alarm_z; // z轴丝杆撞边报警
};

message_data WheelInit{
    uint8_t device_id; // 设备编号 内容：5A
    int16_t target_roll;
    int16_t target_pitch;
    int16_t target_yaw;
    uint8_t flag_balance;
};

/* 平面气浮台和上位机MQTT通信部分 */
message_data NUC_Head{
    uint16_t head;
};

message_data NUC_Tail{
    uint8_t checksum;
};

// 平面气浮台传上位机指令格式 attitude/data
message_data PlaneData {
    uint8_t device_id; // 3    设备号 内容：01/02/03……
    uint8_t platform_type; // 4    气浮台类型 0xF1(平面气浮台)/0xF2(姿态气浮台)
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
    int16_t pwr_v1; // 29~30   电源通道1 电压×10
    int16_t pwr_v2; // 31~32   电源通道2 电压×10
    int16_t pwr_v3; // 33~34   电源通道3 电压×10
    int16_t pwr_v4; // 35~36   电源通道4 电压×10
    int16_t pwr_i1; // 37~38   电源通道1 电流×10
    int16_t pwr_i2; // 39~40   电源通道2 电流×10
    int16_t pwr_i3; // 41~42   电源通道3 电流×10
    int16_t pwr_i4; // 43~44   电源通道4 电流×10
    uint8_t battery_percent; // 45   电源电量 百分比显示整数
    uint8_t traj_ready; // 46   内置轨迹准备完成 00未准备01准备完毕
    int16_t thrust_x; // 47~48 X方向推力 ×100
    int16_t thrust_y; // 49~50 Y方向推力 ×100
    int16_t thrust_z; // 51~52 Z方向推力（平面） ×100
    int16_t torque_yaw; // 53~54 偏航力矩 ×100
    int16_t torque_pitch; // 55~56 俯仰力矩 ×100
    int16_t torque_roll; // 57~58 滚转力矩 ×100
    uint8_t balance_flag; // 59 自动调平标志位 内容：00（未调平）、01（调平完成）
    uint8_t balance_set; // 60 自动调平触发位 是否触发自动调平 内容：00（未触发）、01（触发中）
    uint8_t fan_calibration_flag; // 61 旋翼校准标志位 旋翼校准是否完成 内容：00（未完成） 01（校准完成）
    uint8_t fan_calibration_set; // 62 旋翼校准触发位 是否触发旋翼校准 内容：00（未触发） 01（触发中）
    uint8_t reserved[7]; // 51~60 保留
};

// 上位机传平面气浮台基础指令 attitude/basic
message_data CmdPlaneBasic {
    uint8_t device_id; // 3    设备号 内容：01/02/03……
    uint8_t cmd_type; // 4    指令类型 0x10
    int32_t pos_x; // 5~8  位置x ×100
    int32_t pos_y; // 9~12  位置y ×100
    int32_t rot_z; // 13~16 旋转z ×100
    int16_t yaw; // 17~18 偏航角 ×100
    int16_t pitch; // 19~20 俯仰角 ×100
    int16_t roll; // 21~22 滚转角 ×100
};

// 上位机传平面气浮内置轨迹指令 attitude/trajectory
message_data CmdPlaneTrajectory {
    uint8_t device_id; // 3    设备号 内容：01/02/03……
    uint8_t cmd_type; // 4    指令类型 0x11
    uint8_t traj_id; // 5    内置轨迹ID 01/02/03
    uint8_t start; // 6    00不开始 01开始
};

// 上位机传平面气浮台开关机指令 attitude/power
message_data CmdPlanePower {
    uint8_t device_id; // 3    设备号 内容：01/02/03……
    uint8_t cmd_type; // 4    指令类型 内容：0x12
    uint32_t cmd_data; // 5    指令内容 内容：0x00 执行中 0x01 停机
};

message_data FanTestData{
    uint8_t device_id; // 3  设备号 内容：01/02/03……
    uint8_t cmd_type; // 4    指令类型 内容：0x13
    uint8_t fan_dir; // 5  推力方向后三位分别表示x，y，z的力的方向，0表示正向，1表示负向.例如：04（0000 0100）表示x负向，y，z正向
    uint8_t torque_x; // 6  ×100
    uint8_t torque_y; // 7  ×100
    uint8_t torque_z; // 8  ×100
};

message_data WheelTestData{
    uint8_t device_id; // 3  设备号 内容：01/02/03……
    uint8_t cmd_type; // 4    指令类型 内容：0x14
    uint8_t wheel_model; // 5  动量轮模式 F1（电流模式）F2（转速模式）
    uint8_t wheel_dir; // 6  动量轮旋转方向 55（正向）、AA（反向）
    int16_t wheel_current; // 7-8  动量轮电流 ×100
    int16_t wheel_rpm; // 9-10  动量轮转速 内容：0x01F4-0x1388 （对应500-5000rpm）
};

message_data BalanceData{
    uint8_t len; // 3  数据长度 内容：0x06
    uint8_t cmd; // 4  遥控命令 0x05
    uint8_t device_id; // 5 气浮台ID 01（姿态气浮台）
    uint8_t balance_set; // 6 自动调平触发位 00（不触发）、01（触发）
    uint8_t balance_reset; // 7 自动调平重置位 00（不重置）、01（触发）
};

message_data FanCalibrationMQTTData{
    uint8_t len; // 3  数据长度 内容：0x04
    uint8_t cmd; // 4  遥控命令 0xFF
    uint8_t device_id; // 5 气浮台ID  内容：01（姿态气浮台）
    uint8_t fan_set; // 旋翼校准触发位 内容：00（不触发）、01（触发）
    uint8_t fan_reset; // 旋翼校准重置位 内容：00（不重置）、01（触发）
};

message_data Plane {
    NUC_Head head; // 0x4D,0x47
    PlaneData data;
    NUC_Tail tail;
};

message_data CmdBasic {
    NUC_Head head; // 0x1D,0x97
    CmdPlaneBasic data;
    NUC_Tail tail;
};

message_data CmdTrajectory {
    NUC_Head head; // 0x1D,0x97
    CmdPlaneTrajectory data;
    NUC_Tail tail;
};

message_data CmdPower {
    NUC_Head head; // 0x1D,0x97
    CmdPlanePower data;
    NUC_Tail tail;
};

message_data FanTest{
    NUC_Head head; // 0x1D,0x97
    FanTestData data;
    NUC_Tail tail;
};

message_data WheelTest{
    NUC_Head head; // 0x1D,0x97
    WheelTestData data;
    NUC_Tail tail;
};

message_data Balance{
    NUC_Head head; // 0x5A,0x47
    BalanceData data;
    NUC_Tail tail;
};

message_data FanCalibration{
    NUC_Head head; // 0x5A,0x47
    FanCalibrationMQTTData data;
    NUC_Tail tail;
};

#endif //SATELLITE_MESSAGE_H
