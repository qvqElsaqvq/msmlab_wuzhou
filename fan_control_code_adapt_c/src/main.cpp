#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include "leadscrew_controller.h"
#include "wheel.h"
#include "fan.h"
#include "attitude_pd_controller.h"
#include "mass_center_balancer.h"
#include "serial.h"
#include "gyro_scope.h"
#include "MQTT_server.h"
#include "nokov_bridge.h"
#include "docker.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <ctime>

#include "ForsenseIMU.h"


using Vec3 = Eigen::Vector3d;

msmserial::MsMSerial msm_serial("/dev/ttyACM0", 115200); //串口

GyroScope gyro(msm_serial);
LeadScrewController leadscrew(msm_serial);
Wheel wheel(msm_serial);
Fan fan(msm_serial);
AttitudePDController controller(gyro, fan, wheel, msm_serial);
MassCenterBalancer balancer(gyro, fan, leadscrew, wheel, controller);
Docker docker(controller);

/* ---------------- 线程任务 ---------------- */
void do_balance_task(MassCenterBalancer &balancer) {
    std::cout << "执行自动调平算法...\n";
    std::vector<int16_t> action{-1000};
    leadscrew.moveTo(action); // 先降 Z 轴质量块
    balancer.balance_axes();
}

void do_attitude_control_task(AttitudePDController &controller,
                              const Attitude &target, const std::optional<GyroScope::Vec3> other_av = std::nullopt,
                              const std::optional<GyroScope::Vec3> other_at = std::nullopt) {
    // std::cout << "执行姿态控制算法rpy: " << target.roll << ", " << target.pitch << ", " << target.yaw << std::endl;
    Vec3 euler_angle_target(target.roll, target.pitch, target.yaw);
    controller.setAttitudeInBalancing(euler_angle_target, other_av, other_at);
    // std::cout << "姿态控制完成\n";
}

int send_flag = 0;

int main() {
    msm_serial.spin(true);
    // std::cout << "[测试串口通信]" << std::endl;

    bool if_finish_balancing = false;
    bool if_poweroff = false;

    // 1) 先启动 Nokov（你可以写死 IP，也可以从配置文件读）
    const char *mocap_ip = "192.168.31.3";
    if (Nokov_Start(mocap_ip) != 0) {
        std::cerr << "[NOKOV] start failed\n";
        return 1;
    }

    /* MQTT通信部分 */
    CallBack cb;
    mqtt::async_client client(cb.SERVER_ADDRESS, cb.CLIENT_ID);
    send_flag = 0;

    client.set_callback(cb);

    mqtt::connect_options connOpts;
    connOpts.set_clean_session(false);

    ForsenseIMU::InitSerial("/dev/ttyUSB0");

    bool flag_balancing = false; // 自动调平是否完成
    bool set_balancing = false; // 是否触发自动调平
    bool flag_fan_calibration = false; // 旋翼校准是否完成
    bool set_fan_calibration = false; // 是否触发旋翼校准
    bool current_balance_status = false; // 当前调平状态，是否已经在调平中

    try {
        std::cout << "Connecting..." << std::endl;
        client.connect(connOpts)->wait();

        client.start_consuming();
        client.subscribe(cb.cmd_plane_basic_topic, cb.QOS);
        client.subscribe(cb.cmd_plane_trajectory_topic, cb.QOS);
        client.subscribe(cb.cmd_plane_power_topic, cb.QOS);
        client.subscribe(cb.fan_torque_topic, cb.QOS);
        client.subscribe(cb.fan_velocity_topic, cb.QOS);
        client.subscribe(cb.wheel_test_topic, cb.QOS);
        client.subscribe(cb.balance_topic, cb.QOS);
        client.subscribe(cb.fan_calibration_topic, cb.QOS);
        std::cout << "Subscribe topic " << std::endl;
        std::cout << cb.cmd_plane_basic_topic << std::endl;
        std::cout << cb.cmd_plane_trajectory_topic << std::endl;
        std::cout << cb.cmd_plane_power_topic << std::endl;
        std::cout << cb.fan_torque_topic << std::endl;
        std::cout << cb.fan_velocity_topic << std::endl;
        std::cout << cb.wheel_test_topic << std::endl;
        std::cout << cb.balance_topic << std::endl;
        std::cout << cb.fan_calibration_topic << std::endl;
        std::cout << "Connected!" << std::endl;

        Attitude target_attitude; // 用于存储目标姿态
        std::ofstream log_csv("attitude_compare.csv");
        log_csv << "ms,"
                << "gyro_roll,gyro_pitch,gyro_yaw,"
                << "mocap_roll,mocap_pitch,mocap_yaw,"
                << "new_roll,new_pitch,new_yaw\n";
        log_csv.flush();
        while (true) {

            ForsenseIMU::ProcessSerialData();

            auto new_imu_data = ForsenseIMU::GetLatestData();

            RigidPose pose;
            bool received_pose = Nokov_GetPoseByName("WUZHOUSHANG", pose); // 获取动捕数据
            gyro.updateMocapExtrinsic(pose, received_pose); // ✅ 新增：动捕存在就先做外参标定（内部自动一次性完成）

            if_poweroff = cb.getIfPowerOff();
            fan.setIfPowerOff(if_poweroff);
            wheel.setIfPowerOff(if_poweroff);
            leadscrew.setIfPowerOff(if_poweroff);
            // balancer.setIfPowerOff(if_poweroff);
            // 接收数据更新并执行流程
            if (cb.getIfReceiveFanTorque()) {
                // 退出可能的自动调平/闭环姿态控制状态
                current_balance_status = false;
                balancer.reset_balance();
                controller.setIfFinishBalancing(false);

                TorqueData t = cb.getFanTorqueData();
                fan.sendTorque(t.tx, t.ty, t.tz);
            }
            // 1) 速度控制模式（第二优先级）
            else if (cb.getIfReceiveFanVelocity()) {
                current_balance_status = false;
                balancer.reset_balance();
                controller.setIfFinishBalancing(false);

                auto w = cb.getFanVelData();
                Vec3 wTarget(w.wx, w.wy, w.wz);
                controller.setAngularVelocityInControl(wTarget); // 速度闭环→力矩→sendTorque
            } else if (cb.getIfNeedBalancing()) // cb.getIfNeedBalancing()
            {
                if (!current_balance_status) {
                    std::cout << "执行自动调平算法...\n";
                    std::vector<int16_t> action{3000};
                    leadscrew.moveTo(action); // 先降 Z 轴质量块
                    std::cout << "[FAN-Z] 发送 Z 轴移动指令，位置改变 3000" << std::endl;
                    current_balance_status = true;
                }

                balancer.balance_axes();
                if_finish_balancing = balancer.getIfFinishBalancing();
                controller.setIfFinishBalancing(if_finish_balancing);
                if (if_finish_balancing) {
                    cb.setFlagBalance(true);
                    current_balance_status = false;
                    flag_balancing = true;
                    set_balancing = false;
                    balancer.reset_balance();
                }
            } else if (cb.getIfReceiveAttitudeControl()) {
                AttitudeData angle = cb.getAttitudeData();
                target_attitude.roll = angle.roll; // 你说：这是动捕系下的姿态角c
                target_attitude.pitch = angle.pitch;
                target_attitude.yaw = angle.yaw;

                if (received_pose) {
                    // ===== 1) 目标：动捕绝对目标 -> 平台等价目标（pitch 反号）=====
                    AttitudeData angle = cb.getAttitudeData();
                    Attitude target_mocap{angle.roll, angle.pitch, angle.yaw};

                    Attitude target_plat;
                    target_plat.roll = target_mocap.roll;
                    target_plat.pitch = -target_mocap.pitch; // ✅ 关键：pitch 反号
                    target_plat.yaw = target_mocap.yaw;

                    // 目标四元数（平台等价）
                    Eigen::Quaterniond qT =
                            gyro_util::quatFromEulerZYX_deg(target_plat.roll,
                                                            target_plat.pitch,
                                                            target_plat.yaw);

                    // ===== 2) 当前：动捕当前 -> 转欧拉 -> pitch 反号 -> 再转四元数 =====
                    Eigen::Quaterniond qM_raw(pose.qw, pose.qx, pose.qy, pose.qz);
                    qM_raw.normalize();

                    // 用你统一的欧拉提取（ZYX, deg）
                    Eigen::Vector3d rpyM = gyro_util::eulerZYX_degFromQuat(qM_raw); // [roll,pitch,yaw]
                    rpyM.y() = -rpyM.y(); // ✅ 关键：pitch 反号

                    Eigen::Quaterniond qM =
                            gyro_util::quatFromEulerZYX_deg(rpyM.x(), rpyM.y(), rpyM.z());

                    // ===== 3) 误差驱动：动捕当前(平台等价) -> 目标(平台等价) =====
                    Eigen::Quaterniond qDelta = qM.inverse() * qT;
                    qDelta.normalize();

                    // ===== 4) 陀螺闭环目标：当前陀螺姿态 * 误差 =====
                    Eigen::Quaterniond qG_cur = gyro.getQuaternion();
                    qG_cur.normalize();

                    Eigen::Quaterniond qG_tgt = qG_cur * qDelta;
                    qG_tgt.normalize();

                    // ===== 5) 给 PID 的欧拉目标（平台系）=====
                    Eigen::Vector3d rpyG_tgt = gyro_util::eulerZYX_degFromQuat(qG_tgt);
                    target_attitude.roll = rpyG_tgt.x();
                    target_attitude.pitch = rpyG_tgt.y();
                    target_attitude.yaw = rpyG_tgt.z();
                    auto gatt = gyro.getAttitude();
                    if (fabs(gatt.x - target_attitude.roll) < 1.0f and fabs(gatt.y - target_attitude.pitch) < 1.0f and
                        fabs(gatt.z - target_attitude.yaw) < 3.0f) {
                        controller.setIfFinishBalancing(true);
                    }
                    std::cout << "[Target Converted M->G] roll=" << target_attitude.roll
                            << ", pitch=" << target_attitude.pitch
                            << ", yaw=" << target_attitude.yaw << std::endl;
                } else {
                    controller.setIfFinishBalancing(true);
                }
                // auto now = std::chrono::steady_clock::now();
                // std::chrono::duration<double, std::milli> t = now.time_since_epoch();
                //
                // target_attitude.roll = 5 * sin(2 * 3.14 / 180 * t.count() / 1000); //+=5
                // target_attitude.pitch = 5 * sin(2 * 3.14 / 180 * t.count() / 1000); //+=5
                // target_attitude.yaw = 30 * sin(2 * 3.14 / 180 * t.count() / 1000); //+=30
                //
                // std::cout << target_attitude.roll << " " << target_attitude.pitch << " " << target_attitude.yaw << std::endl;
                do_attitude_control_task(controller, target_attitude);
            }else if (cb.getIfReceiveCoopDock()) {
                CooperationDockData cooperation_dock_data = cb.getCoopDockData();
                bool status = docker.docking(cooperation_dock_data);
                if (!status) {
                    std::cout << "[Cooperation Docker] Docking error, failed to get pose!" << std::endl;
                }
            }

            Attitude att{0.0, 0.0, 0.0};
            // 发送数据更新并上发
            if (received_pose) {
                // 1) 动捕四元数 -> 动捕系欧拉角（ZYX，deg）
                Eigen::Quaterniond qM(pose.qw, pose.qx, pose.qy, pose.qz);
                qM.normalize();

                Eigen::Vector3d rpyM_deg = gyro_util::eulerZYX_degFromQuat(qM); // [roll,pitch,yaw]
                att.roll = rpyM_deg.x();
                att.pitch = rpyM_deg.y();
                att.yaw = rpyM_deg.z();
                auto gyro_att = gyro.getAttitude();
                // std::cout << "[Angle form 动捕] roll=" << att.roll
                //               << ", pitch=" << att.pitch
                //               << ", yaw=" << att.yaw << std::endl;
                //  std::cout << "[Angle form 陀螺仪] roll=" << gyro_att.x
                //                << ", pitch=" << gyro_att.y
                //                << ", yaw=" << gyro_att.z << std::endl;
                // std::cout << "[Angle form 新陀螺仪] roll=" << new_imu_data.roll
                //               << ", pitch=" << new_imu_data.pitch
                //               << ", yaw=" << new_imu_data.yaw << std::endl;
            } else {
                // 2) 没动捕就用陀螺仪姿态角（deg）
                auto gyro_att = gyro.getAttitude();
                att.roll = gyro_att.x;
                att.pitch = gyro_att.y;
                att.yaw = gyro_att.z;
            }


            // auto gatt = gyro.getAttitude();
            // static auto t0 = std::chrono::steady_clock::now();
            // auto now = std::chrono::steady_clock::now();
            // auto ms =
            //         std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();

            // std::chrono::duration<double, std::milli> t = now.time_since_epoch();
            //
            // target_attitude.roll = 5 * sin(2 * 3.14 / 180 * t.count() / 1000); //+=5
            // target_attitude.pitch = 5 * sin(2 * 3.14 / 180 * t.count() / 1000); //+=5
            // target_attitude.yaw = 30 * sin(2 * 3.14 / 180 * t.count() / 1000); //+=30
            //
            // // std::cout << target_attitude.roll << " " << target_attitude.pitch << " " << target_attitude.yaw << std::endl;
            // do_attitude_control_task(controller, target_attitude,
            //     GyroScope::Vec3{-new_imu_data.gx, new_imu_data.gy, -new_imu_data.gz},
            //     GyroScope::Vec3{-new_imu_data.roll, new_imu_data.pitch, -new_imu_data.yaw});
            //
            // log_csv << ms << ","
            //         << std::fixed << std::setprecision(6)
            //         << gatt.x << "," << gatt.y << "," << gatt.z << ","
            //         << att.roll << "," << att.pitch << "," << att.yaw << ","
            //         << new_imu_data.roll << "," << new_imu_data.pitch << "," << new_imu_data.yaw
            //         << "\n";

            // // 可选：防止掉电丢数据
            // static int flush_cnt = 0;
            // if (++flush_cnt >= 50) {
            //     log_csv.flush();
            //     flush_cnt = 0;
            // }

            auto gyro_av = gyro.getAngularVelocity();
            AngularVel av{gyro_av.x, gyro_av.y, gyro_av.z};
            auto wheel_data = wheel.getStatus();

            // flag_balancing = balancer.getIfFinishBalancing();
            set_balancing = balancer.getIfInBalancing();

            send_flag++;
            if (send_flag >= 5) {
                // 数据上发
                for (int i = 0; i < 1; i++) {
                    Plane data{};
                    data.head.head = 0x4D47;
                    data.tail.checksum = 0x00;
                    data.data.device_id = 0x05;
                    data.data.platform_type = 0xF4;
                    data.data.cmd_count = 0x01;
                    data.data.file_count = 0x01;
                    if (if_poweroff)
                        data.data.platform_status = 0x00;
                    else
                        data.data.platform_status = 0x01;

                    data.data.gyro_fault = 0x00;
                    data.data.wx = (int16_t) (gyro_av.x * 100.0f);
                    data.data.wy = (int16_t) (gyro_av.y * 100.0f);
                    data.data.wz = (int16_t) (gyro_av.z * 100.0f);
                    data.data.roll = (int16_t) (att.roll * 100.0f);
                    data.data.pitch = (int16_t) (att.pitch * 100.0f);
                    data.data.yaw = (int16_t) (att.yaw * 100.0f);

                    data.data.wheel_dir = wheel_data.dir;
                    data.data.wheel_current = 0;
                    data.data.wheel_rpm = wheel_data.speed;
                    data.data.wheel_fault = 0x00;
                    data.data.payload_mass = 0;
                    data.data.pwr_v1 = 0;
                    data.data.pwr_v2 = 0;
                    data.data.pwr_v3 = 0;
                    data.data.pwr_v4 = 0;
                    data.data.pwr_i1 = 0;
                    data.data.pwr_i2 = 0;
                    data.data.pwr_i3 = 0;
                    data.data.pwr_i4 = 0;
                    data.data.battery_percent = 100;
                    data.data.traj_ready = 0x01;

                    data.data.thrust_x = 0;
                    data.data.thrust_y = 0;
                    data.data.thrust_z = 0;

                    Vec3 current_torque = controller.getTorque();
                    data.data.torque_roll = (int16_t) (current_torque.x() * 100.0f);
                    data.data.torque_pitch = (int16_t) (current_torque.y() * 100.0f);
                    data.data.torque_yaw = (int16_t) (current_torque.z() * 100.0f);

                    data.data.balance_flag = flag_balancing;
                    data.data.balance_set = set_balancing;
                    data.data.fan_calibration_flag = flag_fan_calibration;
                    data.data.fan_calibration_set = set_fan_calibration;

                    for (int i = 0; i < 7; i++) {
                        data.data.reserved[i] = 0x00;
                    }
                    client.publish(cb.plane_data_topic, &data, sizeof(data), cb.QOS, false);
                    // std::cout << "---------send upper---------" << std::endl;
                    send_flag = 0;
                }
            }

            // 控制循环频率
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    } catch (const mqtt::exception &e) {
        std::cerr << "Mqtt Error: " << e.what() << std::endl;
        client.stop_consuming();
        client.disconnect()->wait();
        std::cerr << "[MQTT] 断开连接" << std::endl;
        return 1;
    }
    Nokov_Stop();
    std::cout << "[NOKOV] stopped\n";

    client.stop_consuming();
    client.disconnect()->wait();
    std::cout << "[MQTT] 断开连接" << std::endl;

    return 0;
}
