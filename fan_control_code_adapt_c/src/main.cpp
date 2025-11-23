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

using Vec3 = Eigen::Vector3d;

msmserial::MsMSerial msm_serial("/dev/ttyACM0", 115200); //串口

GyroScope gyro(msm_serial);
LeadScrewController leadscrew(msm_serial);
Wheel wheel(msm_serial);
Fan fan(msm_serial);
AttitudePDController controller(gyro, fan);
MassCenterBalancer balancer(gyro, fan, leadscrew, wheel, controller);

/* ---------------- 线程任务 ---------------- */
void do_balance_task(MassCenterBalancer &balancer) {
    std::cout << "执行自动调平算法...\n";
    std::vector<int16_t> action{-1000};
    leadscrew.moveTo(action); // 先降 Z 轴质量块
    balancer.balance_both_axes_fan(); // XY轴调平
    balancer.balance_z_axes_fan(); // Z轴调平
    std::cout << "调平完成" << std::endl;
}

void do_attitude_control_task(AttitudePDController &controller,
                              const Attitude &target) {
    // std::cout << "执行姿态控制算法rpy: " << target.roll << ", " << target.pitch << ", " << target.yaw << std::endl;
    Vec3 euler_angle_target(target.roll, target.pitch, target.yaw);
    controller.setAttitudeInBalancing(euler_angle_target);
    // std::cout << "姿态控制完成\n";
}

/* ---------------- main ---------------- */
int main() {
    msm_serial.spin(true);
    // std::cout << "[测试串口通信]" << std::endl;

    /* MQTT通信部分 */
    CallBack cb;
    mqtt::async_client client(cb.SERVER_ADDRESS, cb.CLIENT_ID);

    client.set_callback(cb);

    mqtt::connect_options connOpts;
    connOpts.set_clean_session(false);

    try {
        std::cout << "Connecting..." << std::endl;
        client.connect(connOpts)->wait();
        std::cout << "Connected." << std::endl;

        client.start_consuming();
        client.subscribe(cb.cmd_plane_basic_topic, cb.QOS);
        client.subscribe(cb.cmd_plane_trajectory_topic, cb.QOS);
        client.subscribe(cb.cmd_plane_power_topic, cb.QOS);
        client.subscribe(cb.fan_test_topic, cb.QOS);
        client.subscribe(cb.wheel_test_topic, cb.QOS);
        client.subscribe(cb.balance_topic, cb.QOS);
        client.subscribe(cb.fan_calibration_topic, cb.QOS);
        // std::cout << "Subscribe topic " << cb.plane_data_topic << std::endl;

        Attitude target;
        target.roll = 10;
        target.pitch = 20;
        target.yaw = 30;
        while (true) {
            // std::vector<int16_t> action{1000, 100};
            // leadscrew.moveTo(action);   // 先降 Z 轴质量块
            do_attitude_control_task(controller, target);

            for (int i = 0; i < 1; i++) {
                Plane data{};
                data.head.head = 0x4D47;
                data.tail.checksum = 0x00;
                data.data.device_id = 0x02;
                data.data.platform_type = 0xF2;
                data.data.cmd_count = 0x01;
                data.data.file_count = 0x01;
                data.data.platform_status = 0x00;
                data.data.wx = 40;
                data.data.wy = 40;
                data.data.wz = 40;
                data.data.roll = 30;
                data.data.pitch = 30;
                data.data.yaw = 30;
                data.data.gyro_fault = 0x00;
                data.data.wheel_dir = 0x55;
                data.data.wheel_current = 100;
                data.data.wheel_rpm = 3000;
                data.data.wheel_fault = 0x00;
                data.data.payload_mass = 100;
                data.data.pwr_v1 = 50;
                data.data.pwr_v2 = 50;
                data.data.pwr_v3 = 50;
                data.data.pwr_v4 = 50;
                data.data.pwr_i1 = 20;
                data.data.pwr_i2 = 20;
                data.data.pwr_i3 = 20;
                data.data.pwr_i4 = 20;
                data.data.battery_percent = 70;
                data.data.traj_ready = 0x01;
                data.data.thrust_x = 200;
                data.data.thrust_y = 200;
                data.data.thrust_z = 200;
                data.data.torque_roll = 300;
                data.data.torque_pitch = 300;
                data.data.torque_yaw = 300;
                data.data.balance_flag = 0x00;
                data.data.balance_set = 0x01;
                data.data.fan_calibration_flag = 0x00;
                data.data.fan_calibration_set = 0x01;
                for (int i = 0; i < 7; i++) {
                    data.data.reserved[i] = 0x00;
                }
                client.publish(cb.plane_data_topic, &data, sizeof(data), cb.QOS, false);
                // std::cout << "Publish Plane: " << sizeof(data) << std::endl;
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const mqtt::exception &e) {
        std::cerr << "Mqtt Error: " << e.what() << std::endl;
        client.stop_consuming();
        client.disconnect()->wait();
        std::cerr << "[MQTT] 断开连接" << std::endl;
        return 1;
    }
    client.stop_consuming();
    client.disconnect()->wait();
    std::cout << "[MQTT] 断开连接" << std::endl;
    // try
    // {
    //     while (true)
    //     {
    //         /* 1. 调平指令 */
    //         if (mqtt.flag_balance.exchange(false))
    //         {
    //             std::thread([&balancer]{
    //                 do_balance_task(balancer);
    //             }).detach();
    //         }
    //
    //         /* 2. 欧拉角姿态指令 */
    //         if (mqtt.flag_attitude_euler.exchange(false))
    //         {
    //             std::thread([&controller, &mqtt]{
    //                 do_attitude_control_task(controller,
    //                                          mqtt.attitude_data_euler);
    //             }).detach();
    //         }
    //
    //         /* 3. 四元数姿态指令 */
    //         if (mqtt.flag_attitude_quat.exchange(false))
    //         {
    //             std::thread([&controller, &mqtt]{
    //                 auto q = mqtt.attitude_data_quat;
    //                 // quat -> euler ZYX
    //                 Vec3 ea = q.toRotationMatrix().eulerAngles(2,1,0);
    //                 Attitude att{ea[2], ea[1], ea[0]};
    //                 do_attitude_control_task(controller, att);
    //             }).detach();
    //         }
    //
    //         /* 4. 主循环更新传感器数据 */
    //         Attitude  att  = gyro.getAttitude();
    //         AngularVel av  = gyro.getAngularVelocity();
    //         Eigen::Quaterniond q =
    //             Eigen::AngleAxisd(att.yaw,   Eigen::Vector3d::UnitZ()) *
    //             Eigen::AngleAxisd(att.pitch, Eigen::Vector3d::UnitY()) *
    //             Eigen::AngleAxisd(att.roll,  Eigen::Vector3d::UnitX());
    //
    //         double t = std::chrono::duration<double>(
    //             std::chrono::system_clock::now().time_since_epoch()).count();
    //         mqtt.publish_telemetry(t, att, av, q);
    //
    //         wheel.setCurrent(att, av);
    //
    //         std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //     }
    // }
    // catch (std::exception& e)
    // {
    //     std::cerr << "异常退出: " << e.what() << '\n';
    // }

    return 0;
}
