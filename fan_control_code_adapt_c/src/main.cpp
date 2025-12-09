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
    balancer.balance_axes();
}

void do_attitude_control_task(AttitudePDController &controller,
                              const Attitude &target) {
    // std::cout << "执行姿态控制算法rpy: " << target.roll << ", " << target.pitch << ", " << target.yaw << std::endl;
    Vec3 euler_angle_target(target.roll, target.pitch, target.yaw);
    controller.setAttitudeInBalancing(euler_angle_target);
    // std::cout << "姿态控制完成\n";
}

int send_flag = 0;

int main() {
    msm_serial.spin(true);
    // std::cout << "[测试串口通信]" << std::endl;

    /* MQTT通信部分 */
    CallBack cb;
    mqtt::async_client client(cb.SERVER_ADDRESS, cb.CLIENT_ID);
    send_flag = 0;

    client.set_callback(cb);

    mqtt::connect_options connOpts;
    connOpts.set_clean_session(false);

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
        client.subscribe(cb.fan_test_topic, cb.QOS);
        client.subscribe(cb.wheel_test_topic, cb.QOS);
        client.subscribe(cb.balance_topic, cb.QOS);
        client.subscribe(cb.fan_calibration_topic, cb.QOS);
        std::cout << "Subscribe topic " << std::endl;
        std::cout << cb.cmd_plane_basic_topic << std::endl;
        std::cout << cb.cmd_plane_trajectory_topic << std::endl;
        std::cout << cb.cmd_plane_power_topic << std::endl;
        std::cout << cb.fan_test_topic << std::endl;
        std::cout << cb.wheel_test_topic << std::endl;
        std::cout << cb.balance_topic << std::endl;
        std::cout << cb.fan_calibration_topic << std::endl;

        Attitude target;
        target.roll = 0;
        target.pitch = 0;
        target.yaw = 0;
        while (true) {
            // 接收数据更新并执行流程
            if(1)
            {
                if(!current_balance_status)
                {
                    std::cout << "执行自动调平算法...\n";
                    std::vector<int16_t> action{-1000};
                    leadscrew.moveTo(action); // 先降 Z 轴质量块
                    current_balance_status = true;
                }

                balancer.balance_axes();
                if(balancer.getIfFinishBalancing())
                {
                    cb.setFlagBalance(true);
                    current_balance_status = false;
                    flag_balancing = true;
                    set_balancing = false;
                    balancer.reset_balance();
                }
            }
            else if(0)
            {
                AttitudeData angle;
                // AttitudeData angle = cb.getAttitudeData();
                angle.pitch = 0.0;
                angle.roll = 0.0;
                angle.yaw = 0.0;
                target.roll = angle.roll;
                target.pitch = angle.pitch;
                target.yaw = angle.yaw;
                do_attitude_control_task(controller, target);
            }

            // 发送数据更新并上发
            auto gyro_att = gyro.getAttitude();
            Attitude att{gyro_att.x,gyro_att.y,gyro_att.z};
            auto gyro_av = gyro.getAngularVelocity();
            AngularVel av{gyro_av.x, gyro_av.y, gyro_av.z};

            flag_balancing = balancer.getIfFinishBalancing();
            set_balancing = balancer.getIfInBalancing();

            send_flag++;
            if (send_flag >= 20) {
                // 数据上发
                for (int i = 0; i < 1; i++) {
                    Plane data{};
                    data.head.head = 0x4D47;
                    data.tail.checksum = 0x00;
                    data.data.device_id = 0x02;
                    data.data.platform_type = 0xF2;
                    data.data.cmd_count = 0x01;
                    data.data.file_count = 0x01;
                    data.data.platform_status = 0x01;

                    data.data.gyro_fault = 0x00;
                    data.data.wx = av.wx;
                    data.data.wy = av.wy;
                    data.data.wz = av.wz;
                    data.data.roll = att.roll;
                    data.data.pitch = att.pitch;
                    data.data.yaw = att.yaw;

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

                    data.data.thrust_x = 0;
                    data.data.thrust_y = 0;
                    data.data.thrust_z = 0;
                    data.data.torque_roll = 300;
                    data.data.torque_pitch = 300;
                    data.data.torque_yaw = 300;

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

                // std::cout << "Publish Plane: " << sizeof(data) << std::endl;
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
    client.stop_consuming();
    client.disconnect()->wait();
    std::cout << "[MQTT] 断开连接" << std::endl;

    return 0;
}
