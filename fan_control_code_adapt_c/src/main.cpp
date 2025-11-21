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

msmserial::MsMSerial msm_serial("COM7", 115200);  //串口

GyroScope gyro(&msm_serial);
LeadScrewController leadscrew(&msm_serial);
Wheel wheel(&msm_serial);
Fan fan(&msm_serial);

/* ---------------- 线程任务 ---------------- */
void do_balance_task(MassCenterBalancer& balancer)
{
    std::cout << "执行自动调平算法...\n";
    std::vector<float> action{-100000};
    leadscrew().move_to(action);   // 先降 Z 轴质量块
    balancer.balance_both_axes_fan(); // XY轴调平
    balancer.balance_z_axes_fan(); // Z轴调平
    std::cout << "调平完成" << std::endl;
}

void do_attitude_control_task(AttitudePDController& controller,
                              const Attitude& target)
{
    std::cout << "执行姿态控制算法" << std::endl;
    Vec3 euler_angle_target(target.roll, target.pitch, target.yaw);
    controller.setAttitudeInBalancing(euler_angle_target);
    std::cout << "姿态控制完成\n";
}

/* ---------------- main ---------------- */
int main()
{
    AttitudePDController controller(wheel, gyro, fan);
    MassCenterBalancer balancer(leadscrew, gyro, wheel, controller);

    MQTTServer mqtt("192.168.31.81", 1883, "1",
                    "satellite/data",
                    "satellite/cmd");
    mqtt.start();

    try
    {
        while (true)
        {
            /* 1. 调平指令 */
            if (mqtt.flag_balance.exchange(false))
            {
                std::thread([&balancer]{
                    do_balance_task(balancer);
                }).detach();
            }

            /* 2. 欧拉角姿态指令 */
            if (mqtt.flag_attitude_euler.exchange(false))
            {
                std::thread([&controller, &mqtt]{
                    do_attitude_control_task(controller,
                                             mqtt.attitude_data_euler);
                }).detach();
            }

            /* 3. 四元数姿态指令 */
            if (mqtt.flag_attitude_quat.exchange(false))
            {
                std::thread([&controller, &mqtt]{
                    auto q = mqtt.attitude_data_quat;
                    // quat -> euler ZYX
                    Vec3 ea = q.toRotationMatrix().eulerAngles(2,1,0);
                    Attitude att{ea[2], ea[1], ea[0]};
                    do_attitude_control_task(controller, att);
                }).detach();
            }

            /* 4. 主循环更新传感器数据 */
            Attitude  att  = gyro.getAttitude();
            AngularVel av  = gyro.getAngularVelocity();
            Eigen::Quaterniond q =
                Eigen::AngleAxisd(att.yaw,   Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(att.pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(att.roll,  Eigen::Vector3d::UnitX());

            double t = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            mqtt.publish_telemetry(t, att, av, q);

            wheel.setCurrent(att, av);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "异常退出: " << e.what() << '\n';
    }

    return 0;
}