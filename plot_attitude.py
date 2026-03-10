#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

def plot_attitude(csv_file):
    if not os.path.exists(csv_file):
        print(f"错误: 文件 '{csv_file}' 不存在。")
        return

    # 读取CSV文件
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"读取文件时发生错误: {e}")
        return
    
    # 清理列名中的空格（C++输出的CSV结尾或逗号后可能有空格）
    df.columns = df.columns.str.strip()
    
    # 期望的列名
    required_columns = ['ms', 'gyro_roll', 'gyro_pitch', 'gyro_yaw', 
                        'mocap_roll', 'mocap_pitch', 'mocap_yaw', 
                        'new_roll', 'new_pitch', 'new_yaw']
    
    for col in required_columns:
        if col not in df.columns:
            print(f"错误: CSV文件中缺少列 '{col}'。请检查CSV文件的表头。")
            print(f"当前有的列名: {list(df.columns)}")
            return

    time_ms = df['ms']

    def plot_sensor_rpy(sensor_name, roll_col, pitch_col, yaw_col, filename):
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig.suptitle(f'{sensor_name} RPY Data')

        axes[0].plot(time_ms, df[roll_col], label='Roll', color='r', linewidth=1.5)
        axes[0].set_ylabel('Roll Angle')
        axes[0].grid(True)
        axes[0].legend(loc='upper right')

        axes[1].plot(time_ms, df[pitch_col], label='Pitch', color='g', linewidth=1.5)
        axes[1].set_ylabel('Pitch Angle')
        axes[1].grid(True)
        axes[1].legend(loc='upper right')

        axes[2].plot(time_ms, df[yaw_col], label='Yaw', color='b', linewidth=1.5)
        axes[2].set_ylabel('Yaw Angle')
        axes[2].set_xlabel('Time (ms)')
        axes[2].grid(True)
        axes[2].legend(loc='upper right')

        plt.tight_layout()
        plt.savefig(filename, dpi=300)

    # ==============================
    # 1. 绘制 Gyro 传感器的 RPY 数据
    # ==============================
    plot_sensor_rpy('Gyro Sensor', 'gyro_roll', 'gyro_pitch', 'gyro_yaw', 'gyro_rpy.png')
    
    # ==============================
    # 2. 绘制 Mocap 传感器的 RPY 数据
    # ==============================
    plot_sensor_rpy('Mocap Sensor', 'mocap_roll', 'mocap_pitch', 'mocap_yaw', 'mocap_rpy.png')

    # ==============================
    # 3. 绘制 New (算法/计算) 的 RPY 数据
    # ==============================
    plot_sensor_rpy('New Algorithm', 'new_roll', 'new_pitch', 'new_yaw', 'new_rpy.png')

    print("绘图完成！已保存为当前目录下的:")
    print(" - gyro_rpy.png")
    print(" - mocap_rpy.png")
    print(" - new_rpy.png")
    
    # 弹出窗口显示图片
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("用法: python plot_attitude.py <csv文件路径>")
        print("示例: python plot_attitude.py attitude_compare.csv")
        sys.exit(1)
        
    csv_filename = sys.argv[1]
    plot_attitude(csv_filename)
