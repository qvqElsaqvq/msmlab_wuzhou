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

    # ==============================
    # 1. 绘制 Gyro 传感器的 RPY 数据
    # ==============================
    plt.figure("Gyro Attitude", figsize=(10, 6))
    plt.plot(time_ms, df['gyro_roll'], label='Roll', linewidth=1.5)
    plt.plot(time_ms, df['gyro_pitch'], label='Pitch', linewidth=1.5)
    plt.plot(time_ms, df['gyro_yaw'], label='Yaw', linewidth=1.5)
    plt.xlabel('Time (ms)')
    plt.ylabel('Angle')
    plt.title('Gyro Sensor RPY Data')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('gyro_rpy.png', dpi=300)
    
    # ==============================
    # 2. 绘制 Mocap 传感器的 RPY 数据
    # ==============================
    plt.figure("Mocap Attitude", figsize=(10, 6))
    plt.plot(time_ms, df['mocap_roll'], label='Roll', linewidth=1.5)
    plt.plot(time_ms, df['mocap_pitch'], label='Pitch', linewidth=1.5)
    plt.plot(time_ms, df['mocap_yaw'], label='Yaw', linewidth=1.5)
    plt.xlabel('Time (ms)')
    plt.ylabel('Angle')
    plt.title('Mocap Sensor RPY Data')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('mocap_rpy.png', dpi=300)

    # ==============================
    # 3. 绘制 New (算法/计算) 的 RPY 数据
    # ==============================
    plt.figure("New Attitude", figsize=(10, 6))
    plt.plot(time_ms, df['new_roll'], label='Roll', linewidth=1.5)
    plt.plot(time_ms, df['new_pitch'], label='Pitch', linewidth=1.5)
    plt.plot(time_ms, df['new_yaw'], label='Yaw', linewidth=1.5)
    plt.xlabel('Time (ms)')
    plt.ylabel('Angle')
    plt.title('New Algorithm RPY Data')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('new_rpy.png', dpi=300)

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
