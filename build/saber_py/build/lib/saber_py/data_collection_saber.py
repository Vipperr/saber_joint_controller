#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pandas as pd
import numpy as np

class Data_collection_saber(Node):
    def __init__(self):
        # 初始化节点
        super().__init__('Data_collection_saber')

        # 初始化存储数据的列表
        self.timestamps = []  # 存储时间戳
        self.positions = []  # 存储关节位置
        self.velocities = []  # 存储关节速度
        self.accelerations = [] # 存储关节加速度
        self.efforts = []  # 存储关节力矩
        self.start_time = None
        self.last_velocity = None

        self.subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)

    def listener_callback(self, msg):
        if self.start_time:
            duration_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.start_time
            self.timestamps.append(duration_time)
            print(duration_time)
        else:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            duration_time = 0.0
            self.timestamps.append(duration_time)
        self.positions.append(msg.position)
        self.velocities.append(msg.velocity)
        if self.accelerations:
            acceleration = (np.array(msg.velocity) - self.last_velocity)/0.01
            self.accelerations.append(acceleration)
        else:
            self.accelerations.append(np.zeros_like(msg.position))
        self.last_velocity = np.array(msg.velocity)
        self.efforts.append(msg.effort)
        
    def save_files(self):
        file_path = "/home/xiatenghui/saber_joint_controller/src/saber_py/"
        print(file_path)
        if self.timestamps:
            time = pd.DataFrame(self.timestamps, columns=['time'])

        if self.positions:
            positions = pd.DataFrame(self.positions, columns=[f'position_{i+1}' for i in range(len(self.positions[0]))])
            positions = pd.concat([time, positions], axis=1)
            positions.to_excel(file_path + "saber_positions.xlsx", index=False)
        
        if self.velocities:
            velocities = pd.DataFrame(self.velocities, columns=[f'velocity_{i+1}' for i in range(len(self.velocities[0]))])
            velocities = pd.concat([time, velocities], axis=1)
            velocities.to_excel(file_path + "saber_velocities.xlsx", index=False)
        
        if self.accelerations:
            accelerations = pd.DataFrame(self.accelerations, columns=[f'acceleration_{i+1}' for i in range(len(self.accelerations[0]))])
            accelerations = pd.concat([time, accelerations], axis=1)
            accelerations.to_excel(file_path + "saber_accelerations.xlsx", index=False)
        
        if self.efforts:
            efforts = pd.DataFrame(self.efforts, columns=[f'effort_{i+1}' for i in range(len(self.efforts[0]))])
            efforts = pd.concat([time, efforts], axis=1)
            efforts.to_excel(file_path + "saber_efforts.xlsx", index=False)

def main(args=None):
    rclpy.init(args=args)
    data_collection_saber = Data_collection_saber()
    try:
        rclpy.spin(data_collection_saber)
    except KeyboardInterrupt:
        data_collection_saber.save_files()

if __name__ == '__main__':
    main()
    