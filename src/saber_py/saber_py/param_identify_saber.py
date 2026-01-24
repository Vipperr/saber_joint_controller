import matplotlib
matplotlib.use('Agg')
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from saber_get_regression import saber_get_regression

# 读取数据
file_path = "/home/xiatenghui/saber_joint_controller/src/saber_py/"
positions = pd.read_excel(file_path + "saber_positions.xlsx")
velocities = pd.read_excel(file_path + "saber_velocities.xlsx")
accelerations = pd.read_excel(file_path + "saber_accelerations.xlsx")
efforts = pd.read_excel(file_path + "saber_efforts.xlsx")
times = positions.iloc[:, 0].to_numpy()

# 预处理数据
q_log = [[float(positions[f'position_{i+1}'][j]) for i in range(6)] for j in range(len(positions['position_1']))]
dq_log = [[float(velocities[f'velocity_{i+1}'][j]) for i in range(6)] for j in range(len(velocities['velocity_1']))]
ddq_log = [[float(accelerations[f'acceleration_{i+1}'][j]) for i in range(6)] for j in range(len(accelerations['acceleration_1']))]
tau_log = [[float(efforts[f'effort_{i+1}'][j]) for i in range(6)] for j in range(len(efforts['effort_1']))]

# 转换为NumPy数组便于处理
dq_array = np.array(dq_log)
ddq_array = np.array(ddq_log)

def butter_lowpass_filter(data, cutoff_freq, fs, order=4, axis=0):
    """
    应用Butterworth低通滤波器
    :param data: 输入数据
    :param cutoff_freq: 截止频率(Hz)
    :param fs: 采样频率(Hz)
    :param order: 滤波器阶数
    :return: 滤波后的数据
    """
    nyquist_freq = 0.5 * fs  # 奈奎斯特频率
    normal_cutoff = cutoff_freq / nyquist_freq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data, axis=axis)  # 使用filtfilt实现零相位滤波
    return y

def plot_separate_comparisons(times, dq_array, ddq_array, ddq_filtered, cutoff_frequency=1.0):
    colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown']
    linestyles = ['-', '--', '-.', ':', '-', '--']
    joint_labels = [f'Joint {i+1}' for i in range(6)]
    
    # Create a figure with 3 vertical subplots
    fig, axs = plt.subplots(3, 1, figsize=(12, 12))
    
    # Plot 1: Velocity
    for i in range(6):
        axs[0].plot(times, dq_array[:, i], 
                   color=colors[i], 
                   linestyle=linestyles[i],
                   linewidth=1.2, 
                   label=joint_labels[i])
    axs[0].set_title('Joint Velocity Curves', fontsize=14, fontweight='bold')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Velocity (rad/s)')
    axs[0].legend()
    axs[0].grid(True, alpha=0.3)
    
    # Plot 2: Raw Acceleration
    for i in range(6):
        axs[1].plot(times, ddq_array[:, i], 
                   color=colors[i], 
                   linestyle=linestyles[i],
                   linewidth=1.2, 
                   label=joint_labels[i])
    axs[1].set_title('Raw Joint Acceleration Curves', fontsize=14, fontweight='bold')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Acceleration (rad/s²)')
    axs[1].legend()
    axs[1].grid(True, alpha=0.3)
    
    # Plot 3: Filtered Acceleration
    for i in range(6):
        axs[2].plot(times, ddq_filtered[:, i], 
                   color=colors[i], 
                   linestyle=linestyles[i],
                   linewidth=1.5, 
                   label=joint_labels[i])
    axs[2].set_title(f'Filtered Joint Acceleration Curves (Cutoff Frequency: {cutoff_frequency}Hz)', 
                    fontsize=14, fontweight='bold')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Acceleration (rad/s²)')
    axs[2].legend()
    axs[2].grid(True, alpha=0.3)
    
    # Adjust layout to prevent overlap
    plt.tight_layout()
    plt.savefig('/home/xiatenghui/saber_joint_controller/src/saber_py/identification_result.png') # 保存到你指定的位置
    print("图片已保存为 /home/xiatenghui/saber_joint_controller/src/saber_py/identification_result.png")

# 滤波器参数设置
sampling_freq = 100  # 采样频率为100Hz
cutoff_frequency = 1.0  # 截止频率设置
ddq_filtered = butter_lowpass_filter(ddq_array, cutoff_frequency, sampling_freq)
plot_separate_comparisons(times, dq_array, ddq_array, ddq_filtered, cutoff_frequency)

## Construct the regressor matrix Y and the torque vector tau

Y_total = []
tau_total = []

for q, dq, ddq, tau in zip(q_log, dq_log, ddq_filtered, tau_log):
    Y_r = np.array(saber_get_regression(q, dq, ddq))  # Y: dof x n_baseparms
    Y_r = Y_r.reshape(6, -1)
    Y_c = np.diag(np.sign(dq))
    Y_v = np.diag(dq)
    Y = np.hstack((Y_r, Y_c, Y_v))
    Y_total.append(Y)
    tau_total.append(np.array(tau))

Y_total = np.vstack(Y_total)      # shape: (N*dof, n_baseparms)
tau_total = np.hstack(tau_total)  # shape: (N*dof,)

cond = np.linalg.cond(Y_total)
print("condition : ",cond)
print("Regressor matrix and torque vector constructed.")
## Regression to find base parameters based on least squares method. theta_hat is the estimated rbt.dyn.baseparms
theta_hat, residuals, rank, s = np.linalg.lstsq(Y_total, tau_total, rcond=None)
print("Least squares regression complete.")
ur5e_min_param = pd.DataFrame(theta_hat)
ur5e_min_param.to_excel(file_path + "saber_min_param.xlsx", index=None, header=False)

## Optional: compare the MSE error
tau_pred = Y_total @ theta_hat
error = tau_total - tau_pred
mse = np.mean(error**2)
print('MSE error:', mse)