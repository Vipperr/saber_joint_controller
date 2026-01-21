#ifndef DISTURBANCE_OBSERVER_HPP
#define DISTURBANCE_OBSERVER_HPP
#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include "saber_get_regression.hpp"
#include "OpenXLSX/OpenXLSX.hpp"

enum class Update_Type {
    dynamics,  // 线性化动力学模型计算
    statics,  // 静力学模型近似计算
};

enum class Observer_Type {
    traditional,  // 传统扰动观测器
    kalman_filter // 卡尔曼滤波扰动观测器
};

class DisturbanceObserver {
public:
    /**
     * 传统扰动观测器构造函数
     * @param n_joints 关节数量
     * @param K 增益矩阵（对角矩阵）
     * @param fs 采样频率
     */
    DisturbanceObserver(int n_joints, const Eigen::MatrixXd& K, double fs, const rclcpp::Logger& logger);

    /**
     * 卡尔曼滤波观测器专用构造函数
     */
    DisturbanceObserver(int n_joints, double fs, const rclcpp::Logger& logger,
                       const Eigen::MatrixXd& A_tau = Eigen::MatrixXd(),
                       const Eigen::MatrixXd& Q_p = Eigen::MatrixXd(),
                       const Eigen::MatrixXd& Q_tau = Eigen::MatrixXd(),
                       const Eigen::MatrixXd& R_c = Eigen::MatrixXd());
    
    /**
     * 更新扰动观测器
     * @param q 关节位置向量
     * @param qd 关节速度向量
     * @param tau 控制力矩向量
     * @param M 惯性矩阵计算函数
     * @param C 科里奥利矩阵计算函数
     * @param G 重力向量计算函数
     * @param Ff 摩擦力向量计算函数
     * @return 外部扰动估计值
     */

    

    // 数值差分方法计算M_dot
    Eigen::VectorXd update(const Eigen::VectorXd& q, 
                          const Eigen::VectorXd& qd,
                          const Eigen::VectorXd& tau,
                          Update_Type update_type = Update_Type::dynamics);
    
   
    /**
     * 重置观测器状态
     */
    void reset();
    
    /**
     * 获取当前扰动估计
     */
    Eigen::VectorXd getDisturbanceEstimate() const { 
        return (observer_type_ == Observer_Type::kalman_filter) ? 
               tau_hat_ext_kf_ : tau_hat_ext_; 
    }

    /**
     * 卡尔曼滤波参数配置
     */
    void setKalmanFilterParameters(const Eigen::MatrixXd& A_tau,
                                  const Eigen::MatrixXd& Q_p,
                                  const Eigen::MatrixXd& Q_tau,
                                  const Eigen::MatrixXd& R_c);

private:
    int n_joints_;                          // 关节数量
    Eigen::MatrixXd K_;                     // 增益矩阵
    double fs_;                             // 采样频率
    double dt_;                             // 采样时间
    Observer_Type observer_type_;           // 观测器类型

    // MDH 参数
    std::vector<double> alpha = {0.0, -M_PI/2.0, 0.0, 0.0, -M_PI/2.0, -M_PI/2.0};
    std::vector<double> a = {0.0, 0.0, 0.11141, 0.11141, 0.0, 0.0};
    std::vector<double> d = {0.195, 0.0, 0.0, -0.0085, 0.1185, 0.2485};
    
    // 传统扰动观测器状态变量
    Eigen::VectorXd tau_hat_ext_;           // 外部扰动估计 τ̂_ext
    Eigen::VectorXd P_prev_;                // 上一时刻广义动量 P[t-1]
    Eigen::VectorXd U_prev_;                // 上一时刻U值 U[t-1]
    Eigen::VectorXd integral_sum_;          // 积分累加和
    Eigen::VectorXd P0_;                    // 初始广义动量
    bool is_initialized_;                   // 是否已初始化

    // 卡尔曼滤波观测器参数
    Eigen::MatrixXd A_tau_;      // 外力变化趋势矩阵
    Eigen::MatrixXd Q_p_;        // 过程噪声协方差 - 动量
    Eigen::MatrixXd Q_tau_;      // 过程噪声协方差 - 外力
    Eigen::MatrixXd R_c_;        // 测量噪声协方差
    
    // 卡尔曼滤波系统矩阵
    Eigen::MatrixXd A_c_;        // 连续时间状态转移矩阵
    Eigen::MatrixXd B_c_;        // 连续时间控制输入矩阵
    Eigen::MatrixXd C_c_;        // 输出矩阵
    Eigen::MatrixXd A_d_;        // 离散状态转移矩阵
    Eigen::MatrixXd B_d_;        // 离散控制输入矩阵
    Eigen::MatrixXd C_d_;        // 离散输出矩阵
    Eigen::MatrixXd Q_kf_;          // 过程噪声协方差
    Eigen::MatrixXd P_kf_;       // 卡尔曼滤波误差协方差
    
    // 状态变量
    Eigen::VectorXd tau_hat_ext_kf_;        // 卡尔曼滤波扰动估计
    Eigen::VectorXd x_hat_;                 // 卡尔曼滤波状态估计 [p; tau_ext]


    // 动力学模型参数
    Eigen::VectorXd min_param_set_ = Eigen::VectorXd::Zero(36);
    Eigen::VectorXd min_param_set = Eigen::VectorXd::Zero(48);

    // 静力学模型参数
    Eigen::VectorXd min_param_set_statics_ = Eigen::VectorXd::Zero(10);
    Eigen::VectorXd min_param_set_statics = Eigen::VectorXd::Zero(16);
    Eigen::MatrixXd E_ = Eigen::MatrixXd::Zero(24, 24);
    Eigen::Matrix4d Q_;
    Eigen::RowVector4d g_T;

    // 私有成员，用于存储日志记录器
    rclcpp::Logger logger_; 

    /**
     * 计算广义动量 p = M(q) * qd
     */
    Eigen::VectorXd computeGeneralizedMomentum(const Eigen::VectorXd& q, 
                                              const Eigen::VectorXd& qd);
    
    /**
     * 计算U项：U = C^T * qd - G - Ff + τ + τ̂_ext
     */
    Eigen::VectorXd compute_UT_term(const Eigen::VectorXd& q,
                                const Eigen::VectorXd& qd,
                                const Eigen::VectorXd& tau,
                                Update_Type update_type = Update_Type::dynamics);

    Eigen::MatrixXd compute_mass_matrix(const Eigen::VectorXd& q);
    Eigen::VectorXd compute_coriolis_term(const Eigen::VectorXd& q, const Eigen::VectorXd& qd);
    Eigen::VectorXd compute_massdot_qdot(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, double delta_q = 1e-7);
    Eigen::VectorXd compute_massdot_qdot_improved(const Eigen::VectorXd& q, const Eigen::VectorXd& qd);
    Eigen::VectorXd compute_gravity_term(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, Update_Type update_type);
    Eigen::VectorXd compute_friction_term(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau);
    Eigen::Matrix3d R_mdh(double alpha, double theta);
    Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d& v);
    Eigen::Matrix4d T_mdh(double alpha, double a, double theta, double d);
    Eigen::MatrixXd get_observation(const std::array<double, 6>& q, const std::array<double, 6>& qd);

    // 卡尔曼滤波器私有函数
    void initializeKalmanFilter();
    void discreteTimeSystem();
    void kalmanFilterPredict(const Eigen::VectorXd& mu);
    void kalmanFilterUpdate(const Eigen::VectorXd& p_measure);
    Eigen::VectorXd computeMu(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau, Update_Type update_type);
};


#endif