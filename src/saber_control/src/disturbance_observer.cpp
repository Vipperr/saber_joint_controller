#include "disturbance_observer.hpp"
#include <iostream>

using namespace OpenXLSX; 

DisturbanceObserver::DisturbanceObserver(int n_joints, const Eigen::MatrixXd& K, double fs, const rclcpp::Logger& logger)
    : n_joints_(n_joints), K_(K), fs_(fs), is_initialized_(false), logger_(logger) {
    
    if (n_joints <= 0) {
        throw std::invalid_argument("关节数量必须大于0");
    }

    if (fs <= 0) {
        throw std::invalid_argument("采样频率必须大于0");
    }
    
    dt_ = 1.0 / fs_;
    observer_type_ = Observer_Type::traditional;
    
    // 初始化状态变量
    tau_hat_ext_ = Eigen::VectorXd::Zero(n_joints_);
    P_prev_ = Eigen::VectorXd::Zero(n_joints_);
    U_prev_ = Eigen::VectorXd::Zero(n_joints_);
    integral_sum_ = Eigen::VectorXd::Zero(n_joints_);
    P0_ = Eigen::VectorXd::Zero(n_joints_);

    // 初始化动力学参数
    XLDocument PA;
    PA.open("/home/xiatenghui/work_space/mujoco_ws/src/mujoco_py/saber_min_param.xlsx");

    XLWorksheet PA_sheet = PA.workbook().worksheet("Sheet1");

    int PA_Row = PA_sheet.rowCount();
    for(int row = 1;row <= PA_Row;row++)
    {
        auto cell = PA_sheet.cell(row, 1);
        auto& value = cell.value();
        if (value.type() == OpenXLSX::XLValueType::Empty) continue;

        // 处理数值
        if (value.type() == OpenXLSX::XLValueType::Float || 
            value.type() == OpenXLSX::XLValueType::Integer) {
            double num = value.get<double>();
            min_param_set[row-1] = num;
        }                 
    }
    min_param_set_ = min_param_set.head(36);

    // 初始化静力学参数
    XLDocument PA_statics;
    PA_statics.open("/home/xiatenghui/work_space/mujoco_ws/src/mujoco_py/saber_min_param_statics.xlsx");

    XLWorksheet PA_sheet_statics = PA_statics.workbook().worksheet("Sheet1");

    int PA_Row_statics = PA_sheet_statics.rowCount();
    for(int row = 1;row <= PA_Row_statics;row++)
    {
        auto cell = PA_sheet_statics.cell(row, 1);
        auto& value = cell.value();
        if (value.type() == OpenXLSX::XLValueType::Empty) continue;

        // 处理数值
        if (value.type() == OpenXLSX::XLValueType::Float || 
            value.type() == OpenXLSX::XLValueType::Integer) {
            double num = value.get<double>();
            min_param_set_statics[row-1] = num;
        }                 
    }
    min_param_set_statics_ = min_param_set_statics.head(10);

    Q_ << 0.0, -1.0, 0.0, 0.0,
            1.0,  0.0, 0.0, 0.0,
            0.0,  0.0, 0.0, 0.0,
            0.0,  0.0, 0.0, 0.0;
    g_T << 0, 0, 1, 0;

    XLDocument E;
    E.open("/home/xiatenghui/work_space/mujoco_ws/src/mujoco_sim/src/E_saber.xlsx");
    XLWorksheet E_sheet = E.workbook().worksheet("Sheet1");
    int E_Row = E_sheet.rowCount();
    int E_Col = E_sheet.columnCount();
    for(int row = 1;row <= E_Row;row++)
    {
        for(int col = 1;col <= E_Col;col++)
        {
            auto cell = E_sheet.cell(row, col);
            auto& value = cell.value();
            if (value.type() == OpenXLSX::XLValueType::Empty) continue;

            // 处理数值
            if (value.type() == OpenXLSX::XLValueType::Float || 
                value.type() == OpenXLSX::XLValueType::Integer) {
                double num = value.get<double>();
                E_(row-1, col-1) = num;
            }
        }
    }

}

// 数值差分方法计算M_dot,外力估计误差在1Nm左右
Eigen::VectorXd DisturbanceObserver::update(
    const Eigen::VectorXd& q, 
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& tau,
    Update_Type update_type)
{
    
    // 检查输入维度
    if (q.size() != n_joints_ || qd.size() != n_joints_ || tau.size() != n_joints_) {
        throw std::invalid_argument("输入向量维度不匹配");
    }
    
    if(observer_type_==Observer_Type::traditional)
    {
        // 计算当前广义动量 P[t] = M(q) * qd
        Eigen::VectorXd P_current = computeGeneralizedMomentum(q, qd);
        
        // 第一次调用时进行初始化
        if (!is_initialized_)
        {
            P0_ = P_current;
            P_prev_ = P_current;
            U_prev_ = Eigen::VectorXd::Zero(n_joints_);
            integral_sum_.setZero();
            is_initialized_ = true;
            return tau_hat_ext_;
        }
        
        // 计算当前U值：U[t] = C^T * qd - G - Ff + τ + τ̂_ext
        Eigen::VectorXd U_current;
        switch (update_type)
        {
            case Update_Type::dynamics:
                U_current = compute_UT_term(q, qd, tau, update_type);
                break;
            case Update_Type::statics:
                U_current = compute_UT_term(q, qd, tau, update_type);
                break;
            default:
                break;
        }
        
        
        // 使用梯形公式计算积分增量并累加
        Eigen::VectorXd integral_increment = (U_prev_ + U_current) / (2.0 * fs_);
        integral_sum_ += integral_increment;
        
        // 计算扰动估计：τ̂_ext[t] = K * (P[t] - P0 - integral_sum)
        tau_hat_ext_ = K_ * (P_current - P0_ - integral_sum_);
        
        // 更新历史数据
        P_prev_ = P_current;
        U_prev_ = U_current;
        
        return tau_hat_ext_;
    }
    else if(observer_type_==Observer_Type::kalman_filter)
    {
        // 卡尔曼滤波观测器
        Eigen::VectorXd p_measure = computeGeneralizedMomentum(q, qd);
        Eigen::VectorXd mu = computeMu(q, qd, tau, update_type);
        
        kalmanFilterPredict(mu);
        kalmanFilterUpdate(p_measure);
        
        return tau_hat_ext_kf_;
    }
}


Eigen::VectorXd DisturbanceObserver::computeGeneralizedMomentum(
    const Eigen::VectorXd& q, 
    const Eigen::VectorXd& qd)
{
    
    Eigen::MatrixXd M_matrix = compute_mass_matrix(q);
    return M_matrix * qd;
}

Eigen::VectorXd DisturbanceObserver::compute_UT_term(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& tau,
    Update_Type update_type)
{
    
    Eigen::MatrixXd CT_q = compute_massdot_qdot(q, qd) - compute_coriolis_term(q, qd);
    Eigen::VectorXd G_vec = compute_gravity_term(q, qd, update_type);
    Eigen::VectorXd Ff_vec = compute_friction_term(q, qd, tau);
    
    // U = C^T * qd - G - Ff + τ + τ̂_ext
    return CT_q - G_vec - Ff_vec + tau + tau_hat_ext_;
}

void DisturbanceObserver::reset() {
    if(observer_type_ == Observer_Type::traditional)
    {
        tau_hat_ext_.setZero();
        P_prev_.setZero();
        U_prev_.setZero();
        integral_sum_.setZero();
        P0_.setZero();
        is_initialized_ = false;
    }
    else if(observer_type_ == Observer_Type::kalman_filter)
    {
        int state_dim = 2 * n_joints_;
        x_hat_.setZero();
        P_kf_ = Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.1;
    }

}

Eigen::MatrixXd DisturbanceObserver::compute_mass_matrix(const Eigen::VectorXd& q)
{
    std::array<double, 6> positions{};
    std::copy(q.data(), q.data() + 6, positions.begin());
    std::array<double, 6> velocities{};
    Eigen::VectorXd result = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd mass_matrix = Eigen::MatrixXd::Identity(6, 6);
    double regression_matrix[216];
    std::array<double, 6> v;

    for(int i = 0;i < 6;i++)
    {
        v.fill(0);
        v[i] = 1;
        saber_get_regression(regression_matrix, positions, velocities, v, 0);
        Eigen::Map<Eigen::Matrix<double, 6, 36, Eigen::RowMajor>> Regression_Matrix(regression_matrix);
        result = Regression_Matrix * min_param_set_;
        mass_matrix.col(i) = result;
    }

    // RCLCPP_INFO_STREAM(logger_, "6x6 Mass Matrix:");
    // RCLCPP_INFO_STREAM(logger_, "\n" << mass_matrix.format(Eigen::IOFormat(4, 0, ", ", "\n", "[", "]")));

    return mass_matrix;

}

Eigen::VectorXd DisturbanceObserver::compute_coriolis_term(const Eigen::VectorXd& q, const Eigen::VectorXd& qd)
{
    std::array<double, 6> positions{};
    std::copy(q.data(), q.data() + 6, positions.begin());
    std::array<double, 6> velocities{};
    std::copy(qd.data(), qd.data() + 6, velocities.begin());
    std::array<double, 6> accelerations{};

    Eigen::VectorXd coriolis_term = Eigen::VectorXd::Zero(6);
    double regression_matrix[216];
    saber_get_regression(regression_matrix, positions, velocities, accelerations, 0);
    Eigen::Map<Eigen::Matrix<double, 6, 36, Eigen::RowMajor>> Regression_Matrix(regression_matrix);
    coriolis_term = Regression_Matrix * min_param_set_;

    // RCLCPP_INFO_STREAM(logger_, "6x1 coriolis term:");
    // RCLCPP_INFO_STREAM(logger_, "\n" << coriolis_term.transpose().format(Eigen::IOFormat(4, 0, ", ", "\n", "[", "]")));

    return coriolis_term;
}


Eigen::VectorXd DisturbanceObserver::compute_massdot_qdot(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, double delta_q)
{
    int n = q.size();
    
    // 检查输入有效性
    if (q.size() != qd.size()) 
    {
        throw std::invalid_argument("q and qd must have the same dimension");
    }
    
    if (delta_q <= 0) 
    {
        throw std::invalid_argument("delta_q must be positive");
    }

    // 建立映射关系
    double regression_matrix[216];
    Eigen::Map<Eigen::Matrix<double, 6, 36, Eigen::RowMajor>> Regression_Matrix(regression_matrix);
    Eigen::VectorXd result = Eigen::VectorXd::Zero(n);
    std::array<double, 6> positions{};
    std::copy(q.data(), q.data() + 6, positions.begin());
    std::array<double, 6> velocities{};
    std::array<double, 6> accelerations{};
    std::copy(qd.data(), qd.data() + 6, accelerations.begin());
    for (int i = 0; i < n; ++i) 
    {
        std::array<double, 6> positions_perturbed = positions; 
        // 正向扰动
        positions_perturbed[i] += delta_q;
        saber_get_regression(regression_matrix, positions_perturbed, velocities, accelerations, 0);
        Eigen::VectorXd tau_plus = Regression_Matrix * min_param_set_;

        // 反向扰动
        positions_perturbed[i] -= 2 * delta_q;
        saber_get_regression(regression_matrix, positions_perturbed, velocities, accelerations, 0);
        Eigen::VectorXd tau_minus = Regression_Matrix * min_param_set_;

        // 中心差分近似（精度更高）
        Eigen::VectorXd derivative = (tau_plus - tau_minus) / (2 * delta_q);

        result += derivative * qd(i);
    }

    // RCLCPP_INFO_STREAM(logger_, "6x1 massdot_qdot:");
    // RCLCPP_INFO_STREAM(logger_, "\n" << result.transpose().format(Eigen::IOFormat(4, 0, ", ", "\n", "[", "]")));

    return result;

}

Eigen::VectorXd DisturbanceObserver::compute_gravity_term(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, Update_Type update_type)
{
    Eigen::VectorXd gravity_term = Eigen::VectorXd::Zero(6);
    if(update_type == Update_Type::dynamics)
    {
        // 建立映射关系
        double regression_matrix[216];
        Eigen::Map<Eigen::Matrix<double, 6, 36, Eigen::RowMajor>> Regression_Matrix(regression_matrix);
        std::array<double, 6> positions{};
        std::copy(q.data(), q.data() + 6, positions.begin());
        std::array<double, 6> velocities{};
        std::array<double, 6> accelerations{};
        saber_get_regression(regression_matrix, positions, velocities, accelerations, 9.81);
        gravity_term = Regression_Matrix * min_param_set_;
    }
    else if(update_type == Update_Type::statics)
    {
        Eigen::MatrixXd observation_matrix = Eigen::MatrixXd::Zero(6, 16);
        std::array<double, 6> positions{};
        std::copy(q.data(), q.data() + 6, positions.begin());
        std::array<double, 6> velocities{};
        std::copy(qd.data(), qd.data() + 6, velocities.begin());
        observation_matrix = get_observation(positions, velocities);
        gravity_term =  observation_matrix * min_param_set_statics;
    }
    
    // RCLCPP_INFO_STREAM(logger_, "6x1 gravity term:");
    // RCLCPP_INFO_STREAM(logger_, "\n" << gravity_term.transpose().format(Eigen::IOFormat(4, 0, ", ", "\n", "[", "]")));

    return gravity_term;
}

Eigen::VectorXd DisturbanceObserver::compute_friction_term(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau)
{
    std::array<double, 6> positions{};
    std::copy(q.data(), q.data() + 6, positions.begin());
    std::array<double, 6> velocities{};
    std::copy(q.data(), q.data() + 6, velocities.begin());

    Eigen::VectorXd friction_term = Eigen::VectorXd::Zero(6);

    // for(int i = 0;i < 6;i++)
    // {
    //     friction_term[i] = velocities[i] * 0.2 + std::copysign(1.0, velocities[i]);
    // }

    // RCLCPP_INFO_STREAM(logger_, "6x1 friction term:");
    // RCLCPP_INFO_STREAM(logger_, "\n" << friction_term.transpose().format(Eigen::IOFormat(4, 0, ", ", "\n", "[", "]")));

    return friction_term;
}

Eigen::Matrix3d DisturbanceObserver::R_mdh(double alpha, double theta)
{
    Eigen::Matrix3d R;
    R << std::cos(theta), -std::sin(theta), 0,
        std::sin(theta)*std::cos(alpha), std::cos(theta)*std::cos(alpha), -std::sin(alpha),
        std::sin(theta)*std::sin(alpha), std::cos(theta)*std::sin(alpha), std::cos(alpha);
    return R;
}

Eigen::Matrix3d DisturbanceObserver::skewSymmetricMatrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d mat;
    mat << 0,    -v(2), v(1),
           v(2), 0,    -v(0),
          -v(1), v(0), 0;
    return mat;
}

Eigen::Matrix4d DisturbanceObserver::T_mdh(double alpha, double a, double theta, double d) 
{
    Eigen::Matrix4d T;
    T << std::cos(theta), -std::sin(theta), 0, a,
        std::sin(theta)*std::cos(alpha), std::cos(theta)*std::cos(alpha), -std::sin(alpha), -d*std::sin(alpha),
        std::sin(theta)*std::sin(alpha), std::cos(theta)*std::sin(alpha), std::cos(alpha), d*std::cos(alpha),
        0, 0, 0, 1;
    return T;
}

Eigen::MatrixXd DisturbanceObserver::get_observation(const std::array<double, 6>& q, const std::array<double, 6>& qd)
{
    // 存储变换矩阵
    std::vector<Eigen::Matrix4d> T_list;
    for (int i = 0; i < 6; ++i) 
    {
        T_list.push_back(T_mdh(alpha[i], a[i], q[i], d[i]));
    }
    
    // 初始化观测矩阵 (6x24)
    Eigen::MatrixXd Y_observation = Eigen::MatrixXd::Zero(6, 24);
    
    // 填充观测矩阵
    for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            Eigen::Matrix4d U = Eigen::Matrix4d::Identity();
            
            // 累加 i 之前的所有变换
            for (int k = 0; k <= i; ++k) {
                U = U * T_list[k];
            }
            
            // 乘以 Q 矩阵
            U = U * Q_;
            
            // 累加 i+1 到 j 的变换
            for (int m = i+1; m <= j; ++m) {
                U = U * T_list[m];
            }
            
            // 计算 g_T * U
            Eigen::RowVector4d D = g_T * U;
            
            // 将结果放入对应的列位置 (4*j 到 4*j+3)
            Y_observation.block(i, 4*j, 1, 4) = D;
        }
    }
    
    // 计算 Y_g = Y_observation * E 的前10列 (6x10)
    Eigen::MatrixXd Y_g = Y_observation * E_.block(0, 0, E_.rows(), 10);

    // 计算 Y_f：对角矩阵，符号与 qd 相同 (6x6)
    Eigen::MatrixXd Y_f = Eigen::MatrixXd::Zero(6, 6);
    for(int i = 0;i < 6;i++)
    {
        if(qd[i] > 0)
        {
            Y_f(i, i) = 1.0;
        }
        else
        {
            Y_f(i, i) = -1.0;
        }
    }
    
    // 合并 Y_g 和 Y_f (6x16)
    Eigen::MatrixXd Y(6, 16);
    Y << Y_g, Y_f;
    
    return Y;
}

DisturbanceObserver::DisturbanceObserver(int n_joints, double fs, const rclcpp::Logger& logger,
                                       const Eigen::MatrixXd& A_tau, const Eigen::MatrixXd& Q_p,
                                       const Eigen::MatrixXd& Q_tau, const Eigen::MatrixXd& R_c)
    : n_joints_(n_joints), fs_(fs), dt_(1.0/fs), observer_type_(Observer_Type::kalman_filter),
      logger_(logger), A_tau_(A_tau), Q_p_(Q_p), Q_tau_(Q_tau), R_c_(R_c), is_initialized_(false) {
    
    if (n_joints <= 0) throw std::invalid_argument("关节数量必须大于0");
    if (fs <= 0) throw std::invalid_argument("采样频率必须大于0");

    observer_type_ = Observer_Type::kalman_filter;

    // 初始化状态变量
    tau_hat_ext_kf_ = Eigen::VectorXd::Zero(n_joints_);
    
    initializeKalmanFilter();
}

// 初始化卡尔曼滤波器
void DisturbanceObserver::initializeKalmanFilter() {
    int state_dim = 2 * n_joints_;
    
    // 设置默认参数
    if (A_tau_.size() == 0) {
        A_tau_ = Eigen::MatrixXd::Zero(n_joints_, n_joints_);
    }
    if (Q_p_.size() == 0) {
        Q_p_ = Eigen::MatrixXd::Identity(n_joints_, n_joints_) * 0.01;
    }
    if (Q_tau_.size() == 0) {
        Q_tau_ = Eigen::MatrixXd::Identity(n_joints_, n_joints_) * 0.001;
    }
    if (R_c_.size() == 0) {
        R_c_ = Eigen::MatrixXd::Identity(n_joints_, n_joints_) * 0.1;
    }
    
    // 构建连续时间系统矩阵 (公式3-41)
    A_c_ = Eigen::MatrixXd::Zero(state_dim, state_dim);
    A_c_.block(0, n_joints_, n_joints_, n_joints_) = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
    A_c_.block(n_joints_, n_joints_, n_joints_, n_joints_) = A_tau_;
    
    B_c_ = Eigen::MatrixXd::Zero(state_dim, n_joints_);
    B_c_.block(0, 0, n_joints_, n_joints_) = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
    
    C_c_ = Eigen::MatrixXd::Zero(n_joints_, state_dim);
    C_c_.block(0, 0, n_joints_, n_joints_) = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
    
    // 过程噪声协方差 (公式3-42)
    Q_kf_ = Eigen::MatrixXd::Zero(state_dim, state_dim);
    Q_kf_.block(0, 0, n_joints_, n_joints_) = Q_p_;
    Q_kf_.block(n_joints_, n_joints_, n_joints_, n_joints_) = Q_tau_;
    
    // 离散化系统
    discreteTimeSystem();
    
    // 初始化状态估计和协方差
    x_hat_ = Eigen::VectorXd::Zero(state_dim);
    P_kf_ = Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.1;
    tau_hat_ext_kf_ = Eigen::VectorXd::Zero(n_joints_);
}

// 系统离散化
void DisturbanceObserver::discreteTimeSystem() {
    int state_dim = 2 * n_joints_;
    
    // 使用前向欧拉法进行离散化
    A_d_ = Eigen::MatrixXd::Identity(state_dim, state_dim) + A_c_ * dt_;
    B_d_ = B_c_ * dt_;
    C_d_ = C_c_;
}

// 卡尔曼滤波预测步骤
void DisturbanceObserver::kalmanFilterPredict(const Eigen::VectorXd& mu) {
    // 状态预测: x_hat_k|k-1 = A_d * x_hat_k-1|k-1 + B_d * mu
    x_hat_ = A_d_ * x_hat_ + B_d_ * mu;
    
    // 协方差预测: P_k|k-1 = A_d * P_k-1|k-1 * A_d^T + Q
    P_kf_ = A_d_ * P_kf_ * A_d_.transpose() + Q_kf_;
}

// 卡尔曼滤波更新步骤
void DisturbanceObserver::kalmanFilterUpdate(const Eigen::VectorXd& p_measure) {
    int state_dim = 2 * n_joints_;
    
    // 计算卡尔曼增益
    Eigen::MatrixXd K = P_kf_ * C_d_.transpose() * 
                       (C_d_ * P_kf_ * C_d_.transpose() + R_c_).inverse();
    
    // 状态更新
    x_hat_ = x_hat_ + K * (p_measure - C_d_ * x_hat_);
    
    // 协方差更新
    P_kf_ = (Eigen::MatrixXd::Identity(state_dim, state_dim) - K * C_d_) * P_kf_;
    
    // 更新扰动估计
    tau_hat_ext_kf_ = x_hat_.segment(n_joints_, n_joints_);
}

// 计算控制输入mu
Eigen::VectorXd DisturbanceObserver::computeMu(const Eigen::VectorXd& q, 
                                              const Eigen::VectorXd& qd,
                                              const Eigen::VectorXd& tau,
                                              Update_Type update_type) {
    Eigen::MatrixXd CT_q = compute_massdot_qdot(q, qd) - compute_coriolis_term(q, qd);
    Eigen::VectorXd G_vec = compute_gravity_term(q, qd, update_type);
    Eigen::VectorXd Ff_vec = compute_friction_term(q, qd, tau);
    
    // μ = C^T * qd - G - Ff + τ
    return CT_q - G_vec - Ff_vec + tau;
}

// 设置卡尔曼滤波参数
void DisturbanceObserver::setKalmanFilterParameters(const Eigen::MatrixXd& A_tau,
                                                  const Eigen::MatrixXd& Q_p,
                                                  const Eigen::MatrixXd& Q_tau,
                                                  const Eigen::MatrixXd& R_c) {
    A_tau_ = A_tau;
    Q_p_ = Q_p;
    Q_tau_ = Q_tau;
    R_c_ = R_c;
    
    if (observer_type_ == Observer_Type::kalman_filter) {
        initializeKalmanFilter();
    }
    else{
        RCLCPP_INFO_STREAM(logger_, "Observer type error!");
    }
}
