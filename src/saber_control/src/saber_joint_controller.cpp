#include <rclcpp/rclcpp.hpp>
#include "OpenXLSX/OpenXLSX.hpp"
#include "disturbance_observer.hpp"
#include "Cybergear_control.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <Eigen/Dense>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <chrono>

using namespace OpenXLSX;
using namespace std;
using namespace Eigen;
using Vector6d = Vector<double, 6>;

class saber_joint_controller : public rclcpp::Node
{
    public:
        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
        using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
    
        saber_joint_controller(const string& name, int sock) : Node(name), sock_(sock)
        {
            // 读取傅里叶系数
            XLDocument fourier_series;
            fourier_series.open("/home/xiatenghui/saber_joint_controller/src/saber_control/src/saber_fourier_series.xlsx");
            XLWorksheet fourier_series_sheet = fourier_series.workbook().worksheet("Sheet1");

            int fourier_series_Row = fourier_series_sheet.rowCount();
            for(int row = 1;row <= fourier_series_Row;row++)
            {
                auto cell = fourier_series_sheet.cell(row, 1);
                auto& value = cell.value();
                if (value.type() == XLValueType::Empty) continue;

                // 处理数值
                if (value.type() == XLValueType::Float || 
                    value.type() == XLValueType::Integer) {
                    double num = value.get<double>();
                    fourier_series_.push_back(num);
                }
            }

            Map<Matrix<double, 11, 6, ColMajor>> fourier_series_data(fourier_series_.data());
            for(int i = 0;i < joint_nums_;i++)
            {
                array<double, 5> A;
                array<double, 5> B;
                for(int j = 0;j < 5;j++)
                {
                    A[j] = fourier_series_data(j, i);
                    B[j] = fourier_series_data(j+5, i);
                }
                A_.push_back(A);
                B_.push_back(B);
                C_[i] = fourier_series_data(10, i);
            }

            // 关节角度初始化
            positions_[1] = -M_PI/2.0;positions_[3] = -M_PI/2.0;
            target_positions_[1] = -M_PI/2.0;target_positions_[3] = -M_PI/2.0;

            // 200Hz的频率发布状态
            timer_ = this->create_wall_timer(5ms,bind(&saber_joint_controller::timer_callback, this));

            // 关节名称初始化
            joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

            // 创建状态发布器
            joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

            // moveit通讯接口初始化
            moveit_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
                                    this,
                                    "/saber_planning_controller/follow_joint_trajectory",
                                    std::bind(&saber_joint_controller::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                                    std::bind(&saber_joint_controller::handle_cancel, this, std::placeholders::_1),
                                    std::bind(&saber_joint_controller::handle_accepted, this, std::placeholders::_1));
        }

        // 析构函数
        ~saber_joint_controller()
        {
            // 标记线程终止
            thread_running_ = false;

            // 等待线程结束，回收资源
            if(rx_thread_.joinable()) 
            {
                rx_thread_.join(); 
            }

            // 关闭sock
            if(sock_ > 0)
            {
                close(sock_);
            }
        }

        void timer_callback() 
        {
            // 封装关节实际状态
            auto joint_state_message = sensor_msgs::msg::JointState();
            joint_state_message.header.stamp = this->now();
            joint_state_message.name = joint_names_;
            joint_state_message.position.resize(6);
            joint_state_message.velocity.resize(6);
            joint_state_message.effort.resize(6);
            for(int i = 0;i < joint_nums_;i++)
            {
                joint_state_message.position[i] = positions_[i];
                joint_state_message.velocity[i] = velocities_[i];
                joint_state_message.effort[i] = torques_[i];
            }
            joint_state_message.position[1] = joint_state_message.position[1] + M_PI/2.0;
            joint_state_message.position[2] = -joint_state_message.position[2];
            joint_state_message.velocity[2] = -joint_state_message.velocity[2];
            joint_state_message.effort[2] = -joint_state_message.effort[2];
            joint_state_message.position[3] = joint_state_message.position[3] + M_PI/2.0;
            joint_state_pub_->publish(joint_state_message);
        }

        bool validate_joint_names(const vector<string>& joint_names) 
        {

            if (joint_names.size() != (joint_names_.size())) 
            {
                RCLCPP_ERROR(get_logger(), "Joint count mismatch: expected %zu, got %zu",
                        joint_names_.size(), joint_names.size());
                return false;
            }
            
            for (size_t i = 0; i < (joint_names_.size()); i++)
            {
                if (joint_names[i] != joint_names_[i]) {
                    RCLCPP_ERROR(get_logger(), "Joint name mismatch at index %zu: expected '%s', got '%s'",
                            i, joint_names_[i].c_str(), joint_names[i].c_str());
                    return false;
                }
            }
            
            return true;
        }

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const FollowJointTrajectory::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received trajectory goal with %zu points", 
                    goal->trajectory.points.size());
            
            // 验证关节名称匹配
            if (!validate_joint_names(goal->trajectory.joint_names)) {
                return rclcpp_action::GoalResponse::REJECT;
            }
            
            // 判断规划的轨迹是否为空
            if (goal->trajectory.points.empty()) 
            {
                RCLCPP_ERROR(this->get_logger(), "Received an empty trajectory");
                return rclcpp_action::GoalResponse::REJECT; // 拒绝目标
            }

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandle> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
        {
            thread{bind(&saber_joint_controller::execute_trajectory_quintic_interpolation, this, goal_handle)}.detach();
        }

        // 五次多项式插值
        void execute_trajectory_quintic_interpolation(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Executing trajectory...");

            const auto goal = goal_handle->get_goal();
            auto result = std::make_shared<FollowJointTrajectory::Result>();
            auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
            const auto & points = goal->trajectory.points;

            double rate = 300;  // 300Hz 插补
            rclcpp::Rate loop_rate(rate);
            double end_time = points.back().time_from_start.sec + points.back().time_from_start.nanosec * 1e-9;
            size_t idx = 0; // 路径段索引
            auto  point1 = points[idx];
            auto  point2 = points[idx+1];
            double t1 = point1.time_from_start.sec + point1.time_from_start.nanosec * 1e-9;
            double t2 = point2.time_from_start.sec + point2.time_from_start.nanosec * 1e-9;
            double dt = t2 - t1;
            if (dt < 1e-5) dt = 1e-5;
            // 五次多项式的系数
            Eigen::Matrix<double, 6, 6> coef;
            for(int i = 0;i < 6;i++)
            {
                coef(i, 0) = point1.positions[i];
                coef(i, 1) = point1.velocities[i];
                coef(i, 2) = point1.accelerations[i] / 2;
                coef(i, 3) = (20*(point2.positions[i] - point1.positions[i]) - (8*point2.velocities[i] + 12*point1.velocities[i])*dt - (3*point1.accelerations[i] - point2.accelerations[i])*pow(dt,2)) / (2*pow(dt,3));
                coef(i, 4) = (30*(point1.positions[i] - point2.positions[i]) + (14*point2.velocities[i] + 16*point1.velocities[i])*dt + (3*point1.accelerations[i] - 2*point2.accelerations[i])*pow(dt,2)) / (2*pow(dt,4));
                coef(i, 5) = (12*(point2.positions[i] - point1.positions[i]) - 6*(point2.velocities[i] + point1.velocities[i])*dt - (point1.accelerations[i] - point2.accelerations[i])*pow(dt,2)) / (2*pow(dt,5));
            }
            rclcpp::Time trajectory_start_time = this->now();
            while (rclcpp::ok())
            {
                if (goal_handle->is_canceling()) 
                {
                    result->error_code = -5;
                    goal_handle->canceled(result);
                    return;
                }
                double current_time = (this->now() - trajectory_start_time).seconds();
                if (current_time >= end_time) break;

                // 根据绝对时间判断在哪一段路径
                double next_time = points[idx+1].time_from_start.sec + points[idx+1].time_from_start.nanosec * 1e-9;
                if(current_time > next_time)
                {
                    idx++;
                    point1 = points[idx];
                    point2 = points[idx+1];
                    t1 = point1.time_from_start.sec + point1.time_from_start.nanosec * 1e-9;
                    t2 = point2.time_from_start.sec + point2.time_from_start.nanosec * 1e-9;
                    dt = t2 - t1;
                    if (dt < 1e-5) dt = 1e-5;
                    // 五次多项式的系数
                    for(int i = 0;i < 6;i++)
                    {
                        coef(i, 0) = point1.positions[i];
                        coef(i, 1) = point1.velocities[i];
                        coef(i, 2) = point1.accelerations[i] / 2;
                        coef(i, 3) = (20*(point2.positions[i] - point1.positions[i]) - (8*point2.velocities[i] + 12*point1.velocities[i])*dt - (3*point1.accelerations[i] - point2.accelerations[i])*pow(dt,2)) / (2*pow(dt,3));
                        coef(i, 4) = (30*(point1.positions[i] - point2.positions[i]) + (14*point2.velocities[i] + 16*point1.velocities[i])*dt + (3*point1.accelerations[i] - 2*point2.accelerations[i])*pow(dt,2)) / (2*pow(dt,4));
                        coef(i, 5) = (12*(point2.positions[i] - point1.positions[i]) - 6*(point2.velocities[i] + point1.velocities[i])*dt - (point1.accelerations[i] - point2.accelerations[i])*pow(dt,2)) / (2*pow(dt,5));
                    }
                }
                
                double duration_time = current_time - t1;
                Eigen::Vector<double, 6> positions;
                Eigen::Vector<double, 6> t_vector;
                t_vector[0] = 1;
                t_vector[1] = duration_time;
                t_vector[2] = pow(duration_time, 2);
                t_vector[3] = pow(duration_time, 3);
                t_vector[4] = pow(duration_time, 4);
                t_vector[5] = pow(duration_time, 5);
                positions = coef * t_vector;
                positions[1] = positions[1] - M_PI/2.0;
                positions[2] = -positions[2];
                positions[3] = positions[3] - M_PI/2.0;
                std::copy(positions.begin(), positions.end(), target_positions_.begin());
                
                Eigen::Vector<double, 6> velocities;
                Eigen::Vector<double, 6> t_vector_d;
                t_vector_d[0] = 0;
                t_vector_d[1] = 1;
                t_vector_d[2] = 2 * duration_time;
                t_vector_d[3] = 3 * pow(duration_time, 2);
                t_vector_d[4] = 4 * pow(duration_time, 3);
                t_vector_d[5] = 5 * pow(duration_time, 4);
                velocities = coef * t_vector_d;
                velocities[2] = -velocities[2];
                std::copy(velocities.begin(), velocities.end(), target_velocities_.begin());
                
                Eigen::Vector<double, 6> accelerations;
                Eigen::Vector<double, 6> t_vector_dd;
                t_vector_dd[0] = 0;
                t_vector_dd[1] = 0;
                t_vector_dd[2] = 2;
                t_vector_dd[3] = 6 * duration_time;
                t_vector_dd[4] = 12 * pow(duration_time, 2);
                t_vector_dd[5] = 20 * pow(duration_time, 3);
                accelerations = coef * t_vector_dd;
                accelerations[2] = -accelerations[2];
                std::copy(accelerations.begin(), accelerations.end(), target_accelerations_.begin());
                
                // 提取六个关节的期望数据
                array<double, 6> q, qd;
                std::copy(positions_.begin(), positions_.begin() + 6, q.begin());
                std::copy(target_positions_.begin(), target_positions_.begin() + 6, qd.begin());
                q[1] += M_PI/2.0;qd[1] += M_PI/2.0;
                q[2] = -q[2];qd[2] = -qd[2];
                q[3] += M_PI/2.0;qd[3] += M_PI/2.0;
                
                feedback->header.stamp = this->now();
                feedback->header.frame_id = "base_link";
                feedback->actual.positions.resize(6);
                feedback->desired.positions.resize(6);
                feedback->error.positions.resize(6);
                feedback->actual.positions.assign(q.begin(), q.end());     // 当前实际关节角
                feedback->desired.positions.assign(qd.begin(), qd.end());    // 期望关节角
                feedback->joint_names = joint_names_;
                for (size_t j = 0; j < joint_nums_; j++) 
                {
                    feedback->error.positions[j] = feedback->desired.positions[j] - feedback->actual.positions[j];
                }
                
                goal_handle->publish_feedback(feedback);
                loop_rate.sleep();
            }

            target_velocities_.fill(0);
            target_accelerations_.fill(0);

            result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
            result->error_string = "Goal Reached Successfully";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Trajectory execution completed.");
        }

        void process_can_msgs()
        {
            can_id = (RxMessage.can_id>>8) & 0xFF;
            uncalibrated_flag = (RxMessage.can_id>>21) & 0x1;
            hall_flag = (RxMessage.can_id>>20) & 0x1;
            magnetic_flag = (RxMessage.can_id>>19) & 0x1;
            overtemperature_flag = (RxMessage.can_id>>18) & 0x1;
            overcurrent_flag = (RxMessage.can_id>>17) & 0x1;
            undervoltage_flag = (RxMessage.can_id>>16) & 0x1;
            mode_state = (RxMessage.can_id>>22) & 0x3;
            current_angle = data_conversion(RxMessage.data[0]<<8|RxMessage.data[1],angle_min,angle_max);
            current_angular_velocity = data_conversion(RxMessage.data[2]<<8|RxMessage.data[3],angular_velocity_min,angular_velocity_max);
            current_torque = data_conversion(RxMessage.data[4]<<8|RxMessage.data[5],moment_min,moment_max);
            current_temperature = (RxMessage.data[6]<<8|RxMessage.data[7])/10;
            if(hall_flag || magnetic_flag || overtemperature_flag || undervoltage_flag || current_temperature > 40)
            {
                motor_running_ = 0;
            }
            if(can_id == 2)
            {
                positions_[can_id-1] = current_angle * DH2motor_directions_[can_id-1] - M_PI/2.0;
            }
            else if(can_id == 4)
            {
                positions_[can_id-1] = current_angle * DH2motor_directions_[can_id-1] - M_PI/2.0;
            }
            else
            {
                positions_[can_id-1] = current_angle * DH2motor_directions_[can_id-1];
            }
            velocities_pre_[can_id-1] = velocities_[can_id-1];
            velocities_[can_id-1] = current_angular_velocity * DH2motor_directions_[can_id-1];
            torques_[can_id-1] = current_torque * DH2motor_directions_[can_id-1];
        }

        array<double, 3> fourier_serise(double t, array<double, 5> A, array<double, 5> B,double C)
        {
            double q = C, qd = 0, qdd = 0;
            double w = 2 * M_PI * 0.05;
            for(int k = 1;k <= 5;k++)
            {
                q +=  A[k-1]/(w*k) * sin(k*w*t) - B[k-1]/(w*k) * cos(k*w*t);
                qd += A[k-1] * cos(k*w*t) + B[k-1] * sin(k*w*t);
                qdd += -A[k-1] * k * w * sin(k*w*t) + B[k-1] * k * w * cos(k*w*t);
            }
            std::array<double, 3> target;
            target[0] = q;
            target[1] = qd;
            target[2] = qdd;
            return target;
        }

        void read_can_data()
        {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(sock_, &readfds);
            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 5000;
            if(select(sock_+1, &readfds, NULL, NULL, &timeout) > 0)
            {
                ssize_t nbytes = read(sock_, &RxMessage, sizeof(RxMessage));
                if (nbytes == sizeof(struct can_frame))
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    process_can_msgs();
                }
                else
                {
                    cout << "Failed to receive CAN frame." << endl;
                }
            }
        }

        void end_effector_pick()
        {
            static struct can_frame TxMessage;
            TxMessage.can_id = 0x07;
            TxMessage.can_dlc = 7;
            TxMessage.data[0] = 0x02;
            TxMessage.data[1] = 0x01;
            TxMessage.data[2] = 0x20;
            TxMessage.data[3] = 0x49;
            TxMessage.data[4] = 0x20;
            TxMessage.data[5] = 0x00;
            TxMessage.data[6] = 0xC8;
            if (write(sock_, &TxMessage, sizeof(TxMessage)) == -1)
            {
                cout << "Failed to send CAN frame." << endl;
                close(sock_);
            }
        }

        void end_effector_place()
        {
            static struct can_frame TxMessage;
            TxMessage.can_id = 0x07;
            TxMessage.can_dlc = 7;
            TxMessage.data[0] = 0x02;
            TxMessage.data[1] = 0x00;
            TxMessage.data[2] = 0x20;
            TxMessage.data[3] = 0x49;
            TxMessage.data[4] = 0x20;
            TxMessage.data[5] = 0x00;
            TxMessage.data[6] = 0xC8;
            if (write(sock_, &TxMessage, sizeof(TxMessage)) == -1)
            {
                cout << "Failed to send CAN frame." << endl;
                close(sock_);
            }
        }

        void init()
        {
            // 标记线程开始运行
            thread_running_ = true;
            // 启动CAN的接收线程
            std::thread rx_thread([this]()
            {
                while(rclcpp::ok() && thread_running_) 
                {
                    read_can_data();
                }
            });
            rx_thread.detach();
            for(int i = 0;i < joint_nums_;i++)
            {
                // 设置机械零位
                set_mechzero(motors_id[i], master_id, sock_);
                // 电机使能
                motor_enable(motors_id[i], master_id, sock_);
                // 初始化目标角度和PD参数
                motor_controlmode(motors_id[i], 0, 0, 0, 66, 0.6, sock_);
            }
        }

        void write_motors(array<double, 6> torques, array<double, 6> positions)
        {
            motor_controlmode(motors_id[0], torques[0], positions[0], 0, 60, 0.2, sock_);
            motor_controlmode(motors_id[1], torques[1], positions[1], 0, 100, 0.2, sock_);
            motor_controlmode(motors_id[2], torques[2], positions[2], 0, 80, 0.2, sock_);
            motor_controlmode(motors_id[3], torques[3], positions[3], 0, 60, 0.2, sock_);
            motor_controlmode(motors_id[4], torques[4], positions[4], 0, 40, 0.2, sock_);
            motor_controlmode(motors_id[5], torques[5], positions[5], 0, 30, 0.2, sock_);
        }

        void run()
        {
            // 设置控制频率3ms
            rclcpp::Rate loop_rate(333);
            array<double, 6> feedforward_torques;
            while(rclcpp::ok() && motor_running_)
            {
                //// 统计控制频率代码
                // auto start = std::chrono::high_resolution_clock::now();
                // 转换成电机期望的方向和角度
                array<double, 6> q;
                std::copy(target_positions_.begin(), target_positions_.end(), q.begin());
                for(int i = 0;i < joint_nums_;i++)
                {
                    q[i] = q[i] * DH2motor_directions_[i];
                }
                q[1] += M_PI/2.0;
                q[3] += M_PI/2.0;
                write_motors(feedforward_torques, q);
                loop_rate.sleep();
                // auto end = std::chrono::high_resolution_clock::now();
                // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                // std::chrono::duration<double> diff = end - start;
                // double cost_seconds = diff.count(); // 获取秒数值 (double 类型)
                // static int count = 0;
                // if (count++ % 100 == 0)
                // {
                //     // RCLCPP_INFO(this->get_logger(), "核心控制耗时: %.6f s (秒)", cost_seconds);
                //     for(int i = 0;i < joint_nums_;i++)
                //     {
                //         RCLCPP_INFO(this->get_logger(), "torques_[%d]: %.6f", i, torques_[i]);
                //     }
                // }
            }
        }


    private:
        // saber基本属性
        vector<string> joint_names_;
        uint8_t joint_nums_ = 6;

        // moveit to DH
        array<int8_t, 6> moveit2DH_directions_ = {1, 1, -1, 1, 1, 1};
        // DH to motor
        array<int8_t, 6> DH2motor_directions_ = {-1, 1, -1, 1, -1, -1};

        // saber关节控制器发布状态
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr observer_pub_;

        // 用于发布状态的定时器
        rclcpp::TimerBase::SharedPtr timer_;

        // moveit通讯接口
        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr moveit_action_server_;

        // saber各个关节实际状态
        array<double, 6> positions_;
        array<double, 6> velocities_;
        array<double, 6> velocities_pre_;
        array<double, 6> torques_;

        // saber各个关节期望状态
        array<double, 6> target_positions_;
        array<double, 6> target_velocities_;
        array<double, 6> target_accelerations_;

        // 最小惯性参数集和傅里叶级数系数
        VectorXd PA_ = VectorXd::Zero(48);
        vector<double> fourier_series_;
        vector<array<double, 5>> A_;
        vector<array<double, 5>> B_;
        array<double, 5> C_;

        // 观测的关节外力
        Vector6d tau_hat_ext_;

        // CAN通信所需变量
        struct can_frame RxMessage;
        int sock_;

        // 小米微电机反馈数据
        uint8_t can_id = 0;
        uint8_t uncalibrated_flag = 0;
        uint8_t hall_flag = 0;
        uint8_t magnetic_flag = 0;
        uint8_t overtemperature_flag = 0;
        uint8_t overcurrent_flag = 0;
        uint8_t undervoltage_flag = 0;
        uint8_t mode_state = 0;
        double current_angle = 0;
        double current_angular_velocity = 0;
        double current_torque = 0;
        double current_temperature = 0;

        // 电机可运行标志位
        uint8_t motor_running_ = 1;

        // 电机ID定义
        array<uint8_t, 6> motors_id = {1, 2, 3, 4, 5, 6};
        uint8_t master_id = 0;

        std::thread rx_thread_;        // 保存线程句柄
        std::atomic<bool> thread_running_; // 控制线程退出的标志位
        std::mutex mutex_;             // 保护数据的锁

};

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);

    //// CAN通信初始化
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock == -1)
        cout << "Failed to create SocketCAN socket." << endl;

    cout << "SocketCAN socket created." << endl;
    struct sockaddr_can addr;
    struct ifreq ifr;
    std::strcpy(ifr.ifr_ifrn.ifrn_name, "can1");
    if (ioctl(sock, SIOCGIFINDEX, &ifr) == -1)
    {
        cout << "Failed to get CAN interface index." << endl;
        close(sock);
    }
    cout << "Get CAN interface index." << endl;

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifru.ifru_ivalue;

    // 绑定CAN地址
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) == -1)
    {
        cout << "Failed to bind SocketCAN socket." << endl;
        close(sock);
    }
    cout << "Bind SocketCAN socket." << endl;

    auto saber_joint_controller_node = std::make_shared<saber_joint_controller>("saber_joint_controller", sock);
    // 初始化电机
    saber_joint_controller_node->init();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(saber_joint_controller_node);
    std::thread spin_thread([&executor]() {
        executor.spin();
    });
    saber_joint_controller_node->run();
    rclcpp::shutdown();
    return 0;
}