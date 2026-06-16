#ifndef _SCP_MPC_SMOOTHER_HPP_
#define _SCP_MPC_SMOOTHER_HPP_

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>

class MpcSmoother
{
public:
    using StateVector = Eigen::Matrix<double, 9, 1>;

    struct Config {
        int N = 50;               // 预测步数
        double dt = 0.02;         // 50 Hz
        double max_jerk = 25.0;
        double max_acc = 6.0;
        double max_vel = 15.0;
        double w_pos = 50.0;      // 位置跟踪权重
        double w_terminal_pos = 200.0;
        double w_jerk = 0.1;
        double w_jerk_delta = 0.05;
    };

    MpcSmoother(const Config& cfg);

    // 输入：当前状态 [px,py,pz,vx,vy,vz,ax,ay,az]
    //       参考位置矩阵 3×(N+1) （只提供位置，速度/加速度自由）
    // 输出：全部状态矩阵 9×(N+1)，控制输入 3×N
    bool solve(const StateVector& x0,
               const Eigen::MatrixXd& ref_pos, // 3 x (N+1)
               Eigen::MatrixXd& out_states,
               Eigen::MatrixXd& out_inputs);

private:
    Config cfg_;
    OsqpEigen::Solver solver_;
    Eigen::SparseMatrix<double> H_;
    Eigen::VectorXd g_;
    Eigen::VectorXd lb_;
    Eigen::VectorXd ub_;
    int num_vars_, num_cons_;
    bool solver_initialized_ = false;

};

#endif
