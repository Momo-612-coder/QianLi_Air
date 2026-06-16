#ifndef _MPC_SMOOTHER_HPP_
#define _MPC_SMOOTHER_HPP_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>

struct MpcObstacle
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d size = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

class MpcSmoother
{
public:
    using StateVector = Eigen::Matrix<double, 9, 1>;

    struct Config {
        int N = 50;                    // prediction steps
        double dt = 0.02;              // control interval
        double max_jerk = 25.0;        // m/s^3
        double max_acc = 6.0;          // m/s^2
        double max_vel = 15.0;         // m/s
        double w_pos = 50.0;
        double w_vel = 10.0;
        double w_terminal_pos = 50.0;
        double w_jerk = 0.1;
        double w_jerk_delta = 0.05;
        bool obstacle_constraints_enabled = true;
        double obstacle_safe_distance = 2.0;
        double obstacle_active_distance = 8.0;
        int obstacle_max_count = 3;
        int obstacle_check_step = 5;
    };

    explicit MpcSmoother(const Config& cfg);

    void setObstacles(const std::vector<MpcObstacle>& obstacles);

    bool solve(const StateVector& x0,
               const Eigen::MatrixXd& ref_pos,
               Eigen::MatrixXd& out_states,
               Eigen::MatrixXd& out_inputs);

private:
    void buildProblem();
    void updateObstacleConstraints(const StateVector& x0, const Eigen::MatrixXd& ref_pos, int start_row);

    Config cfg_;
    OsqpEigen::Solver solver_;

    Eigen::SparseMatrix<double> hessian_;
    Eigen::SparseMatrix<double> constraint_matrix_;
    Eigen::VectorXd gradient_;
    Eigen::VectorXd lower_bound_;
    Eigen::VectorXd upper_bound_;

    int num_vars_ = 0;
    int num_cons_ = 0;
    int obstacle_constraint_start_row_ = 0;
    std::vector<int> obstacle_step_indices_;
    std::vector<MpcObstacle> obstacles_;
    bool solver_initialized_ = false;
};

#endif
