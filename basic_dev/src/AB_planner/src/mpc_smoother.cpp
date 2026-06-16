#include "mpc_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
constexpr double kBoundInf = 1e10;
}

MpcSmoother::MpcSmoother(const Config& cfg) : cfg_(cfg)
{
    cfg_.N = std::max(1, cfg_.N);
    cfg_.dt = std::max(1e-3, cfg_.dt);
    cfg_.max_jerk = std::max(1e-3, cfg_.max_jerk);
    cfg_.max_acc = std::max(1e-3, cfg_.max_acc);
    cfg_.max_vel = std::max(1e-3, cfg_.max_vel);
    cfg_.w_pos = std::max(0.0, cfg_.w_pos);
    cfg_.w_vel = std::max(0.0, cfg_.w_vel);
    cfg_.w_terminal_pos = std::max(0.0, cfg_.w_terminal_pos);
    cfg_.w_jerk = std::max(1e-9, cfg_.w_jerk);
    cfg_.w_jerk_delta = std::max(0.0, cfg_.w_jerk_delta);
    cfg_.obstacle_safe_distance = std::max(0.0, cfg_.obstacle_safe_distance);
    cfg_.obstacle_active_distance = std::max(cfg_.obstacle_safe_distance, cfg_.obstacle_active_distance);
    cfg_.obstacle_max_count = std::max(0, cfg_.obstacle_max_count);
    cfg_.obstacle_check_step = std::max(1, cfg_.obstacle_check_step);

    buildProblem();
}

void MpcSmoother::setObstacles(const std::vector<MpcObstacle>& obstacles)
{
    obstacles_ = obstacles;
}

void MpcSmoother::buildProblem()
{
    const int N = cfg_.N;
    const double dt = cfg_.dt;
    const int state_dim = 9;
    const int input_dim = 3;
    const int state_vars = state_dim * (N + 1);
    const int input_vars = input_dim * N;

    obstacle_step_indices_.clear();
    if (cfg_.obstacle_constraints_enabled && cfg_.obstacle_max_count > 0) {
        for (int k = cfg_.obstacle_check_step; k <= N; k += cfg_.obstacle_check_step) {
            obstacle_step_indices_.push_back(k);
        }
        if (obstacle_step_indices_.empty() || obstacle_step_indices_.back() != N) {
            obstacle_step_indices_.push_back(N);
        }
    }

    num_vars_ = state_vars + input_vars;
    obstacle_constraint_start_row_ = state_dim + state_dim * N + input_dim * N * 3;
    num_cons_ = obstacle_constraint_start_row_ +
                static_cast<int>(obstacle_step_indices_.size()) * cfg_.obstacle_max_count * 6;

    std::vector<Eigen::Triplet<double>> hessian_triplets;
    hessian_triplets.reserve(6 * (N + 1) + 9 * N);

    for (int k = 0; k <= N; ++k) {
        const double weight = (k == N) ? cfg_.w_terminal_pos : cfg_.w_pos;
        for (int dim = 0; dim < 3; ++dim) {
            hessian_triplets.emplace_back(k * state_dim + dim, k * state_dim + dim, weight);
            hessian_triplets.emplace_back(k * state_dim + 3 + dim, k * state_dim + 3 + dim, cfg_.w_vel);
        }
    }

    for (int k = 0; k < N; ++k) {
        for (int dim = 0; dim < 3; ++dim) {
            const int idx = state_vars + k * input_dim + dim;
            hessian_triplets.emplace_back(idx, idx, cfg_.w_jerk);
        }
    }

    for (int k = 1; k < N; ++k) {
        for (int dim = 0; dim < 3; ++dim) {
            const int prev = state_vars + (k - 1) * input_dim + dim;
            const int curr = state_vars + k * input_dim + dim;
            hessian_triplets.emplace_back(prev, prev, cfg_.w_jerk_delta);
            hessian_triplets.emplace_back(curr, curr, cfg_.w_jerk_delta);
            hessian_triplets.emplace_back(prev, curr, -cfg_.w_jerk_delta);
            hessian_triplets.emplace_back(curr, prev, -cfg_.w_jerk_delta);
        }
    }

    hessian_.resize(num_vars_, num_vars_);
    hessian_.setFromTriplets(hessian_triplets.begin(), hessian_triplets.end());

    std::vector<Eigen::Triplet<double>> constraint_triplets;
    constraint_triplets.reserve(state_dim + state_dim * N * 5 + input_dim * N * 3);
    int row = 0;

    for (int i = 0; i < state_dim; ++i) {
        constraint_triplets.emplace_back(row + i, i, 1.0);
    }
    row += state_dim;

    for (int k = 0; k < N; ++k) {
        const int xk = k * state_dim;
        const int xk1 = (k + 1) * state_dim;
        const int uk = state_vars + k * input_dim;

        for (int dim = 0; dim < 3; ++dim) {
            const int p = dim;
            const int v = 3 + dim;
            const int a = 6 + dim;

            constraint_triplets.emplace_back(row + p, xk1 + p, 1.0);
            constraint_triplets.emplace_back(row + p, xk + p, -1.0);
            constraint_triplets.emplace_back(row + p, xk + v, -dt);
            constraint_triplets.emplace_back(row + p, xk + a, -0.5 * dt * dt);
            constraint_triplets.emplace_back(row + p, uk + dim, -(dt * dt * dt) / 6.0);

            constraint_triplets.emplace_back(row + v, xk1 + v, 1.0);
            constraint_triplets.emplace_back(row + v, xk + v, -1.0);
            constraint_triplets.emplace_back(row + v, xk + a, -dt);
            constraint_triplets.emplace_back(row + v, uk + dim, -0.5 * dt * dt);

            constraint_triplets.emplace_back(row + a, xk1 + a, 1.0);
            constraint_triplets.emplace_back(row + a, xk + a, -1.0);
            constraint_triplets.emplace_back(row + a, uk + dim, -dt);
        }
        row += state_dim;
    }

    for (int k = 1; k <= N; ++k) {
        for (int dim = 0; dim < 3; ++dim) {
            constraint_triplets.emplace_back(row++, k * state_dim + 3 + dim, 1.0);
        }
    }

    for (int k = 1; k <= N; ++k) {
        for (int dim = 0; dim < 3; ++dim) {
            constraint_triplets.emplace_back(row++, k * state_dim + 6 + dim, 1.0);
        }
    }

    for (int k = 0; k < N; ++k) {
        for (int dim = 0; dim < 3; ++dim) {
            constraint_triplets.emplace_back(row++, state_vars + k * input_dim + dim, 1.0);
        }
    }

    for (size_t step_i = 0; step_i < obstacle_step_indices_.size(); ++step_i) {
        const int k = obstacle_step_indices_[step_i];
        for (int obs_i = 0; obs_i < cfg_.obstacle_max_count; ++obs_i) {
            constraint_triplets.emplace_back(row++, k * state_dim + 0, 1.0);
            constraint_triplets.emplace_back(row++, k * state_dim + 0, -1.0);
            constraint_triplets.emplace_back(row++, k * state_dim + 1, 1.0);
            constraint_triplets.emplace_back(row++, k * state_dim + 1, -1.0);
            constraint_triplets.emplace_back(row++, k * state_dim + 2, 1.0);
            constraint_triplets.emplace_back(row++, k * state_dim + 2, -1.0);
        }
    }

    constraint_matrix_.resize(num_cons_, num_vars_);
    constraint_matrix_.setFromTriplets(constraint_triplets.begin(), constraint_triplets.end());

    gradient_.setZero(num_vars_);
    lower_bound_.setZero(num_cons_);
    upper_bound_.setZero(num_cons_);

    row = state_dim + state_dim * N;
    for (int k = 1; k <= N; ++k) {
        lower_bound_.segment<3>(row).setConstant(-cfg_.max_vel);
        upper_bound_.segment<3>(row).setConstant(cfg_.max_vel);
        row += input_dim;
    }
    for (int k = 1; k <= N; ++k) {
        lower_bound_.segment<3>(row).setConstant(-cfg_.max_acc);
        upper_bound_.segment<3>(row).setConstant(cfg_.max_acc);
        row += input_dim;
    }
    for (int k = 0; k < N; ++k) {
        lower_bound_.segment<3>(row).setConstant(-cfg_.max_jerk);
        upper_bound_.segment<3>(row).setConstant(cfg_.max_jerk);
        row += input_dim;
    }
    for (; row < num_cons_; ++row) {
        lower_bound_(row) = -kBoundInf;
        upper_bound_(row) = kBoundInf;
    }

    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.data()->setNumberOfVariables(num_vars_);
    solver_.data()->setNumberOfConstraints(num_cons_);
    solver_.data()->setHessianMatrix(hessian_);
    solver_.data()->setGradient(gradient_);
    solver_.data()->setLinearConstraintsMatrix(constraint_matrix_);
    solver_.data()->setLowerBound(lower_bound_);
    solver_.data()->setUpperBound(upper_bound_);
    solver_initialized_ = solver_.initSolver();
}

void MpcSmoother::updateObstacleConstraints(const StateVector& x0,
                                            const Eigen::MatrixXd& ref_pos,
                                            int start_row)
{
    const int N = cfg_.N;
    const int max_obstacles = std::min(cfg_.obstacle_max_count, static_cast<int>(obstacles_.size()));

    for (int row = start_row; row < num_cons_; ++row) {
        lower_bound_(row) = -kBoundInf;
        upper_bound_(row) = kBoundInf;
    }

    for (size_t step_i = 0; step_i < obstacle_step_indices_.size(); ++step_i) {
        const int k = std::max(1, std::min(obstacle_step_indices_[step_i], N));
        const Eigen::Vector3d ref_point = ref_pos.block<3, 1>(0, k);

        for (int obs_i = 0; obs_i < cfg_.obstacle_max_count; ++obs_i) {
            const int row_base = start_row + (static_cast<int>(step_i) * cfg_.obstacle_max_count + obs_i) * 6;

            if (obs_i >= max_obstacles) {
                continue;
            }

            const MpcObstacle& obstacle = obstacles_[obs_i];
            if (!std::isfinite(obstacle.center.x()) || !std::isfinite(obstacle.center.y()) || !std::isfinite(obstacle.center.z()) ||
                !std::isfinite(obstacle.size.x()) || !std::isfinite(obstacle.size.y()) || !std::isfinite(obstacle.size.z())) {
                continue;
            }

            const Eigen::Vector3d raw_half_size = 0.5 * obstacle.size.cwiseMax(Eigen::Vector3d::Zero()) +
                                              Eigen::Vector3d::Constant(cfg_.obstacle_safe_distance);
            Eigen::Matrix3d R = obstacle.orientation.toRotationMatrix();
            Eigen::Vector3d ex = R.col(0) * raw_half_size.x();
            Eigen::Vector3d ey = R.col(1) * raw_half_size.y();
            Eigen::Vector3d ez = R.col(2) * raw_half_size.z();
            const Eigen::Vector3d half_size(
                std::abs(ex.x()) + std::abs(ey.x()) + std::abs(ez.x()),
                std::abs(ex.y()) + std::abs(ey.y()) + std::abs(ez.y()),
                std::abs(ex.z()) + std::abs(ey.z()) + std::abs(ez.z())
            );

            Eigen::Vector3d direction = ref_point - obstacle.center;
            const double ref_distance = direction.norm();
            if (ref_distance > cfg_.obstacle_active_distance + half_size.norm()) {
                continue;
            }

            if (ref_distance < 1e-3) {
                direction = x0.segment<3>(0) - obstacle.center;
            }
            if (direction.norm() < 1e-3) {
                direction = x0.segment<3>(3);
            }
            if (direction.norm() < 1e-3) {
                direction = Eigen::Vector3d::UnitX();
            }

            Eigen::Vector3d normalized_direction = direction.cwiseQuotient(half_size.cwiseMax(Eigen::Vector3d::Constant(1e-3)));
            Eigen::Index axis_idx = 0;
            normalized_direction.cwiseAbs().maxCoeff(&axis_idx);
            const int axis = static_cast<int>(axis_idx);
            const double sign = direction(axis) >= 0.0 ? 1.0 : -1.0;
            const int axis_row = row_base + axis * 2 + (sign > 0.0 ? 0 : 1);
            lower_bound_(axis_row) = sign > 0.0
                ? obstacle.center(axis) + half_size(axis)
                : -obstacle.center(axis) + half_size(axis);
            upper_bound_(axis_row) = kBoundInf;
        }
    }
}

bool MpcSmoother::solve(const StateVector& x0,
                        const Eigen::MatrixXd& ref_pos,
                        Eigen::MatrixXd& out_states,
                        Eigen::MatrixXd& out_inputs)
{
    const int N = cfg_.N;
    const int state_dim = 9;
    const int input_dim = 3;
    const int state_vars = state_dim * (N + 1);

    if (!solver_initialized_ ||
        (ref_pos.rows() != 3 && ref_pos.rows() != 6) ||
        ref_pos.cols() != N + 1) {
        return false;
    }

    gradient_.setZero(num_vars_);
    for (int k = 0; k <= N; ++k) {
        const double weight = (k == N) ? cfg_.w_terminal_pos : cfg_.w_pos;
        for (int dim = 0; dim < 3; ++dim) {
            gradient_(k * state_dim + dim) = -weight * ref_pos(dim, k);
            if (ref_pos.rows() == 6) {
                gradient_(k * state_dim + 3 + dim) = -cfg_.w_vel * ref_pos(3 + dim, k);
            }
        }
    }

    lower_bound_.head<state_dim>() = x0;
    upper_bound_.head<state_dim>() = x0;

    int row = state_dim;
    for (int k = 0; k < N; ++k) {
        lower_bound_.segment<state_dim>(row).setZero();
        upper_bound_.segment<state_dim>(row).setZero();
        row += state_dim;
    }

    for (int k = 1; k <= N; ++k) {
        lower_bound_.segment<3>(row).setConstant(-cfg_.max_vel);
        upper_bound_.segment<3>(row).setConstant(cfg_.max_vel);
        row += input_dim;
    }
    for (int k = 1; k <= N; ++k) {
        lower_bound_.segment<3>(row).setConstant(-cfg_.max_acc);
        upper_bound_.segment<3>(row).setConstant(cfg_.max_acc);
        row += input_dim;
    }
    for (int k = 0; k < N; ++k) {
        lower_bound_.segment<3>(row).setConstant(-cfg_.max_jerk);
        upper_bound_.segment<3>(row).setConstant(cfg_.max_jerk);
        row += input_dim;
    }
    const int obstacle_bound_start_row = row;
    updateObstacleConstraints(x0, ref_pos, row);

    if (!solver_.updateGradient(gradient_) || !solver_.updateBounds(lower_bound_, upper_bound_)) {
        return false;
    }

    solver_.solveProblem();
    if (solver_.getStatus() != OsqpEigen::Status::Solved) {
        const bool used_obstacle_constraints =
            cfg_.obstacle_constraints_enabled && !obstacles_.empty() && obstacle_bound_start_row < num_cons_;
        if (!used_obstacle_constraints) {
            return false;
        }

        for (int bound_row = obstacle_bound_start_row; bound_row < num_cons_; ++bound_row) {
            lower_bound_(bound_row) = -kBoundInf;
            upper_bound_(bound_row) = kBoundInf;
        }
        if (!solver_.updateBounds(lower_bound_, upper_bound_)) {
            return false;
        }
        solver_.solveProblem();
        if (solver_.getStatus() != OsqpEigen::Status::Solved) {
            return false;
        }
    }

    const Eigen::VectorXd solution = solver_.getSolution();
    out_states = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>>(
        solution.data(), state_dim, N + 1);
    out_inputs = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>>(
        solution.data() + state_vars, input_dim, N);

    return true;
}
