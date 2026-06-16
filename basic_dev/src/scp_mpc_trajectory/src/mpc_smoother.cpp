#include "mpc_smoother.hpp"

MpcSmoother::MpcSmoother(const Config& cfg) : cfg_(cfg) {
    int N = cfg_.N;
    double dt = cfg_.dt;
    num_vars_ = 9 * (N + 1) + 3 * N;       // 状态 + 控制
    num_cons_ = 9 + 9 * N + 3 * N + 3 * N + 3 * N; // 初始 + 动力学 + 速度/加速度/jerk 限幅

    // ---------- Hessian ----------
    H_.resize(num_vars_, num_vars_);
    std::vector<Eigen::Triplet<double>> H_trip;
    // 位置跟踪代价 (只跟踪前三个轴)
    for (int k = 0; k <= N; ++k)
        for (int dim = 0; dim < 3; ++dim)
            H_trip.emplace_back(k*9 + dim, k*9 + dim, k == N ? cfg_.w_terminal_pos : cfg_.w_pos);
    // jerk 惩罚
    for (int k = 0; k < N; ++k)
        for (int dim = 0; dim < 3; ++dim)
            H_trip.emplace_back(9*(N+1) + k*3 + dim, 9*(N+1) + k*3 + dim, cfg_.w_jerk);
    // 相邻 jerk 变化率惩罚，降低 jerk 贴边和高频抖动。
    for (int k = 1; k < N; ++k) {
        for (int dim = 0; dim < 3; ++dim) {
            const int u_prev = 9*(N+1) + (k-1)*3 + dim;
            const int u_curr = 9*(N+1) + k*3 + dim;
            H_trip.emplace_back(u_prev, u_prev, cfg_.w_jerk_delta);
            H_trip.emplace_back(u_curr, u_curr, cfg_.w_jerk_delta);
            H_trip.emplace_back(u_prev, u_curr, -cfg_.w_jerk_delta);
            H_trip.emplace_back(u_curr, u_prev, -cfg_.w_jerk_delta);
        }
    }
    H_.setFromTriplets(H_trip.begin(), H_trip.end());

    // ---------- 约束矩阵 ----------
    Eigen::SparseMatrix<double> A_cons(num_cons_, num_vars_);
    std::vector<Eigen::Triplet<double>> A_trip;
    int row = 0;

    // 初始状态等式 x0 = current
    for (int i = 0; i < 9; ++i) {
        A_trip.emplace_back(row + i, i, 1.0);
    }
    row += 9;

    // 动力学等式，状态顺序: [px,py,pz,vx,vy,vz,ax,ay,az]，控制输入为 jerk。
    for (int k = 0; k < N; ++k) {
        int xk = k*9, xk1 = (k+1)*9, uk = 9*(N+1) + k*3;
        for (int dim = 0; dim < 3; ++dim) {
            const int p = dim;
            const int v = 3 + dim;
            const int a = 6 + dim;

            // p_{k+1} - p_k - dt*v_k - 0.5*dt^2*a_k - dt^3/6*j_k = 0
            A_trip.emplace_back(row + p, xk1 + p, 1.0);
            A_trip.emplace_back(row + p, xk + p, -1.0);
            A_trip.emplace_back(row + p, xk + v, -dt);
            A_trip.emplace_back(row + p, xk + a, -0.5 * dt * dt);
            A_trip.emplace_back(row + p, uk + dim, -(dt * dt * dt) / 6.0);

            // v_{k+1} - v_k - dt*a_k - 0.5*dt^2*j_k = 0
            A_trip.emplace_back(row + v, xk1 + v, 1.0);
            A_trip.emplace_back(row + v, xk + v, -1.0);
            A_trip.emplace_back(row + v, xk + a, -dt);
            A_trip.emplace_back(row + v, uk + dim, -0.5 * dt * dt);

            // a_{k+1} - a_k - dt*j_k = 0
            A_trip.emplace_back(row + a, xk1 + a, 1.0);
            A_trip.emplace_back(row + a, xk + a, -1.0);
            A_trip.emplace_back(row + a, uk + dim, -dt);
        }
        row += 9;
    }

    // 速度约束 (3*N 个)
    for (int k = 0; k < N; ++k) {
        for (int dim = 0; dim < 3; ++dim)
            A_trip.emplace_back(row++, k*9 + 3 + dim, 1.0); // vx, vy, vz
    }

    // 加速度约束 (3*N 个)
    for (int k = 0; k < N; ++k) {
        for (int dim = 0; dim < 3; ++dim)
            A_trip.emplace_back(row++, k*9 + 6 + dim, 1.0);
    }
    // jerk 约束 (3*N 个)
    for (int k = 0; k < N; ++k) {
        for (int dim = 0; dim < 3; ++dim)
            A_trip.emplace_back(row++, 9*(N+1) + k*3 + dim, 1.0);
    }
    A_cons.setFromTriplets(A_trip.begin(), A_trip.end());

    // OSQP 初始化时必须提供完整 g/lb/ub；后续 solve() 只更新这些向量。
    g_.setZero(num_vars_);
    lb_.setZero(num_cons_);
    ub_.setZero(num_cons_);

    row = 9 + 9 * N;
    for (int k = 0; k < N; ++k) {
        lb_.segment<3>(row).setConstant(-cfg_.max_vel);
        ub_.segment<3>(row).setConstant( cfg_.max_vel);
        row += 3;
    }
    for (int k = 0; k < N; ++k) {
        lb_.segment<3>(row).setConstant(-cfg_.max_acc);
        ub_.segment<3>(row).setConstant( cfg_.max_acc);
        row += 3;
    }
    for (int k = 0; k < N; ++k) {
        lb_.segment<3>(row).setConstant(-cfg_.max_jerk);
        ub_.segment<3>(row).setConstant( cfg_.max_jerk);
        row += 3;
    }

    // ---------- OSQP 设置 ----------
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.data()->setNumberOfVariables(num_vars_);
    solver_.data()->setNumberOfConstraints(num_cons_);
    solver_.data()->setHessianMatrix(H_);
    solver_.data()->setGradient(g_);
    solver_.data()->setLinearConstraintsMatrix(A_cons);
    solver_.data()->setLowerBound(lb_);
    solver_.data()->setUpperBound(ub_);
    solver_initialized_ = solver_.initSolver();
}

bool MpcSmoother::solve(const StateVector& x0,
                        const Eigen::MatrixXd& ref_pos,
                        Eigen::MatrixXd& out_states,
                        Eigen::MatrixXd& out_inputs) {
    int N = cfg_.N;
    if (!solver_initialized_ || ref_pos.rows() != 3 || ref_pos.cols() != N + 1) {
        return false;
    }

    // ---------- 梯度 (位置跟踪) ----------
    g_.setZero(num_vars_);
    for (int k = 0; k <= N; ++k)
        for (int dim = 0; dim < 3; ++dim)
            g_(k*9 + dim) = -(k == N ? cfg_.w_terminal_pos : cfg_.w_pos) * ref_pos(dim, k);

    // ---------- 上下界 ----------
    // 初始状态
    lb_.head<9>() = x0;
    ub_.head<9>() = x0;
    int row = 9;
    // 动力学等式 (右端=0)
    for (int k = 0; k < N; ++k) {
        lb_.segment<9>(row).setZero();
        ub_.segment<9>(row).setZero();
        row += 9;
    }
    // 速度约束
    for (int k = 0; k < N; ++k) {
        lb_.segment<3>(row).setConstant(-cfg_.max_vel);
        ub_.segment<3>(row).setConstant( cfg_.max_vel);
        row += 3;
    }
    // 加速度约束
    for (int k = 0; k < N; ++k) {
        lb_.segment<3>(row).setConstant(-cfg_.max_acc);
        ub_.segment<3>(row).setConstant( cfg_.max_acc);
        row += 3;
    }
    // jerk 约束
    for (int k = 0; k < N; ++k) {
        lb_.segment<3>(row).setConstant(-cfg_.max_jerk);
        ub_.segment<3>(row).setConstant( cfg_.max_jerk);
        row += 3;
    }

    if (!solver_.updateGradient(g_) || !solver_.updateBounds(lb_, ub_)) {
        return false;
    }
    solver_.solveProblem();

    if (solver_.getStatus() == OsqpEigen::Status::Solved) {
        Eigen::VectorXd sol = solver_.getSolution();
        out_states = Eigen::Map<Eigen::MatrixXd>(sol.data(), 9, N+1);
        out_inputs = Eigen::Map<Eigen::MatrixXd>(sol.data() + 9*(N+1), 3, N);
        return true;
    }
    return false;
}
