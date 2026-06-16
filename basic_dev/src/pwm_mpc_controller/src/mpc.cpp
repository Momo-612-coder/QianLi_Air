#include "mpc.hpp"

MPC::MPC()
{

}

MPC::~MPC()
{

}

// 初始化MPC问题，设置状态转移函数、代价函数、约束等
void MPC::init_mpc_problem(const problem_params_t& problem_params, const model_params_t& model_params)
{
    this->problem_params = problem_params;
    this->model_params = model_params;
    Function dynamics = ned_x_quad_dynamics(model_params);
	const double ts = problem_params.ts;
	const int N = problem_params.N;
	DM R = problem_params.R;
	DM Q = problem_params.Q;
	DM P = problem_params.P;
	const double q_att_weight = problem_params.q_att_weight;
	const double q_att_terminal_weight = problem_params.q_att_terminal_weight;

	MX X0 = MX::sym("X0", n_state);
	MX Xd = MX::sym("Xd", n_state, N + 1);
	MX Ud = MX::sym("Ud", n_input, N);
	MX X = MX::sym("X", n_state, N + 1);
	MX U = MX::sym("U", n_input, N);

	std::vector<double> eq_state(n_state, 0.0);
	std::vector<double> roll_pitch_lb{-problem_params.max_roll_pitch, -problem_params.max_roll_pitch};
	std::vector<double> roll_pitch_ub{problem_params.max_roll_pitch, problem_params.max_roll_pitch};
	std::vector<double> yaw_rate_lb{-problem_params.max_yaw_rate};
	std::vector<double> yaw_rate_ub{problem_params.max_yaw_rate};
	std::vector<double> quat_norm_lb{1.0};
	std::vector<double> quat_norm_ub{1.0};
	std::vector<double> input_lb{problem_params.min_F, -problem_params.max_Tx, -problem_params.max_Ty, -problem_params.max_Tz};
	std::vector<double> input_ub{problem_params.max_F,
								 problem_params.max_Tx,
								 problem_params.max_Ty,
								 problem_params.max_Tz};
	std::vector<double> motor_lb(4, model_params.min_w * model_params.min_w);
	std::vector<double> motor_ub(4, model_params.max_w * model_params.max_w);

	DM S = DM::zeros(9, n_state);
	for (int i = 0; i < 6; ++i) S(i, i) = 1.0;
	for (int i = 0; i < 3; ++i) S(6 + i, 10 + i) = 1.0;
	DM Q_lin = mtimes(S, mtimes(Q, S.T()));
	DM P_lin = mtimes(S, mtimes(P, S.T()));

	DM mixing = get_x_mixing_matrix(model_params.ct, model_params.cm, model_params.R);

	if (!lbg.empty()) lbg.clear();
	if (!ubg.empty()) ubg.clear();

	MX cost = 0;
	std::vector<MX> constraints;

	for (int k = 0; k < N; ++k) {
		MX Uk = U(Slice(), k);
		MX Xk = X(Slice(), k);
		MX qk = X(Slice(6, 10), k);
		MX qd_k = Xd(Slice(6, 10), k);
		MX Uerr = Uk - Ud(Slice(), k);
		MX Xerr = Xk - Xd(Slice(), k);
		MX Xlin_err = mtimes(S, Xerr);

		cost += dot(Uerr, mtimes(R, Uerr));
		cost += dot(Xlin_err, mtimes(Q_lin, Xlin_err));
		cost += q_att_weight * quat_attitude_cost(qk, qd_k);

		// Wrench bounds: [F, Tx, Ty, Tz]. Torque lower bounds are set to 0.
		constraints.push_back(Uk);
		lbg = join(lbg, input_lb);
		ubg = join(ubg, input_ub);

		auto Xdot = dynamics(std::vector<MX>{Xk, Uk});
		constraints.push_back(X(Slice(), k + 1) - Xk - Xdot[0] * ts);
		lbg = join(lbg, eq_state);
		ubg = join(ubg, eq_state);

		// Enforce motor limits via X-frame mixing matrix.
		constraints.push_back(mtimes(mixing, Uk));
		lbg = join(lbg, motor_lb);
		ubg = join(ubg, motor_ub);

		MX rpy_k = quat_to_rpy(qk);
		constraints.push_back(rpy_k(Slice(0, 2)));
		lbg = join(lbg, roll_pitch_lb);
		ubg = join(ubg, roll_pitch_ub);

		constraints.push_back(dot(qk, qk));
		lbg = join(lbg, quat_norm_lb);
		ubg = join(ubg, quat_norm_ub);

		constraints.push_back(X(Slice(12, 13), k));
		lbg = join(lbg, yaw_rate_lb);
		ubg = join(ubg, yaw_rate_ub);
	}

	MX XNerr = X(Slice(), N) - Xd(Slice(), N);
	MX XNlin_err = mtimes(S, XNerr);
	MX qN = X(Slice(6, 10), N);
	MX qd_N = Xd(Slice(6, 10), N);
	cost += dot(XNlin_err, mtimes(P_lin, XNlin_err));
	cost += q_att_terminal_weight * quat_attitude_cost(qN, qd_N);

	constraints.push_back(X(Slice(), 0) - X0);
	lbg = join(lbg, eq_state);
	ubg = join(ubg, eq_state);

	MX rpy_N = quat_to_rpy(qN);
	constraints.push_back(rpy_N(Slice(0, 2)));
	lbg = join(lbg, roll_pitch_lb);
	ubg = join(ubg, roll_pitch_ub);

	constraints.push_back(dot(qN, qN));
	lbg = join(lbg, quat_norm_lb);
	ubg = join(ubg, quat_norm_ub);

	constraints.push_back(X(Slice(12, 13), N));
	lbg = join(lbg, yaw_rate_lb);
	ubg = join(ubg, yaw_rate_ub);

	nlp = {
			{"x", vertcat(reshape(U, -1, 1), reshape(X, -1, 1))},
			{"p", vertcat(X0, reshape(Xd, -1, 1), reshape(Ud, -1, 1))},
			{"f", cost},
			{"g", vertcat(constraints)}
	};
}

void MPC::init_solver(std::string solver_type,  casadi::Dict solver_opts) {
	//mpc_solver = nlpsol("solver", solver_type, nlp, solver_opts);
	mpc_solver = casadi::nlpsol("solver", solver_type, nlp, solver_opts);
}

// 构建一个四旋翼飞行器在 NED 坐标系下的连续时间非线性动力学模型
Function MPC::ned_x_quad_dynamics(model_params_t params) {
	const double g = params.g;
	const double m = params.m;
	const DM J = DM::diag({params.Jxx, params.Jyy, params.Jzz});

	// Inputs
	MX f = MX::sym("f", 1);
	MX tau = MX::sym("tau", 3);

	// States in NED
	MX p_ned = MX::sym("p_ned", 3);
	MX v_ned = MX::sym("v_ned", 3);
	MX q = MX::sym("q", 4);          // [qw, qx, qy, qz], body->NED
	MX w = MX::sym("w", 3);          // body rates [p, q, r]

	MX e3 = MX::vertcat(std::vector<MX>{MX(0), MX(0), MX(1)});
	MX thrust_body = MX::vertcat(std::vector<MX>{MX(0), MX(0), -f});
	MX thrust_ned = quat_rotate_vec(q, thrust_body);

	MX dp = v_ned;
	MX dv = g * e3 + thrust_ned / m;
	MX dq = 0.5 * quat_mult(q, MX::vertcat(std::vector<MX>{MX(0), w}));
	MX dw = mtimes(inv(J), (tau - cross(w, mtimes(J, w))));

	MX state = MX::vertcat({p_ned, v_ned, q, w});
	MX input = MX::vertcat({f, tau});
	MX rhs = MX::vertcat({dp, dv, dq, dw});
	return Function("ned_x_quad_dynamics", {state, input}, {rhs});
}

// 计算MPC控制输入，输入为当前状态current_state、参考轨迹traj、参考输入u_ref、以及初始猜测u_guess和x_guess，输出为优化后的控制输入和状态轨迹
DMDict MPC::compute(const DM &current_state,
					 const DM &traj,
					 const DM &u_ref,
					 const DM &u_guess,
					 const DM &x_guess) {
	DMDict arg = {
			{"x0", vertcat(reshape(u_guess, -1, 1), reshape(x_guess, -1, 1))},
			{"p",  vertcat(current_state, reshape(traj, -1, 1), reshape(u_ref, -1, 1))},
			{"lbg", lbg},
			{"ubg", ubg}
	};
	return mpc_solver(arg);
}

// 构建一个以目标位置为中心的参考轨迹，输入为目标位置target_pos_ned，输出为一个(n_state, N+1)的矩阵，每列为一个时间步的参考状态，其中位置部分为目标位置，姿态部分为单位四元数，速度和角速度部分为零
DM MPC::build_target_traj(const DM &target_pos_ned) const {
	DM traj = DM::zeros(n_state, problem_params.N + 1);
	traj(Slice(0, 3), Slice()) = repmat(target_pos_ned, 1, problem_params.N + 1);
	traj(6, Slice()) = 1.0;
	return traj;
}

// 计算从当前状态current_state到目标位置target_pos_ned的MPC控制输入，输入为当前状态current_state、目标位置target_pos_ned、以及初始猜测u_guess和x_guess，输出为优化后的控制输入和状态轨迹
DMDict MPC::compute_to_target(const DM &current_state,
								const DM &target_pos_ned,
								const DM &u_guess,
								const DM &x_guess) {
	DM traj = build_target_traj(target_pos_ned);
	DM u_ref = DM::zeros(n_input, problem_params.N);
	u_ref(0, Slice()) = model_params.m * model_params.g;
	return compute(current_state, traj, u_ref, u_guess, x_guess);
}

// 得到控制分配矩阵的逆矩阵，输入为无人机参数ct, cm, R，输出为4x4的矩阵，用于将期望的总推力和力矩转换为每个电机的转速平方
DM MPC::get_x_mixing_matrix(double ct, double cm, double R) {
	// Motor order: [front-right, back-left, front-left, back-right].
	// Wrench = A * w2, where w2 is squared motor speed.
	const double M_23 = std::sqrt(2) / 2.0 * ct * R;
	DM A = DM::zeros(4, 4);

	A(0, 0) = ct;
	A(0, 1) = ct;
	A(0, 2) = ct;
	A(0, 3) = ct;

	A(1, 0) = -M_23;
	A(1, 1) = M_23;
	A(1, 2) = M_23;
	A(1, 3) = -M_23;

	A(2, 0) = M_23;
	A(2, 1) = -M_23;
	A(2, 2) = M_23;
	A(2, 3) = -M_23;

	A(3, 0) = cm;
	A(3, 1) = cm;
	A(3, 2) = -cm;
	A(3, 3) = -cm;

	return inv(A);
}

// 四元数乘法，输入为两个四元数q1和q2，输出为它们的乘积q1 * q2
MX MPC::quat_mult(const MX &q1, const MX &q2) {
	MX w1 = q1(0), x1 = q1(1), y1 = q1(2), z1 = q1(3);
	MX w2 = q2(0), x2 = q2(1), y2 = q2(2), z2 = q2(3);

	MX w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
	MX x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
	MX y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
	MX z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
	return vertcat(w, x, y, z);
}

// 四元数旋转向量，输入为四元数q和向量v，输出为旋转后的向量q * v * q^{-1}，其中v被视为纯四元数(0, v)
MX MPC::quat_rotate_vec(const MX &q, const MX &v) {
	MX q_conj = MX::vertcat(std::vector<MX>{q(0), -q(1), -q(2), -q(3)});
	MX v_quat = MX::vertcat(std::vector<MX>{MX(0), v(0), v(1), v(2)});
	MX v_rot = quat_mult(quat_mult(q, v_quat), q_conj);
	return v_rot(Slice(1, 4));
}

// 由四元数得到欧拉角，输入为四元数q，输出为对应的roll、pitch、yaw角，单位为弧度
MX MPC::quat_to_rpy(const MX &q) {
	MX qw = q(0), qx = q(1), qy = q(2), qz = q(3);

	MX sinr_cosp = 2.0 * (qw * qx + qy * qz);
	MX cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
	MX roll = atan2(sinr_cosp, cosr_cosp);

	MX sinp = 2.0 * (qw * qy - qz * qx);
	MX sinp_clamped = if_else(sinp > 1.0, MX(1.0), if_else(sinp < -1.0, MX(-1.0), sinp));
	MX pitch = asin(sinp_clamped);

	MX siny_cosp = 2.0 * (qw * qz + qx * qy);
	MX cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
	MX yaw = atan2(siny_cosp, cosy_cosp);

	return vertcat(roll, pitch, yaw);
}

// 四元数姿态误差代价函数，输入为当前四元数q和目标四元数q_d，输出为一个标量代价，表示两者之间的姿态误差。使用了内积的平方来消除四元数符号的二义性。
MX MPC::quat_attitude_cost(const MX &q, const MX &q_d) {
	// q and -q represent the same attitude. Use squared inner-product metric to remove sign ambiguity.
	const MX eps = MX(1e-9);
	MX q_norm = q / sqrt(dot(q, q) + eps);
	MX qd_norm = q_d / sqrt(dot(q_d, q_d) + eps);
	MX c = dot(q_norm, qd_norm);
	return 1.0 - c * c;
}

// 提供接口获取MPC问题的定义和约束边界，以便在其他模块中使用
void MPC::get_mpc_problem(MXDict &problem, std::vector<double> &lbg_out, std::vector<double> &ubg_out) const {
	problem = nlp;
	lbg_out = lbg;
	ubg_out = ubg;
}

// 提供接口设置MPC问题的定义和约束边界，以便在其他模块中使用
void MPC::set_mpc_problem(const MXDict &problem, const std::vector<double> &lbg_in, const std::vector<double> &ubg_in) {
	nlp = problem;
	lbg = lbg_in;
	ubg = ubg_in;
}

