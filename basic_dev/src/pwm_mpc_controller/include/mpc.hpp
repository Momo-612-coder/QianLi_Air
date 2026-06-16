#ifndef MPC_HPP
#define MPC_HPP

#include <map>
#include <string>
#include <vector>
#include <cmath>

#include <casadi/casadi.hpp>

using namespace casadi;

// State (NED): [pn, pe, pd, vn, ve, vd, qw, qx, qy, qz, p, q, r]
// Input (body wrench): [F, T_x, T_y, T_z]
struct problem_params_t {
		// 不要在这里修改参数，修改请在 pwm_mpc_controller.cpp 的 PwmMpcController::PwmMpcController() 函数中修改
		// 这里只是提供默认值，实际使用时会被覆盖
		double ts = 0.02;
		int N = 30; // 预测步数
		DM R = DM::diag({2.0, 1.0, 1.0, 1.0}); // 输入权重，单位为N^2和N*m^2
		DM Q = DM::diag({8.0, 8.0, 5.0,
						 1.0, 1.0, 1.5,
						 0.8, 0.8, 0.8, 0.8,
						 0.1, 0.1, 0.1}); // 状态权重
		DM P = DM::diag({10.0, 10.0, 12.0,
						 1.2, 1.2, 1.8,
						 1.0, 1.0, 1.0, 1.0,
						 0.15, 0.15, 0.15}); // 终端状态权重
		double q_att_weight = 5.0; // 姿态误差权重
		double q_att_terminal_weight = 10.0; // 终端姿态误差权重
		double max_roll_pitch = 30.0 / 57.3; // 最大滚转和俯仰角，单位为弧度
		double max_yaw_rate = 100 / 57.3;  // 最大偏航角速度，单位为rad/s
		double max_F = 12.0; // 最大总推力，单位为N
		double min_F = 0.0; // 最小总推力，单位为N
		double max_Tx = 1.0; // 最大滚转力矩，单位为N*m
		double max_Ty = 1.0; // 最大俯仰力矩，单位为N*m
		double max_Tz = 0.5; // 最大偏航力矩，单位为N*m
	};

struct model_params_t {
	double g = 9.81;     // NED uses +z down
	double m = 0.648318; // kg
	double ct = 0.000367717 * (1.0 / 2.0 / M_PI) * (1.0 / 2.0 / M_PI); // 电机升力系数，单位为N/(rad/s)^2
	double cm = 4.888486266072161e-06 * (1.0 / 2.0 / M_PI) * (1.0 / 2.0 / M_PI); // 电机反扭矩系数，单位为N*m/(rad/s)^2
	//double ct = 0.000367717; // 电机升力系数，单位为N/(rad/s)^2
	//double cm = 4.888486266072161e-06; // 电机反扭矩系数，单位为N*m/(rad/s)^2
	double R = 0.18;
	double max_n = 11079.03; // max motor speed r/min
	double min_w = 0.0; 
	double max_w = max_n * 2.0 * M_PI / 60.0; // rad/s
	double Jxx = 0.0046890742;
	double Jyy = 0.0069312;
	double Jzz = 0.010421166;
};

class MPC
{
public:
    MPC();
    ~MPC();

    void init_mpc_problem(const problem_params_t& problem_params, const model_params_t& model_params);
    void init_solver(std::string solver_type = "ipopt",
					 casadi::Dict solver_opts = {
						{"print_time", true},
						{"ipopt.print_level", 5},
						{"ipopt.max_iter", 100},
						{"ipopt.tol", 1e-4},
						{"ipopt.acceptable_tol", 1e-3},
						{"ipopt.warm_start_init_point", "yes"}
					 });
    Function ned_x_quad_dynamics(model_params_t params);
    DMDict compute(const DM &current_state,
					 const DM &traj,
					 const DM &u_ref,
					 const DM &u_guess,
					 const DM &x_guess);
    DM build_target_traj(const DM &target_pos_ned) const;
    DMDict compute_to_target(const DM &current_state,
								const DM &target_pos_ned,
								const DM &u_guess,
								const DM &x_guess);
    DM get_x_mixing_matrix(double ct, double cm, double R);
    MX quat_mult(const MX &q1, const MX &q2);
    MX quat_rotate_vec(const MX &q, const MX &v);
    MX quat_to_rpy(const MX &q);
    MX quat_attitude_cost(const MX &q, const MX &q_d);
    void get_mpc_problem(MXDict &problem, std::vector<double> &lbg_out, std::vector<double> &ubg_out) const;
    void set_mpc_problem(const MXDict &problem, const std::vector<double> &lbg_in, const std::vector<double> &ubg_in);

    const int n_state = 13;
	const int n_input = 4;

    problem_params_t problem_params;
	model_params_t model_params;
	std::vector<double> lbg{}, ubg{};
	MXDict nlp;
	Function mpc_solver;

};


#endif // MPC_HPP