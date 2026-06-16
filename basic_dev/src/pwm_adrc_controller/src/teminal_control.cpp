#include "teminal_control.hpp"
#include <Eigen/Dense>   
#include <cmath>
#include <iomanip> 
#include <algorithm>

// 基础控制外部变量
extern double key_target_x;
extern double key_target_y;
extern double term_target_yaw;
extern double term_target_z; 
extern bool is_flying; 
extern bool enable_position_loop;
extern bool g_use_planner_input;
extern bool g_attitude_lock_enabled;

// ⭐ 引入姿态模式控制变量
extern double term_target_roll;
extern double term_target_pitch;
extern double current_yaw;

// 调参外部变量声明
extern Eigen::Vector3d g_tune_kp_pos;
extern Eigen::Vector3d g_tune_wc_vel;
extern Eigen::Vector3d g_tune_wp_vel;
extern Eigen::Vector3d g_tune_kp_ang;
extern Eigen::Vector3d g_tune_wc_att;
extern Eigen::Vector3d g_tune_wp_att;
extern Eigen::Vector3d g_base_b0;
extern Eigen::Vector3d g_tune_b0_scale;
extern Eigen::Vector3d g_tune_b0;
extern double g_tune_tau;

extern int g_tuning_param_idx;
extern int g_tuning_axis_idx;

const char* param_names[] = {
    "None", 
    "Outer_Kp", "Outer_Wc", "Outer_Wp", 
    "Inner_Kp", "Inner_Wc", "Inner_Wp", 
    "b0_Comp", "Tau"
};
const char* axis_names[] = {"[X-Roll] ", "[Y-Pitch]", "[Z-Yaw]  ", "[Global] "};

void print_keyboard_help() {
    std::cout
        << "\n=== ADRC Pro Teleop & Matrix Tuning ===\n"
        << "U/u: Takeoff (Enter Flight Mode)\n"
        << "M/m: Land\n"
        << "R/r: Reset -> Forced to (0,0,0), input source unchanged\n"
        << "1: Attitude Mode | 2: Position Mode\n"
        << "T/t: Toggle Input Source (Planner / Keyboard)\n"
        << "P/p: Toggle PWM (ARM/DISARM)\n"
        << "Shift+I/J/K/L: ATTI lock 45 deg, press same shifted key again to level\n"
        << "\n=== Axis-Independent Tuning ===\n"
        << "1. Select Param : [3] to [9], [0] for Tau\n"
        << "2. Select Axis  : [X], [Y], [Z], or [G] for Global(All)\n"
        << "3. Adjust Value : [ -> Decrease | ] -> Increase\n"
        << "b0 tuning uses scale factor, step = 0.1x\n"
        << "\n=== Flight Controls ===\n"
        << "I/K/J/L: ATTI (Tilt) / POS (Move XY)\n"
        << "POS Shift+I/K: 10 m forward/back, Shift+J/L: 5 m left/right\n"
        << "Shift+A/D: ATTI yaw 60 deg command\n"
        << "W/S: Up / Down (Z)  |  A/D: Yaw CCW / CW\n"
        << "O/o: Exit\n"
        << "=======================================\n";
}

void process_key_input(bool& need_exit) {
    if (!g_key_control_enabled) return;

    // 姿态模式默认是弹簧摇杆；锁定姿态时保持上一次 Shift+I/J/K/L 的目标角。
    if (!enable_position_loop && !g_attitude_lock_enabled) {
        term_target_roll = 0.0;
        term_target_pitch = 0.0;
    }

    char buf[64] = {0};
    const ssize_t nread = read(g_keyboard_fd, buf, sizeof(buf));
    
    if (nread <= 0) return; 

    const double xy_step = 0.5;  
    const double pos_big_forward_step = 10.0;
    const double pos_big_side_step = 5.0;
    const double yaw_step = 5.0 * M_PI / 180.0;
    const double yaw_fast_step = 60.0 * M_PI / 180.0;
    const double z_step = 0.5; 
    const double tilt_angle = 45.0 * M_PI / 180.0;
    const double locked_tilt_angle = 45.0 * M_PI / 180.0;
    
    bool target_changed = false;

    auto adjust_param = [&](Eigen::Vector3d& param, double step) {
        if (g_tuning_axis_idx == 0 || g_tuning_axis_idx == 3) param.x() = std::max(0.01, param.x() + step);
        if (g_tuning_axis_idx == 1 || g_tuning_axis_idx == 3) param.y() = std::max(0.01, param.y() + step);
        if (g_tuning_axis_idx == 2 || g_tuning_axis_idx == 3) param.z() = std::max(0.01, param.z() + step);
    };

    auto adjust_b0_scale = [&](double step) {
        if (g_tuning_axis_idx == 0 || g_tuning_axis_idx == 3) g_tune_b0_scale.x() = std::max(0.1, g_tune_b0_scale.x() + step);
        if (g_tuning_axis_idx == 1 || g_tuning_axis_idx == 3) g_tune_b0_scale.y() = std::max(0.1, g_tune_b0_scale.y() + step);
        if (g_tuning_axis_idx == 2 || g_tuning_axis_idx == 3) g_tune_b0_scale.z() = std::max(0.1, g_tune_b0_scale.z() + step);
        g_tune_b0 = g_base_b0.cwiseProduct(g_tune_b0_scale);
    };

    auto reset_attitude_lock = [&]() {
        g_attitude_lock_enabled = false;
        term_target_roll = 0.0;
        term_target_pitch = 0.0;
    };

    auto set_or_toggle_attitude_lock = [&](double roll, double pitch, const char* label) {
        const bool same_target =
            g_attitude_lock_enabled &&
            std::abs(term_target_roll - roll) < 1e-6 &&
            std::abs(term_target_pitch - pitch) < 1e-6;
        if (same_target) {
            reset_attitude_lock();
            std::cout << "\n[ATTI Lock: level]\n";
        } else {
            g_attitude_lock_enabled = true;
            term_target_roll = roll;
            term_target_pitch = pitch;
            std::cout << "\n[ATTI Lock: " << label << " 45 deg]\n";
        }
    };

    for (ssize_t i = 0; i < nread; ++i) {
        const char c = buf[i];
        switch (c) {
            case '1':
                enable_position_loop = false;
                reset_attitude_lock();
                std::cout << "\n[Mode: Attitude (ATTI)]\n";
                break;
            case '2':
                enable_position_loop = true;
                reset_attitude_lock();
                std::cout << "\n[Mode: Position (POS)]\n";
                break;
            case 't': case 'T':
                g_use_planner_input = !g_use_planner_input;
                if (g_use_planner_input) enable_position_loop = true;
                reset_attitude_lock();
                std::cout << "\n[Input: " << (g_use_planner_input ? "Planner" : "Keyboard") << "]\n";
                target_changed = true;
                break;
            case 'r': case 'R': 
                is_reset = true; 
                is_pwm_published = false; 
                is_flying = false; 
                enable_position_loop = false; // 复位默认姿态模式
                reset_attitude_lock();
                std::cout << "\n>> RESET: System to Origin. Input source unchanged <<\n";
                break;
            case 'm': case 'M': is_land = true; break;
            case 'p': case 'P':
                is_pwm_published = !is_pwm_published;
                if(!is_pwm_published) {
                    is_flying = false;
                    reset_attitude_lock();
                }
                std::cout << "\nPWM: " << (is_pwm_published ? "ON (IDLE)" : "OFF") << "\n";
                break; 
            case 'u': case 'U': is_takeoff = true; is_flying = true; break;

            // ⭐ 核心分流逻辑：按键在两种模式下的不同作用
            case 'i':
                if(!is_flying) break; 
                if(enable_position_loop) { key_target_x += xy_step * cos(current_yaw); key_target_y += xy_step * sin(current_yaw); }
                else { reset_attitude_lock(); term_target_pitch = -tilt_angle; } // 姿态前倾
                target_changed = true; break;
            case 'I':
                if(!is_flying) break; 
                if(enable_position_loop) { key_target_x += pos_big_forward_step * cos(current_yaw); key_target_y += pos_big_forward_step * sin(current_yaw); }
                else { set_or_toggle_attitude_lock(0.0, -locked_tilt_angle, "forward"); }
                target_changed = true; break;
            case 'k':
                if(!is_flying) break; 
                if(enable_position_loop) { key_target_x -= xy_step * cos(current_yaw); key_target_y -= xy_step * sin(current_yaw); }
                else { reset_attitude_lock(); term_target_pitch = tilt_angle; } // 姿态后仰
                target_changed = true; break;
            case 'K':
                if(!is_flying) break; 
                if(enable_position_loop) { key_target_x -= pos_big_forward_step * cos(current_yaw); key_target_y -= pos_big_forward_step * sin(current_yaw); }
                else { set_or_toggle_attitude_lock(0.0, locked_tilt_angle, "back"); }
                target_changed = true; break;
            case 'j':
                if(!is_flying) break; 
                if(enable_position_loop) { key_target_x += xy_step * sin(current_yaw); key_target_y -= xy_step * cos(current_yaw); }
                else { reset_attitude_lock(); term_target_roll = -tilt_angle; } // 姿态左倾
                target_changed = true; break;
            case 'J':
                if(!is_flying) break; 
                if(enable_position_loop) { key_target_x += pos_big_side_step * sin(current_yaw); key_target_y -= pos_big_side_step * cos(current_yaw); }
                else { set_or_toggle_attitude_lock(-locked_tilt_angle, 0.0, "left"); }
                target_changed = true; break;
            case 'l':
                if(!is_flying) break; 
                if(enable_position_loop) { key_target_x -= xy_step * sin(current_yaw); key_target_y += xy_step * cos(current_yaw); }
                else { reset_attitude_lock(); term_target_roll = tilt_angle; } // 姿态右倾
                target_changed = true; break;
            case 'L':
                if(!is_flying) break; 
                if(enable_position_loop) { key_target_x -= pos_big_side_step * sin(current_yaw); key_target_y += pos_big_side_step * cos(current_yaw); }
                else { set_or_toggle_attitude_lock(locked_tilt_angle, 0.0, "right"); }
                target_changed = true; break;

            case 'w': case 'W': is_flying = true; term_target_z -= z_step; target_changed = true; break;
            case 's': case 'S': if(!is_flying) break; term_target_z += z_step; target_changed = true; break;
            case 'a':
                term_target_yaw -= yaw_step;
                target_changed = true;
                break;
            case 'A':
                term_target_yaw -= enable_position_loop ? yaw_step : yaw_fast_step;
                target_changed = true;
                break;
            case 'd':
                term_target_yaw += yaw_step;
                target_changed = true;
                break;
            case 'D':
                term_target_yaw += enable_position_loop ? yaw_step : yaw_fast_step;
                target_changed = true;
                break;
            
            // 矩阵调参
            case '3': g_tuning_param_idx = 1; target_changed = true; break;
            case '4': g_tuning_param_idx = 2; target_changed = true; break;
            case '5': g_tuning_param_idx = 3; target_changed = true; break;
            case '6': g_tuning_param_idx = 4; target_changed = true; break;
            case '7': g_tuning_param_idx = 5; target_changed = true; break;
            case '8': g_tuning_param_idx = 6; target_changed = true; break;
            case '9': g_tuning_param_idx = 7; target_changed = true; break;
            case '0': g_tuning_param_idx = 8; target_changed = true; break;
            case 'x': case 'X': g_tuning_axis_idx = 0; target_changed = true; break;
            case 'y': case 'Y': g_tuning_axis_idx = 1; target_changed = true; break;
            case 'z': case 'Z': g_tuning_axis_idx = 2; target_changed = true; break;
            case 'g': case 'G': g_tuning_axis_idx = 3; target_changed = true; break;
            case '[': case ']': {
                double step = (c == '[') ? -0.1 : 0.1;
                if (g_tuning_param_idx == 1) adjust_param(g_tune_kp_pos, step);
                if (g_tuning_param_idx == 2) adjust_param(g_tune_wc_vel, step);
                if (g_tuning_param_idx == 3) adjust_param(g_tune_wp_vel, step);
                if (g_tuning_param_idx == 4) adjust_param(g_tune_kp_ang, step);
                if (g_tuning_param_idx == 5) adjust_param(g_tune_wc_att, step);
                if (g_tuning_param_idx == 6) adjust_param(g_tune_wp_att, step);
                if (g_tuning_param_idx == 7) adjust_b0_scale(step);
                if (g_tuning_param_idx == 8) g_tune_tau = std::max(0.01, g_tune_tau + step);
                target_changed = true;
                break;
            }
            case 'o': case 'O': need_exit = true; break;
            default: break;
        }
    }

    if (target_changed) {
        Eigen::Vector3d* cur_vec = nullptr;
        if (g_tuning_param_idx == 1) cur_vec = &g_tune_kp_pos;
        if (g_tuning_param_idx == 2) cur_vec = &g_tune_wc_vel;
        if (g_tuning_param_idx == 3) cur_vec = &g_tune_wp_vel;
        if (g_tuning_param_idx == 4) cur_vec = &g_tune_kp_ang;
        if (g_tuning_param_idx == 5) cur_vec = &g_tune_wc_att;
        if (g_tuning_param_idx == 6) cur_vec = &g_tune_wp_att;
        if (g_tuning_param_idx == 7) cur_vec = &g_tune_b0_scale;

        std::cout << "\r[" << (enable_position_loop ? "POS" : (g_attitude_lock_enabled ? "ATT-LOCK" : "ATT"))
                  << "/" << (g_use_planner_input ? "PLAN" : "KEY")
                  << "] Z:" << std::fixed << std::setprecision(1) << term_target_z 
                  << (is_flying ? " FLY" : " IDL") 
                  << " | TUNE: " << axis_names[g_tuning_axis_idx] << param_names[g_tuning_param_idx];
                  
        if (g_tuning_param_idx == 8) {
            std::cout << " -> " << std::setprecision(2) << g_tune_tau << " s      " << std::flush;
        } else if (cur_vec != nullptr) {
            std::cout << (g_tuning_param_idx == 7 ? " scale" : "")
                      << " -> X:" << std::setprecision(2) << cur_vec->x()
                      << " Y:" << cur_vec->y()
                      << " Z:" << cur_vec->z() << "      " << std::flush;
        } else {
            std::cout << "                      " << std::flush;
        }
    }
}
