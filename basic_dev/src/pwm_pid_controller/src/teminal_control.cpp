#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <cerrno>

#include "teminal_control.hpp"

void print_keyboard_help() {
    std::cout
        << "\n=== Keyboard Control ===\n"
        << "R/r: Reset\n"
        << "U/u: Takeoff\n"
        << "L/l: Land\n"
        << "P/p: Toggle PWM publish\n"
        << "C/c: Toggle console output\n"
        << "W/w: Increase target x\n"
        << "S/s: Decrease target x\n"
        << "D/d: Increase target y\n"
        << "A/a: Decrease target y\n"
        << "Z/z: Increase target z\n"
        << "Space: Decrease target z\n"
        << "E/e: Increase target yaw\n"
        << "Q/q: Decrease target yaw\n"
        << "I/i: Increase Kp\n"
        << "K/k: Decrease Kp\n"
        << "Y/y: Increase Kd\n"
        << "H/h: Decrease Kd\n"
        << "1: Select roll PID controller\n"
        << "2: Select pitch PID controller\n"
        << "3: Select z PID controller\n"
        << "4: Select v_roll PID controller\n"
        << "5: Select v_pitch PID controller\n"
        << "6: Select v_z PID controller\n"
        << "O/o: Exit\n"
        << "========================\n";
}

void process_key_input(bool& need_exit) {
    if (!g_key_control_enabled) {
        return;
    }

    char buf[64] = {0};
    const ssize_t nread = read(g_keyboard_fd, buf, sizeof(buf));
    if (nread <= 0) {
        return;
    }

    static int esc_state = 0;
    for (ssize_t i = 0; i < nread; ++i) {
        const char c = buf[i];

        switch (c) {
            case 'r':
            case 'R':
                is_reset = true;
                break;
            case 'l':
            case 'L':
                is_land = true;
                break;
            case 'u':
            case 'U':
                is_takeoff = true;
                break;
            case 'p':
            case 'P':
                if(is_pwm_published) {
                    is_pwm_published = false;
                    std::cout << "PWM publish: " << (is_pwm_published ? "ON" : "OFF") << std::endl;
                } else {
                    is_pwm_published = true;
                    std::cout << "PWM publish: " << (is_pwm_published ? "ON" : "OFF") << std::endl;
                }
                break; 
            case 'c':
            case 'C':
                if(is_cout) {
                    is_cout = false;
                    print_keyboard_help();
                } else {
                    is_cout = true;
                }
                break;
            case 'w':
            case 'W':
                //target_pitch += 0.1 / 57.3;
                //std::cout << "Increase target pitch: " << target_pitch * 57.3 << " deg\n";
                target_x += 0.5;
                std::cout << "Increase target x: " << target_x << " m\n";
                break;   
            case 's':
            case 'S':
                //target_pitch -= 0.1 / 57.3;
                //std::cout << "Decrease target pitch: " << target_pitch * 57.3 << " deg\n";
                target_x -= 0.5;
                std::cout << "Decrease target x: " << target_x << " m\n";
                break;
            case 'd':
            case 'D':
                //target_roll += 0.1 / 57.3;
                //std::cout << "Increase target roll: " << target_roll * 57.3 << " deg\n";
                target_y += 0.5;
                std::cout << "Increase target y: " << target_y << " m\n";
                break;   
            case 'a':
            case 'A':
                //target_roll -= 0.1 / 57.3;
                //std::cout << "Decrease target roll: " << target_roll * 57.3 << " deg\n";
                target_y -= 0.5;
                std::cout << "Decrease target y: " << target_y << " m\n";
                break;
            case 'z':
            case 'Z':
                target_z += 0.5;
                std::cout << "Increase target z: " << target_z << " m\n";
                break;   
            case ' ':
                target_z -= 0.5;
                std::cout << "Decrease target z: " << target_z << " m\n";
                break; 
            case 'e':
            case 'E':
                target_yaw += 15.0 / 57.3;
                std::cout << "Increase target yaw: " << target_yaw * 57.3 << " deg\n";
                break;   
            case 'q':
            case 'Q':
                target_yaw -= 15.0 / 57.3;
                std::cout << "Decrease target yaw: " << target_yaw * 57.3 << " deg\n";
                break;
            case 'i':
            case 'I':
                teminal_pid_set.is_pid_changed = true;
                teminal_pid_set.kp += 0.1;
                std::cout << "Increase Kp: " << teminal_pid_set.kp << std::endl;
                break; 
            case 'k':
            case 'K':
                teminal_pid_set.is_pid_changed = true;
                teminal_pid_set.kp -= 0.1;
                std::cout << "Decrease Kp: " << teminal_pid_set.kp << std::endl;
                break;
            case 'y':
            case 'Y':
                teminal_pid_set.is_pid_changed = true;
                teminal_pid_set.kd += 0.1;
                std::cout << "Increase Kd: " << teminal_pid_set.kd << std::endl;
                break; 
            case 'h':
            case 'H':
                teminal_pid_set.is_pid_changed = true;
                teminal_pid_set.kd -= 0.1;
                std::cout << "Decrease Kd: " << teminal_pid_set.kd << std::endl;
                break;
            case '1':
                teminal_pid_set.pid_controller_index = 1;
                teminal_pid_set.kp = 0.0; // 清零或给初始值
                std::cout << "Selected PID controller: y\n";
                break;
            case '2':
                teminal_pid_set.pid_controller_index = 2;
                teminal_pid_set.kp = 0.0; // 清零或给初始值
                //std::cout << "Selected PID controller: pitch\n";
                std::cout << "Selected PID controller: x\n";
                break;
            case '3':
                teminal_pid_set.pid_controller_index = 3;
                teminal_pid_set.kp = 0.0; // 清零或给初始值
                std::cout << "Selected PID controller: yaw\n";  
                break;
            case '4':
                teminal_pid_set.pid_controller_index = 4;
                teminal_pid_set.kp = 0.0; // 清零或给初始值
                std::cout << "Selected PID controller: v_y\n";
                break;
            case '5':
                teminal_pid_set.pid_controller_index = 5;
                teminal_pid_set.kp = 0.0; // 清零或给初始值
                std::cout << "Selected PID controller: v_x\n";
                break;
            case '6':
                teminal_pid_set.pid_controller_index = 6;
                teminal_pid_set.kp = 0.0; // 清零或给初始值
                std::cout << "Selected PID controller: v_yaw\n";
                break;
            case 'o':
            case 'O':
                need_exit = true;
                break;
            default:
                break;
        }
    }
}