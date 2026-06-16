#ifndef _TEMINAL_CONTROL_HPP_
#define _TEMINAL_CONTROL_HPP_

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <chrono>
#include <cerrno>

extern bool is_takeoff;
extern bool is_land;
extern bool is_reset;
extern bool is_pwm_published;
extern bool is_cout;

// ===============================================
// ADRC 与键盘测试专用变量
// ===============================================
// 是否启用外环(位置环)。若设为 false，则直接由键盘指令接管姿态环
extern bool enable_position_loop; 
extern bool g_use_planner_input;

// 键盘控制输出的目标状态
extern double key_target_x;
extern double key_target_y;
extern double term_target_yaw;
extern double term_target_throttle; // 目标总推力 (N)
extern double term_target_z; // 目标总推力 (N)
extern double current_yaw;

extern int g_keyboard_fd;
extern bool g_key_control_enabled;

struct TerminalGuard {
    int fd;
    termios old_termios;
    int old_flags;
    bool ok;

    TerminalGuard() : fd(-1), old_flags(0), ok(false) {
        fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            if (isatty(STDIN_FILENO)) fd = STDIN_FILENO;
            else return;
        }

        if (tcgetattr(fd, &old_termios) != 0) {
            if (fd != STDIN_FILENO) { close(fd); fd = -1; }
            return;
        }
        termios new_termios = old_termios;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        new_termios.c_cc[VMIN] = 0;
        new_termios.c_cc[VTIME] = 0;
        if (tcsetattr(fd, TCSANOW, &new_termios) != 0) {
            if (fd != STDIN_FILENO) { close(fd); fd = -1; }
            return;
        }

        old_flags = fcntl(fd, F_GETFL, 0);
        if (old_flags < 0) {
            tcsetattr(fd, TCSANOW, &old_termios);
            if (fd != STDIN_FILENO) { close(fd); fd = -1; }
            return;
        }
        if (fcntl(fd, F_SETFL, old_flags | O_NONBLOCK) < 0) {
            tcsetattr(fd, TCSANOW, &old_termios);
            if (fd != STDIN_FILENO) { close(fd); fd = -1; }
            return;
        }
        ok = true;
    }

    ~TerminalGuard() {
        if (!ok) return;
        tcsetattr(fd, TCSANOW, &old_termios);
        fcntl(fd, F_SETFL, old_flags);
        if (fd != STDIN_FILENO) close(fd);
    }
};

void print_keyboard_help();
void process_key_input(bool& need_exit);

#endif
