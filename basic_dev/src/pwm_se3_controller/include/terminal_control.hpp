#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <cerrno>

extern bool is_takeoff;
extern bool is_land;
extern bool is_reset;
extern bool is_pwm_published;
extern bool is_cout;
extern bool is_manual_control;
extern double target_pitch;
extern double target_roll;
extern double target_yaw;
extern double target_x;
extern double target_y;
extern double target_z;

extern int g_keyboard_fd;
extern bool g_key_control_enabled;

struct TeminalPidSet
{
    bool is_pid_changed;
    int pid_controller_index; // 1 - pid_roll, 2 - pid_pitch, 3 - pid_yaw, 4 - pid_v_roll, 5 - pid_v_pitch, 6 - pid_v_yaw
    double kp;
    double ki;
    double kd;
};

extern TeminalPidSet teminal_pid_set;

struct TerminalGuard {
    int fd;
    termios old_termios;
    int old_flags;
    bool ok;

    TerminalGuard() : fd(-1), old_flags(0), ok(false) {
        fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            if (isatty(STDIN_FILENO)) {
                fd = STDIN_FILENO;
            } else {
                return;
            }
        }

        if (tcgetattr(fd, &old_termios) != 0) {
            if (fd != STDIN_FILENO) {
                close(fd);
                fd = -1;
            }
            return;
        }
        termios new_termios = old_termios;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        new_termios.c_cc[VMIN] = 0;
        new_termios.c_cc[VTIME] = 0;
        if (tcsetattr(fd, TCSANOW, &new_termios) != 0) {
            if (fd != STDIN_FILENO) {
                close(fd);
                fd = -1;
            }
            return;
        }

        old_flags = fcntl(fd, F_GETFL, 0);
        if (old_flags < 0) {
            tcsetattr(fd, TCSANOW, &old_termios);
            if (fd != STDIN_FILENO) {
                close(fd);
                fd = -1;
            }
            return;
        }
        if (fcntl(fd, F_SETFL, old_flags | O_NONBLOCK) < 0) {
            tcsetattr(fd, TCSANOW, &old_termios);
            if (fd != STDIN_FILENO) {
                close(fd);
                fd = -1;
            }
            return;
        }
        ok = true;
    }

    ~TerminalGuard() {
        if (!ok) {
            return;
        }
        tcsetattr(fd, TCSANOW, &old_termios);
        fcntl(fd, F_SETFL, old_flags);
        if (fd != STDIN_FILENO) {
            close(fd);
        }
    }
};

void print_keyboard_help();
void process_key_input(bool& need_exit);