#include "controllerTest.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <cerrno>

namespace {

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

int g_keyboard_fd = -1;
bool g_key_control_enabled = false;
float g_des_x = 0.0f;
float g_des_y = 0.0f;
float g_des_z = -0.5f;
bool g_des_initialized = false;
float g_cmd_vx = 0.0f;
float g_cmd_vy = 0.0f;
float g_cmd_vz = 0.0f;
ros::Time g_last_move_cmd_time(0.0);
ros::Time g_last_des_update_time(0.0);

void print_keyboard_help() {
    std::cout
        << "\n=== Keyboard Control ===\n"
        << "Hold Arrow / WASD: continuous move in XY\n"
        << "Hold R/F: continuous move up/down (Z)\n"
        << "Release key: auto-hover\n"
        << "Space: force hover\n"
        << "L: toggle logs on/off\n"
        << "Q: quit\n"
        << "========================\n";
}

void process_key_input(bool& need_exit) {
    if (!g_key_control_enabled) {
        return;
    }

    constexpr float kMoveSpeedXY = 0.8f;
    constexpr float kMoveSpeedZ = 0.6f;

    char buf[64] = {0};
    const ssize_t nread = read(g_keyboard_fd, buf, sizeof(buf));
    if (nread <= 0) {
        return;
    }

    static int esc_state = 0;
    for (ssize_t i = 0; i < nread; ++i) {
        const char c = buf[i];

        if (esc_state == 1) {
            if (c == '[') {
                esc_state = 2;
                continue;
            }
            esc_state = 0;
        } else if (esc_state == 2) {
            switch (c) {
                case 'A':
                    g_cmd_vx = kMoveSpeedXY;
                    g_cmd_vy = 0.0f;
                    g_cmd_vz = 0.0f;
                    g_last_move_cmd_time = ros::Time::now();
                    break;
                case 'B':
                    g_cmd_vx = -kMoveSpeedXY;
                    g_cmd_vy = 0.0f;
                    g_cmd_vz = 0.0f;
                    g_last_move_cmd_time = ros::Time::now();
                    break;
                case 'C':
                    g_cmd_vx = 0.0f;
                    g_cmd_vy = -kMoveSpeedXY;
                    g_cmd_vz = 0.0f;
                    g_last_move_cmd_time = ros::Time::now();
                    break;
                case 'D':
                    g_cmd_vx = 0.0f;
                    g_cmd_vy = kMoveSpeedXY;
                    g_cmd_vz = 0.0f;
                    g_last_move_cmd_time = ros::Time::now();
                    break;
                default:
                    break;
            }
            esc_state = 0;
            continue;
        }

        if (c == '\x1b') {
            esc_state = 1;
            continue;
        }

        switch (c) {
            case 'w':
            case 'W':
                g_cmd_vx = kMoveSpeedXY;
                g_cmd_vy = 0.0f;
                g_cmd_vz = 0.0f;
                g_last_move_cmd_time = ros::Time::now();
                break;
            case 's':
            case 'S':
                g_cmd_vx = -kMoveSpeedXY;
                g_cmd_vy = 0.0f;
                g_cmd_vz = 0.0f;
                g_last_move_cmd_time = ros::Time::now();
                break;
            case 'a':
            case 'A':
                g_cmd_vx = 0.0f;
                g_cmd_vy = kMoveSpeedXY;
                g_cmd_vz = 0.0f;
                g_last_move_cmd_time = ros::Time::now();
                break;
            case 'd':
            case 'D':
                g_cmd_vx = 0.0f;
                g_cmd_vy = -kMoveSpeedXY;
                g_cmd_vz = 0.0f;
                g_last_move_cmd_time = ros::Time::now();
                break;
            case 'r':
            case 'R':
                g_cmd_vx = 0.0f;
                g_cmd_vy = 0.0f;
                g_cmd_vz = kMoveSpeedZ;
                g_last_move_cmd_time = ros::Time::now();
                break;
            case 'f':
            case 'F':
                g_cmd_vx = 0.0f;
                g_cmd_vy = 0.0f;
                g_cmd_vz = -kMoveSpeedZ;
                g_last_move_cmd_time = ros::Time::now();
                break;
            case ' ':
                g_cmd_vx = 0.0f;
                g_cmd_vy = 0.0f;
                g_cmd_vz = 0.0f;
                break;
            case 'q':
            case 'Q':
                need_exit = true;
                break;
            case 'l':
            case 'L':
                g_controller_debug_log_enabled = !g_controller_debug_log_enabled;
                if (g_controller_debug_log_enabled) {
                    ROS_INFO("Debug logs enabled");
                } else {
                    ROS_INFO("Debug logs muted");
                }
                break;
            default:
                break;
        }
    }
}

}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_test"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle n; // 创建node控制句柄
    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    g_takeoff_client = n.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    g_pwm_publisher = n.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 1);
    ros::Subscriber odom_suber = n.subscribe<nav_msgs::Odometry>("/eskf_odom", 1, odom_cb);
    // ros::Subscriber gt_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, gt_cb);
    ros::Subscriber init_pose_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/initial_pose", 1, init_pose_cb);
    ros::Subscriber end_pose_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/end_goal", 1, end_position_cb);
    airsim_ros::Takeoff  tf_cmd;
    tf_cmd.request.waitOnLastTask = 1;
    // g_takeoff_client.call(tf_cmd);

    get_init_pose = false;
    get_end_goal = false;

    TerminalGuard terminal_guard;
    g_key_control_enabled = terminal_guard.ok;
    g_keyboard_fd = terminal_guard.fd;
    if (g_key_control_enabled) {
        print_keyboard_help();
    } else {
        ROS_WARN("Keyboard control disabled: failed to configure terminal (/dev/tty or stdin).\n");
    }

    ros::Rate loop_rate(200);
    bool need_exit = false;
    while(ros::ok()){
        ros::spinOnce();
        process_key_input(need_exit);
        if (need_exit) {
            ROS_INFO("Exit requested by keyboard (Q).\n");
            break;
        }
        loop_rate.sleep();
    }
    return 0;
}

void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaternion Q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Matrix3d rotationM = Q.normalized().toRotationMatrix();
    // std::cout<<"initial pose: \n"<<rotationM<<std::endl;
    Eigen::Vector3d pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    // std::cout<<"initial pos: "<<pos.transpose()<<std::endl;
    Tw0 = Eigen::Matrix4d::Identity();
    Tw0.block(0, 0, 3, 3)=rotationM;
    Tw0.block(0, 3, 3 ,1) = pos;
    Twb_last = Tw0;
    // std::cout<<"Tw0:\n"<<Tw0<<std::endl;
    get_init_pose = true;
}

void end_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Pwend = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    // std::cout<<"end pos: "<<Pwend.transpose()<<std::endl;
    if(get_init_pose && !get_end_goal)
    {
        for(auto ps: globalPaths)
        {
            if((ps[0]-Tw0.block(0, 3, 3, 1)).norm() < 10)
            {
                for(int i = 0; i < ps.size(); i++)
                {
                    globalPath.emplace_back(ps[i]);
                }
                break;
            }
        }
        for(auto ps: globalPaths)
        {
            if((ps[0]-Pwend).norm() < 10)
            {
                for(int i = 0; i < ps.size(); i++)
                {
                    globalPath.emplace_back(ps[ps.size()-i]);
                }
                break;
            }
        }
        get_end_goal = true;
    }
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    cb_cnt ++;
    if(cb_cnt / 100 < 10)return;
    if(! get_init_pose)return;
    Eigen::Quaternion Q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
    Twb.block(0, 0, 3 ,3) = Q.normalized().toRotationMatrix();
    Twb(0, 3) = msg->pose.pose.position.x;
    Twb(1, 3) = msg->pose.pose.position.y;
    Twb(2, 3) = msg->pose.pose.position.z;
    // std::cout<<"Twb rt:\n"<<Twb<<std::endl;
    // std::cout<<phi<<" "<<theta<<" "<<psi<<std::endl;
    Eigen::VectorXf X_des, X_real;
    X_des.resize(12);
    X_real.resize(12);
    // Twb_last = Twb;
    // std::cout<<"Tw0:\n"<<Tw0<<std::endl;
    Eigen::Matrix4d TWfluWned;
    TWfluWned << 1, 0, 0, 0, 
                0, -1, 0, 0,
                0, 0, -1, 0, 
                0, 0, 0, 1;
    Eigen::Matrix4d TWflu0 = TWfluWned * Tw0 * TWfluWned.inverse();
    Eigen::Matrix4d TWflub = TWfluWned * Twb * TWfluWned.inverse();
    Eigen::Matrix4d T0flub = TWflu0.inverse() * TWflub;
    Eigen::Vector3d VWned(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Vector3d VBned = Twb.block(0, 0, 3 ,3).inverse() * VWned;
    Eigen::Vector3d VBflu = TWfluWned.block<3, 3>(0, 0) * VBned;
    Eigen::Vector3d Wned(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    Eigen::Vector3d Wflu = TWfluWned.block<3, 3>(0, 0) * Wned;
    const float phi = std::asin(T0flub(2, 1));
    const float theta = std::atan2(-T0flub(2, 0)/std::cos(phi), T0flub(2, 2)/std::cos(phi));
    const float psi = std::atan2(-T0flub(0, 1)/std::cos(phi), T0flub(1, 1)/std::cos(phi));

    if (!g_des_initialized) {
        g_des_x = static_cast<float>(T0flub(0, 3));
        g_des_y = static_cast<float>(T0flub(1, 3));
        g_des_z = static_cast<float>(T0flub(2, 3));
        g_des_initialized = true;
        g_last_des_update_time = ros::Time::now();
    }

    constexpr double kCmdHoldTimeout = 0.60;
    const ros::Time now = ros::Time::now();
    if (g_last_move_cmd_time.isZero() || (now - g_last_move_cmd_time).toSec() > kCmdHoldTimeout) {
        g_cmd_vx = 0.0f;
        g_cmd_vy = 0.0f;
        g_cmd_vz = 0.0f;
    }

    double dt = 0.0;
    if (!g_last_des_update_time.isZero()) {
        dt = (now - g_last_des_update_time).toSec();
        if (dt < 0.0) {
            dt = 0.0;
        } else if (dt > 0.05) {
            dt = 0.05;
        }
    }
    g_last_des_update_time = now;

    g_des_x += static_cast<float>(g_cmd_vx * dt);
    g_des_y += static_cast<float>(g_cmd_vy * dt);
    g_des_z += static_cast<float>(g_cmd_vz * dt);
    g_des_z = std::max(-3.0f, std::min(0.0f, g_des_z));

    X_real<<T0flub(0, 3), T0flub(1, 3), T0flub(2, 3), 
        VBflu.x(), VBflu.y(), VBflu.z(), 
        phi, theta, psi, Wflu.x(), Wflu.y(), Wflu.z();
    X_des << g_des_x, g_des_y, g_des_z, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Eigen::Vector4f output = g_PDcontroller.execute( X_des, X_real);
    airsim_ros::RotorPWM pwm_cmd;
    pwm_cmd.rotorPWM0 = output[0];
    pwm_cmd.rotorPWM1 = output[1];
    pwm_cmd.rotorPWM2 = output[2];
    pwm_cmd.rotorPWM3 = output[3];
    if (g_controller_debug_log_enabled && cb_cnt % 100 == 0) {
        std::cout
            << "des(x,y,z)=(" << g_des_x << ", " << g_des_y << ", " << g_des_z << ") "
            << "real(x,y,z)=(" << X_real[0] << ", " << X_real[1] << ", " << X_real[2] << ") "
            << "cmd(vx,vy,vz)=(" << g_cmd_vx << ", " << g_cmd_vy << ", " << g_cmd_vz << ")"
            << std::endl;
    }
    g_pwm_publisher.publish(pwm_cmd);

}
