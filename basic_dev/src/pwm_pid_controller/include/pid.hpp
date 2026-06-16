//#ifndef _MY_PID_CONTROLLER_HPP_
//#define _MY_PID_CONTROLLER_HPP_

#include <stdlib.h>
#include "Eigen/Dense"

class Pid
{
public:
    Pid();
    Pid(double kp, double ki, double kd, double min_output, double max_output);
    void setGains(double kp, double ki, double kd, double min_output, double max_output);
    double updateWithLimit(double error); // 不考虑时间间隔
    double updateWithLimitAndFeedforward(double error, double feedforward); // 不考虑时间间隔
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double previous_error_;
    double min_output_;
    double max_output_;
};

//#endif // _MY_PID_CONTROLLER_HPP_