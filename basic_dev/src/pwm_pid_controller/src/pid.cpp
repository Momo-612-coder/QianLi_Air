#ifndef _MY_PID_CONROLLER_HPP_
#define _MY_PID_CONTROLLER_HPP_

#include "pid.hpp"

Pid::Pid()
    : kp_(0.0), ki_(0.0), kd_(0.0), integral_(0.0), previous_error_(0.0), min_output_(0.0), max_output_(0.0)
{
}

Pid::Pid(double kp, double ki, double kd, double min_output, double max_output)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), previous_error_(0.0), min_output_(min_output), max_output_(max_output)
{
}

void Pid::setGains(double kp, double ki, double kd, double min_output, double max_output)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    min_output_ = min_output;
    max_output_ = max_output;
}

double Pid::updateWithLimit(double error)
{
    integral_ += error;
    if (integral_ > max_output_)
    {
        integral_ = max_output_;
    }
    else if (integral_ < min_output_)
    {
        integral_ = min_output_;
    }
    double derivative = error - previous_error_;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    previous_error_ = error;

    // 输出限幅
    if (output > max_output_)
    {
        output = max_output_;
    }
    else if (output < min_output_)
    {
        output = min_output_;
    }

    return output;
}

double Pid::updateWithLimitAndFeedforward(double error, double feedforward)
{
    integral_ += error;
    if (integral_ > max_output_)
    {
        integral_ = max_output_;
    }
    else if (integral_ < min_output_)
    {
        integral_ = min_output_;
    }
    double derivative = error - previous_error_;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative + feedforward;
    previous_error_ = error;

    // 输出限幅
    if (output > max_output_)
    {
        output = max_output_;
    }
    else if (output < min_output_)
    {
        output = min_output_;
    }

    return output;
}

#endif // _MY_PID_CONTROLLER_HPP_