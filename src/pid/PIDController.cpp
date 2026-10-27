#include "pid/PIDController.hpp"

#include <algorithm>

PIDController::PIDController(double kp, double ki, double kd,
                             double minOut, double maxOut)
    : kp_(kp), ki_(ki), kd_(kd),
      minOut_(minOut), maxOut_(maxOut),
      integral_(0.0), prevError_(0.0), initialized_(false) {}

double PIDController::update(double setpoint, double measurement, double dt) {
    const double error = setpoint - measurement;

    if (!initialized_) {
        prevError_ = error;
        initialized_ = true;
    }

    integral_ += error * dt;
    const double derivative = (error - prevError_) / dt;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    prevError_ = error;
    output = std::clamp(output, minOut_, maxOut_);
    return output;
}

void PIDController::reset() {
    integral_ = 0.0;
    prevError_ = 0.0;
    initialized_ = false;
}
