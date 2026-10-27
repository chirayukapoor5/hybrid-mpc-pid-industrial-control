#include "hybrid/HybridController.hpp"

#include <algorithm>
#include <cmath>

HybridController::HybridController(double kp, double ki, double kd,
                                   double minOut, double maxOut)
    : pid_(kp, ki, kd, minOut, maxOut),
      mpc_(0.5, 5, 12.0, 0.8, 0.4),
      setpoint_(0.55),
      previousInput_(0.2),
      useMpc_(false) {
    mpc_.configure(minOut, maxOut);
}

void HybridController::configure(double setpoint) {
    setpoint_ = setpoint;
}

double HybridController::update(double measurement, double dt) {
    const double trackingError = std::abs(setpoint_ - measurement);
    useMpc_ = trackingError > 0.10;

    double u = 0.0;
    if (useMpc_) {
        u = mpc_.compute(measurement, setpoint_, previousInput_);
    } else {
        u = pid_.update(setpoint_, measurement, dt);
    }

    previousInput_ = std::clamp(u, 0.0, 0.45);
    return previousInput_;
}
