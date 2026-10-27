#pragma once

class PIDController {
public:
    PIDController(double kp, double ki, double kd,
                  double minOut, double maxOut);

    double update(double setpoint, double measurement, double dt);
    void reset();

private:
    double kp_;
    double ki_;
    double kd_;
    double minOut_;
    double maxOut_;
    double integral_;
    double prevError_;
    bool initialized_;
};
