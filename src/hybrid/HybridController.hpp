#pragma once

class HybridController {
public:
    HybridController(double kp = 1.8,
                    double ki = 0.75,
                    double kd = 0.25,
                    double minOut = 0.0,
                    double maxOut = 0.45);

    void configure(double setpoint);
    double update(double measurement, double dt);

private:
    class PIDController* pid_;
    class MPCController* mpc_;
    double setpoint_;
    double previousInput_;
    bool useMpc_;
};
