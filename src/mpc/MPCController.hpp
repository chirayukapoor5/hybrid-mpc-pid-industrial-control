
#pragma once

class MPCController {
public:
    MPCController(
        double dt,
        int horizon,
        double qWeight,
        double rWeight,
        double deltaWeight
    );

    void configure(double uMin, double uMax);

    double compute(
        double currentHeight,
        double setpoint,
        double previousInput
    );

private:
    double dt_;
    int horizon_;

    double qWeight_;
    double rWeight_;
    double deltaWeight_;

    double uMin_;
    double uMax_;
};
