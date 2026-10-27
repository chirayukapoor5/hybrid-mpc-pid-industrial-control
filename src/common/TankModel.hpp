#pragma once

class TankModel {
public:
    TankModel(double area = 1.5,
              double qInMax = 0.45,
              double outCoeff = 0.12);

    double step(double h, double qIn, double dt);
    double linearizedA(double h0, double dt) const;
    double linearizedB(double dt) const;

    double getArea() const;
    double getQInMax() const;
    double getOutCoeff() const;

private:
    double area_;
    double qInMax_;
    double outCoeff_;
};
