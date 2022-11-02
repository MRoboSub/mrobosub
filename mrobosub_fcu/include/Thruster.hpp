#pragma once

#include <string>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "Screw.hpp"

class Thruster {
public:
    struct QuadParams;
    struct PWMFit;

    Thruster(
        int id, Pose<double> pose, int epsilon, double zero_pwm,
        double min_neg_pwm, double min_pos_pwm, double max_neg_pwm, double max_pos_pwm,
        bool reversed, std::vector<PWMFit> pwm_fits, double drag=1.0
    ) : 
        id(id),
        pose(pose),
        zero_pwm(zero_pwm),
        min_neg_pwm(min_neg_pwm),
        min_pos_pwm(min_pos_pwm),
        max_neg_pwm(max_neg_pwm),
        max_pos_pwm(max_pos_pwm),
        reversed(reversed),
        drag(drag),
        pwm_fits(pwm_fits),
        contribution(calculate_contribution(pose))
        { }

    Wrench<double> get_contribution() {
        return contribution;
    }

    Wrench<double> calculate_contribution(const Pose<double> &pose);

    int thrust_to_pwm(double thrust, double voltage);
    double pwm_to_thrust(int pwm, double voltage);

    struct QuadParams { 
        const double a;
        const double b;
        const double c;

        const double a_4;
        const double b_sq;

        QuadParams(double a, double b, double c) :
            a(a), b(b), c(c), a_4(4*a), b_sq(b*b) { } 
    };

    struct PWMFit {
        double voltage;
        QuadParams forward_quad_params;
        QuadParams reverse_quad_params;
    };

    double get_max_pos_thrust();
    double get_max_neg_thrust();

private:
    int id;
    Pose<double> pose;
    double zero_pwm;
    double min_neg_pwm;
    double min_pos_pwm;
    double max_neg_pwm;
    double max_pos_pwm;
    int epsilon;
    bool reversed;
    double drag;
    double max_pos_thrust = 1.0;
    double max_neg_thrust = -0.8;
    double min_pos_thrust = 0;
    double min_neg_thrust = 0;
    std::vector<PWMFit> pwm_fits;
    Wrench<double> contribution;

    int bound_pwm(int pwm);
};