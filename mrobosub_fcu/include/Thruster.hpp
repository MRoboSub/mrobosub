#pragma once

#include <string>
#include <cmath>

#include <Eigen/Dense>

#include "Screw.hpp"

class Thruster {
public:
    struct QuadParams;
    struct PWMFit;

    Thruster(
        int id, Pose<double> pose, int epsilon, double zero_pwm,
        double min_neg_pwm, double min_pos_pwm, double max_neg_pwm, double max_pos_pwm,
        bool reversed, PWMFit pwm_fit, double drag=1.0
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
        pwm_fit(pwm_fit),
        contribution(calculate_contribution(pose))
        { }

    Wrench<double> get_contribution() {
        return contribution;
    }

    static Wrench<double> calculate_contribution(const Pose<double> &pose);

    int thrust_to_pwm(double thrust);
    double pwm_to_thrust(int pwm);

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
    int epsilon;
    bool reversed;
    double drag;
    double max_pos_thrust = 1;
    double max_neg_thrust = -0.8;
    double min_pos_thrust = 0;
    double min_neg_thrust = 0;
    PWMFit pwm_fit;
    Wrench<double> contribution;

    int bound_pwm(int pwm);
};