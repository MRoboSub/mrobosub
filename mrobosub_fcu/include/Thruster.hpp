#pragma once

#include <string>
#include <cmath>

#include <Eigen/Dense>

#include "Screw.hpp"

class Thruster {
public:
    struct QuadParams;

    Thruster(
        int id, Pose<double> pose,
        double min_neg_pwm, double min_pos_pwm,
        bool reversed, QuadParams quad_params, double drag=1.0
    ) : 
        id(id),
        pose(pose),
        min_neg_pwm(min_neg_pwm),
        min_pos_pwm(min_pos_pwm),
        reversed(reversed),
        drag(drag),
        quad_params(quad_params),
        contribution(calculate_contribution(pose))
        { }

    Wrench<double> get_contribution() {
        return contribution;
    }

    static Wrench<double> calculate_contribution(const Pose<double> &pose);

    double thrust_to_pwm(double thrust);

    struct QuadParams { 
        const double a;
        const double b;
        const double c;

        const double a_4;
        const double b_sq;

        QuadParams(double a, double b, double c) :
            a(a), b(b), c(c), a_4(4*a), b_sq(b*b) { } 
    };

    double get_max_pos_thrust();
    double get_max_neg_thrust();

private:
    int id;
    Pose<double> pose;
    double min_neg_pwm;
    double min_pos_pwm;
    bool reversed;
    double drag;
    double max_pos_thrust = 1;
    double max_neg_thrust = -0.8;
    double min_pos_thrust = 0;
    double min_neg_thrust = 0;
    QuadParams quad_params;
    Wrench<double> contribution;
};