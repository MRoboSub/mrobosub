#pragma once

#include <string>
#include <cmath>

#include <Eigen/Dense>

#include "Screw.hpp"

class Thruster {
public:
    struct QuadParams;

    Thruster(
        int id, Pose pose, float min_neg_pwm, float min_pos_pwm, bool reversed, QuadParams quad_params, float drag=1.0
    ) : 
        pose(pose),
        id(id),
        min_neg_pwm(min_neg_pwm),
        min_pos_pwm(min_pos_pwm),
        reversed(reversed),
        drag(drag),
        quad_params(quad_params),
        contribution(calculate_contribution(pose))
        { }

    static Wrench calculate_contribution(const Pose &pose)
    {
        // TODO: figure out how to do this
    }

    float thrust_to_pwm(float thrust) {
        // TODO: deadbanding

        // Constant term minus thrust
        // So that we can solve for f(x) = 0 rather than f(x) = thrust
        float c = quad_params.c - thrust;
        // Discriminant
        float D = quad_params.b_sq - quad_params.a_4*c;

        if(D < 0) return 0;

        // TODO: verify
        float pwm;
        if(quad_params.b > 0) {
            float z = -0.5*(quad_params.b + sqrt(D));
            pwm = c / z;
        } else {
            float z = -0.5*(quad_params.b - sqrt(D));
            pwm = z / quad_params.a;
        }
            
        return int(round(pwm));
    }

    struct QuadParams { 
        const float a;
        const float b;
        const float c;

        const float a_4;
        const float b_sq;

        QuadParams(float a, float b, float c) :
            a(a), b(b), c(c), a_4(4*a), b_sq(b*b) { } 
    };

private:
    int id;
    Pose pose;
    float min_neg_pwm;
    float min_pos_pwm;
    bool reversed;
    float drag;
    float max_pos_thrust;
    float max_neg_thrust;
    float min_pos_thrust;
    float min_neg_thrust;
    QuadParams quad_params;
    Wrench contribution;
};