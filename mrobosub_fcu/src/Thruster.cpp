#include "Thruster.hpp"

Wrench<double> Thruster::calculate_contribution(const Pose<double> &pose) {
    // TODO: figure out how to do this

    return {};
}

double Thruster::thrust_to_pwm(double thrust) {
    // TODO: deadbanding

    // Constant term minus thrust
    // So that we can solve for f(x) = 0 rather than f(x) = thrust
    double c = quad_params.c - thrust;
    // Discriminant
    double D = quad_params.b_sq - quad_params.a_4*c;

    if(D < 0) return 0;

    // TODO: verify
    double pwm;
    if(quad_params.b > 0) {
        double z = -0.5*(quad_params.b + sqrt(D));
        pwm = c / z;
    } else {
        double z = -0.5*(quad_params.b - sqrt(D));
        pwm = z / quad_params.a;
    }
        
    return int(round(pwm));
}
