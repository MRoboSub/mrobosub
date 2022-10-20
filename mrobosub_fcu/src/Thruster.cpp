#include "Thruster.hpp"

#include <cmath>

Wrench<double> Thruster::calculate_contribution(const Pose<double> &pose) {
    // TODO: figure out how to do this

    // Note: roll should always be 0

    // TODO: paramertize, make force
    double thrust = 1;

    Eigen::Vector3d forces;

    double xy_force = thrust * cos(-pose.pitch());
    forces[0] = xy_force * cos(pose.yaw());
    forces[1] = xy_force * sin(pose.yaw());
    forces[2] = thrust * sin(-pose.pitch());

    // torques = forces x momement arms
    Eigen::Vector3d torques = forces.cross(pose.linear());

    return Wrench<double>{forces, torques};
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

double Thruster::get_max_pos_thrust() {
    return max_pos_thrust;
}

double Thruster::get_max_neg_thrust() {
    return max_neg_thrust;
}
