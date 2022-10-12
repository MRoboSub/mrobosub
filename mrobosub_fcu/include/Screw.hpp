#pragma once

#include <Eigen/Dense>

using Vector6 = Eigen::Matrix<float, 6, 1>;
using Vector3 = Eigen::Matrix<float, 3, 1>;

class Screw {
public:
    Screw() { }
    Screw(float surge, float sway, float heave,
            float roll, float pitch, float yaw)
    {
        screw << surge, sway, heave, roll, pitch, yaw;
    }

    Vector6 as_vector6() {
        return screw;
    }

    Vector3 linear() {
        return Vector3(screw[0], screw[1], screw[2]);
    }

    Vector3 angular() {
        return Vector3(screw[3], screw[4], screw[5]);
    }

    float surge() { return screw[0]; }
    float sway()  { return screw[1]; }
    float heave() { return screw[2]; }
    float roll()  { return screw[3]; }
    float pitch() { return screw[4]; }
    float yaw()   { return screw[5]; }

private:
    Vector6 screw;
};

using Pose = Screw;
using Twist = Screw;
using Wrench = Screw;