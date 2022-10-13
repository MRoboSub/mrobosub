#pragma once

#include <Eigen/Dense>

template <typename T>
using Vector6 = Eigen::Matrix<T, 6, 1>;

template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

#define NUM_DOF_AXES 6

template <typename T>
class Screw {
public:

    Screw() { }
    Screw(T surge, T sway, T heave,
            T roll, T pitch, T yaw)
    {
        screw << surge, sway, heave, roll, pitch, yaw;
    }

    Vector6<T> as_vector6() {
        return screw;
    }

    Vector3<T> linear() {
        return Vector3<T>(screw[0], screw[1], screw[2]);
    }

    Vector3<T> angular() {
        return Vector3<T>(screw[3], screw[4], screw[5]);
    }

    T surge() { return screw[0]; }
    T sway()  { return screw[1]; }
    T heave() { return screw[2]; }
    T roll()  { return screw[3]; }
    T pitch() { return screw[4]; }
    T yaw()   { return screw[5]; }

private:
    Eigen::Matrix<T, 6, 1> screw;
};

template <typename T>
using Pose = Screw<T>;

template <typename T>
using Twist = Screw<T>;

template <typename T>
using Wrench = Screw<T>;
