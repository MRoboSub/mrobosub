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
    Screw(Vector3<T> linear, Vector3<T> angular)
    {
        screw << linear, angular;    
    }

    const Vector6<T> as_vector6() const {
        return screw;
    }

    const Vector3<T> linear() const {
        return Vector3<T>(screw[0], screw[1], screw[2]);
    }

    const Vector3<T> angular() const {
        return Vector3<T>(screw[3], screw[4], screw[5]);
    }

    const T surge() const { return screw[0]; }
    const T sway()  const { return screw[1]; }
    const T heave() const { return screw[2]; }
    const T roll()  const { return screw[3]; }
    const T pitch() const { return screw[4]; }
    const T yaw()   const { return screw[5]; }

private:
    Eigen::Matrix<T, 6, 1> screw;
};

template <typename T>
using Pose = Screw<T>;

template <typename T>
using Twist = Screw<T>;

template <typename T>
using Wrench = Screw<T>;
