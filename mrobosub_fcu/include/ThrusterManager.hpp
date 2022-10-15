#pragma once

#include <Eigen/Dense>
#include <vector>
#include "Thruster.hpp"

struct Axis {
    double x;
    double y;
    double z;
};

struct Limits {
    double min;
    double max;
};

class ThrusterManager {
public:
    ThrusterManager(std::vector<Thruster> thrusters) : thrusters(thrusters) {}
    
    std::vector<Thruster>& get_thrusters(){
        return thrusters;
    }

    void calculate_thruster_matrices();
    void update_thruster(int i);
    Limits get_extrema(Axis axis, bool torque);

    Screw<double>& limit_desires(Screw<double> desires);
    


private:
    std::vector<Thruster> thrusters;
    
    Screw<Axis> axes = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Screw<Limits> dof_limits;
    
    // Eigen::Matrix<double, 3, Eigen::Dynamic> thrust_matrix;
    // Eigen::Matrix<double, 3, Eigen::Dynamic> torque_matrix;

    using ThrusterMatrix_t = Eigen::Matrix<double, NUM_DOF_AXES, Eigen::Dynamic>;
    ThrusterMatrix_t thrusts_to_screws;
    Eigen::CompleteOrthogonalDecomposition<ThrusterMatrix_t> thrusts_to_screws_decomp;
    //Eigen::Matrix<double, Eigen::Dynamic, NUM_DOF_AXES> screw_to_thrusts;

    Eigen::Quaterniond orientation; //orientation from sensor
    Eigen::Quaterniond measured_heading; // measured heading from sensor (heading_quat)
    Eigen::Quaterniond measured_heading_pitch; // heading with pitch value (hp_quat)
    Eigen::Quaterniond absolute_heading; // multiply orientation transpose with heading for correction (spitz_to_sub_quat)

    void limit(Screw<double> desires, Screw<double> limits);
};