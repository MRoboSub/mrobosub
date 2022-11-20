#pragma once

#include <Eigen/Dense>
#include <vector>
#include "Thruster.hpp"

class ThrusterManager {
public:
    ThrusterManager() {};

    ThrusterManager(std::vector<Thruster> thrusters) : thrusters(thrusters) {
        for (auto it = thrusters.begin(); it != thrusters.end(); ++it)
            it->debug_log();
        
        calculate_thruster_matrices();
    }
    
    std::vector<Thruster>& get_thrusters() {
        return thrusters;
    }

    Eigen::VectorXd calculate_thrusts(Wrench<double> local_wrench);

    /**
     * Converts individual thruster PWMs to thrusts
    */
    std::vector<uint16_t> thrusts_to_pwms(Eigen::VectorXd thrusts, double voltage);

private:
    std::vector<Thruster> thrusters;

    using ThrusterMatrix_t = Eigen::Matrix<double, NUM_DOF_AXES, Eigen::Dynamic>;
    ThrusterMatrix_t thrusts_to_screws;
    Eigen::CompleteOrthogonalDecomposition<ThrusterMatrix_t> thrusts_to_screws_decomp;

    void calculate_thruster_matrices();

    Eigen::VectorXd calculate_raw_thrusts(Wrench<double> local_wrench);
    Eigen::VectorXd normalize_thrusts(Eigen::VectorXd raw_thrusts);
};