#include "ThrusterManager.hpp"

void ThrusterManager::calculate_thruster_matrices() {
    // Number of columns is number of thrusters
    thrusts_to_screws.resize(Eigen::NoChange, thrusters.size());
    for(size_t i = 0; i < thrusters.size(); ++i) {
        thrusts_to_screws.col(i) = thrusters[i].get_contribution().as_vector6();
    }
    
    //screw_to_thrusts = thrusts_to_screws.completeOrthogonalDecomposition().pseudoInverse();
    thrusts_to_screws_decomp = thrusts_to_screws.completeOrthogonalDecomposition();
}

std::vector<uint16_t> ThrusterManager::calculate_pwm_output(Screw<double> local_wrench) {
    Eigen::VectorXd sol = thrusts_to_screws_decomp.solve(local_wrench.as_vector6());
    
    std::vector<uint16_t> pwms(sol.rows());

    for(size_t i = 0; i < sol.rows(); ++i) {
        pwms[i] = thrusters[i].thrust_to_pwm(sol[i]);
    }

    return pwms;
}