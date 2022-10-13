#include "ThrusterManager.hpp"

void ThrusterManager::calculate_thruster_matrices() {
    // Number of columns is number of thrusters
    thrusts_to_screws.resize(thrusts_to_screws.rows(), thrusters.size());
    for(size_t i = 0; i < thrusters.size(); ++i) {
        thrusts_to_screws.col(i) = thrusters[i].get_contribution().as_vector6();
    }
    
    screw_to_thrusts = thrusts_to_screws.completeOrthogonalDecomposition().pseudoInverse();
}