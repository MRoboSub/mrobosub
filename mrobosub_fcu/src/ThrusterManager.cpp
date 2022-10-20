#include "ThrusterManager.hpp"

#include <iostream>

void ThrusterManager::calculate_thruster_matrices() {
    // Number of columns is number of thrusters
    thrusts_to_screws.resize(Eigen::NoChange, thrusters.size());
    for(size_t i = 0; i < thrusters.size(); ++i) {
        thrusts_to_screws.col(i) = thrusters[i].get_contribution().as_vector6();
    }
    
    //screw_to_thrusts = thrusts_to_screws.completeOrthogonalDecomposition().pseudoInverse();
    thrusts_to_screws_decomp = thrusts_to_screws.completeOrthogonalDecomposition();
    //auto p = thrusts_to_screws_decomp.pseudoInverse();

    // std::cout << thrusts_to_screws << std::endl;

    // Eigen::FullPivLU<ThrusterMatrix_t> lu(thrusts_to_screws);
}

/**
 * After calculating thrusts, normalizes them so that no
 * thruster max is exceeded
 **/
Eigen::VectorXd ThrusterManager::calculate_thrusts(Wrench<double> local_wrench) {
    return normalize_thrusts(calculate_raw_thrusts(local_wrench));
}

Eigen::VectorXd ThrusterManager::normalize_thrusts(Eigen::VectorXd raw_thrusts)
{
    double max_thrust_ratio = 0;
    for (size_t i = 0; i < raw_thrusts.size(); ++i)
    {
        double thrust_ratio = raw_thrusts[i];

        if (raw_thrusts[i] < 0) {
            thrust_ratio /= thrusters[i].get_max_neg_thrust();
        } else {
            thrust_ratio /= thrusters[i].get_max_pos_thrust();
        }
        
        thrust_ratio = std::abs(thrust_ratio);

        if (thrust_ratio > max_thrust_ratio) {
            max_thrust_ratio = thrust_ratio;
        }
    }

    if (max_thrust_ratio > 1) {
        for (size_t i = 0; i < raw_thrusts.size(); ++i) {
            raw_thrusts[i] /= max_thrust_ratio;
        }
    }

    return raw_thrusts;
}

Eigen::VectorXd ThrusterManager::calculate_raw_thrusts(Wrench<double> local_wrench) {
    Eigen::VectorXd thrusts = thrusts_to_screws_decomp.solve(local_wrench.as_vector6());
    
    // std::vector<double> thrusts(sol.rows());

    // for(size_t i = 0; i < sol.rows(); ++i) {
    //     pwms[i] = thrusters[i].thrust_to_pwm(sol[i]);
    // }

    return thrusts;
}

/**
 * Converts individual thruster PWMs to thrus
*/
std::vector<uint16_t> ThrusterManager::thrusts_to_pwms(Eigen::VectorXd thrusts) {
    std::vector<uint16_t> pwms(thrusts.rows());
    
    for(size_t i = 0; i < thrusts.rows(); ++i) {
       pwms[i] = thrusters[i].thrust_to_pwm(thrusts[i]);
    }

    return pwms;
}