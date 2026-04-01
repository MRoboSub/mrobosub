#ifndef  __POSEGRAPH_H__
#define  __POSEGRAPH_H__

#include "PreintegratedVelocityHelpers.h"
#include "Parameters.h"
#include "PosegraphNode.h"

#include <gtsam/base/Matrix.h>                     // Matrix44
#include <gtsam/base/Vector.h>                     // Vector
#include <gtsam/inference/Symbol.h>                // Symbol
#include <gtsam/inference/Key.h>                   // Key
#include <gtsam/geometry/Pose3.h>                  // Pose3
#include <gtsam/geometry/Rot3.h>                   // Rot3
#include <gtsam/navigation/NavState.h>             // NavState
#include <gtsam/navigation/CombinedImuFactor.h>    // PreintegratedCombinedMeasurements
#include <gtsam/navigation/ImuBias.h>              // imuBias::ConstantFactor
#include <gtsam/nonlinear/ISAM2.h>                 // ISAM2Params
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h> // LevenbergMarquardtParams, LevenbergMarquardtOptimizer
#include <gtsam/nonlinear/NonlinearFactorGraph.h>  // NonlinearFactorGraph
#include <gtsam/nonlinear/Values.h>                // Values
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h> // BatchFixedLabSmoother
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>      // FixedLagSmoother::KeyTimestampMap
#include <gtsam/slam/BetweenFactor.h>              // BetweenFactor 

#include <string>
#include <vector>
#include <random>
#include <memory> // std::unique_ptr

namespace localization {
class Posegraph {
public: // Members
    int _index;

    // Smoother
    gtsam::ISAM2Params _smoother_parameters;
    double _smoother_lag = 6.0; // TODO
    gtsam::BatchFixedLagSmoother _smoother_ISAM2;
    gtsam::FixedLagSmoother::KeyTimestampMap _smoother_timestamps;

    // GTSAM
    boost::shared_ptr<gtsam::NonlinearFactorGraph> _graph;
    boost::shared_ptr<gtsam::Values> _initial;
    boost::shared_ptr<gtsam::Values> _result;
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> _preintegrated_measurements; // used to be pim
    boost::shared_ptr<PreintegratedVelocityMeasurementsDvlOnly> _preintegrated_velocity_measurements; // used to be pvm
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> _preintegrated_measurement_params; // used to be pim_params
    std::unique_ptr<Parameters> _pose_graph_params; // use to be params

    // Prior imuBias
    gtsam::imuBias::ConstantBias _prior_imu_bias;
    gtsam::imuBias::ConstantBias _prior_dvl_bias; 

    // Transforms
    gtsam::Matrix44 _T_SD;   // sensor (IMU) to DVL affine transform
    gtsam::Matrix44 _T_SB;   // sensor (IMU) to robot center affine transform
    gtsam::Matrix44 _T_W_WD; // world to DVL world transform

    // Previous
    gtsam::Pose3 _prev_pose;
    gtsam::Vector3 _prev_vel;
    gtsam::NavState _prev_state;
    double _prev_keyframe_time; // OLD TODO: used for saving the previous keyframe time
    double _prev_dvl_odometry_time;
    gtsam::Rot3 _prev_dvl_odometry_rot;

    // Current
    gtsam::Pose3 _curr_pose;
    gtsam::Vector3 _curr_vel;
    gtsam::NavState _curr_state;
    gtsam::imuBias::ConstantBias _curr_imu_bias;
    double _curr_time;

    std::vector<gtsam::Vector3> _curr_dvl_vels;
    std::vector<gtsam::Pose3> _curr_dvl_poses;
    std::vector<gtsam::Rot3> _curr_dvl_rotations;
    std::vector<double> _curr_dvl_foms; // TODO: What does FOM mean?
    std::vector<gtsam::Rot3> _imu_rot_list;
    gtsam::Rot3 _imu_prev_rot;

    std::vector<double> _curr_dvl_timestamps;
    std::vector<double> _curr_dvl_local_timestamps;

private: // Members
    std::mt19937 _rng;
    std::normal_distribution<> _normal_distribution;
    double _visual_gap_time;
    
    double _W_measurement_z;
    gtsam::Vector3 _B_accelerometer_S;
    gtsam::Vector3 _B_gyroscope_S;
    gtsam::Vector3 _B_velocity_D;
    gtsam::Vector3 _W_position_C;

public: // Methods
    // Ctors + Dtors
    Posegraph();
    ~Posegraph();


    // Barometer factors
    void add_barometric_factor(double W_measurement_z, double measurement_noise, int baro_id);   

    // IMU factors
    void add_imu_factor();     

    // DVL Factors
    void add_velocity_factor();
    void add_velocity_factor_with_rotation_interpolation(bool using_slerp);
    void add_dvl_factor(bool using_slerp);
    void add_dvl_factor_imu_rotation();
    void add_dvl_odometry_factor(double noise);

    // DVL Suppliers
    void add_dvl_velocity(double time_stamp, gtsam::Vector3 velocity);
    void add_dvl_pose(double time_stamp, gtsam::Pose3 pose);
    void add_dvl_rotation(double time_stamp, gtsam::Rot3 rotation);

    // Other factors
    void add_visual_constraint_factor(gtsam::Pose3 between_pose, double weight, int prev_idx, int curr_idx);
    void add_prior_factor(gtsam::Pose3 initial_pose, gtsam::Vector initial_vel, double pose_noise);
   
    // Creating transforms
    void define_transforms();

    // Adding estimates
    void add_simple_estimate(double dt, double noise);
    void add_initial_estimate(gtsam::Pose3 initial_pose, gtsam::Vector3 intitial_vel);
    void add_visual_estimate(const std::vector<double> vertex);
  
    // Working with the pose graph
    void initialize_pose_graph();
    void initialize_pose_graph_from_imu(gtsam::Rot3 initial_rotation);
    void optimize_pose_graph();
    void optimize_pose_graph_smoother();

    // Properties
    gtsam::Rot3 find_current_pose_for_dvl_vel(double time_stamp) const;
    
    // Getters
    double get_depth_measurement() const;
    gtsam::Vector3 get_accelerometer_measurement() const;
    gtsam::Vector3 get_gyroscope_measurement() const;
    gtsam::Vector3 get_velocity_measurement() const;
    gtsam::Vector3 get_position_measurement() const;
    double get_visual_gap_time() const;
    
    // Setters 
    void set_depth_measurement(double W_measurement_z);
    void set_accelerometer_measurement(gtsam::Vector3 B_accelerometer_S);
    void set_gyroscope_measurement(gtsam::Vector3 B_gyroscope_S);
    void set_velocity_measurement(gtsam::Vector3 B_velocity_D);
    void set_position_measurement(gtsam::Vector3 W_position_C);
    void set_visual_gap_time(double visual_gap_time);

private: // Methods
    void initialize_parameters(std::shared_ptr<PosegraphNode> node);
    void set_imu_parameters();
    void set_smoother_parameters();

    // Helper methods used for creating the dvl factors
    void calculate_interpolated_rotations(bool is_using_slerp, std::vector<gtsam::Rot3> &interpolated_rotations);
    void calculate_integrated_pose_rotation(gtsam::Rot3 &integrated_pose_rotation);
    void reset_dvl_odometry();
    void reset_imu_rotation();
    void clear_dvl_timestamp_pose_vel();
};
} // namespace localization

#endif //__POSEGRAPH_H__