#ifndef  __POSEGRAPH_H__
#define  __POSEGRAPH_H__
#include <string>
#include <vector>
#include <random>

namespace localization {
class Posegraph {
public: // Members
    int _index;

    // Smoother
    int /* gtsam::ISAM2Params */ _smoother_parameters;
    double _smoother_lag = 6.0; // TODO
    int /* gtsam::BatchFixedLagSmoother */ _smoother_ISAM2;
    int /* gtsam::FixedLagSmoother::KeyTimestampMap */ _smoother_timestamps;

    // GTSAM
    int /*gtsam::NonlinearFactorGraph*/ *_graph;
    int /*gtsam::Values*/ *_initial;
    int /*gtsam::Values*/ *_result;
    int /*gtsam::PreintegratedCombinedMeasurements*/ *_preintegrated_measurements; // used to be pim
    // BlueRovPreintegratedVelocityMeasurements*/ *_pvm;
    int /*PreintegratedVelocityMeasurementsDvlOnly*/ *_preintegrated_velocity_measurements; // used to be pvm
    int /*gtsam::PreintegratedCombinedMeasurements::Params*/ *_preintergrated_measurement_params; // used to be pim_params
    int /*PoseGraphParameters*/ *_pose_graph_params; // use to be params

    // Prior imuBias
    int /*gtsam::imuBias::ConstantBias*/ _prior_imu_bias;
    int /*gtsam::imuBias::ConstantBias*/ _prior_dvl_bias; // TODO: = gtsam::imuBias::ConstantBias(gtsam::Vector3(0.01, 0.01, 0.01), gtsam::Vector3(0, 0, 0));

    // Transforms
    int /*gtsam::Matrix44*/ _T_SD;   // sensor (IMU) to DVL affine transform
    int /*gtsam::Matrix44*/ _T_SB;   // sensor (IMU)to robot center affine transform
    int /*gtsam::Matrix44*/ _T_W_WD; // world to DVL world transform

    // Previous
    int /*gtsam::Pose3*/ _prev_pose;
    int /*gtsam::Vector3*/ _prev_vel;
    int /*gtsam::NavState*/ _prev_state;
    double _prev_keyframe_time; // OLD TODO: used for saving the previous keyframe time

    // Current
    int /*gtsam::Pose3*/ _current_pose;
    int /*gtsam::Vector3*/ _current_vel;
    int /*gtsam::NavState*/ _current_state;
    int /*gtsam::imuBias::ConstantBias*/ _current_imu_bias;
    double _current_time;

    std::vector<int /*gtsam::Vector3*/> _current_dvl_vels;
    std::vector<int /*gtsam::Pose3*/> _current_dvl_poses;
    std::vector<int /*gtsam::Rot3*/> _current_dvl_rotations;
    std::vector<double> _current_dvl_foms; // TODO: What does FOM mean?
    std::vector<int /*gtsam::Rot3*/> _imu_rot_list;
    int /*gtsam::Rot3*/ _imu_prev_rot;

    std::vector<double> _current_dvl_timestamps;
    std::vector<double> _current_dvl_local_timestamps;

private: // Members
    std::mt19937 _rng;
    std::normal_distribution<> _normal_distribution;
    double _visual_gap_time;
    
    double _W_measurement_z;
    /* gtsam::Vector3 */ double _B_accelerometer_S;
    /* gtsam::Vector3 */ double _B_gyroscope_S;
    /* gtsam::Vector3 */ double _B_velocity_D;
    /* gtsam::Vector3 */ double _B_position_C;

public: // Methods
    // Ctors + Dtors
    Posegraph();
    Posegraph(std::string &config_file);
    Posegraph();

    // Depth factors
    void add_depth_factor();   

    // IMU factors
    void add_imu_factor();     
    void set_imu_params();

    // DVL Factors
    void add_velocity_factor();
    void add_velocity_factor_with_rotation_interpolation(bool using_slerp);
    void add_dvl_factor(bool using_slerp);
    void add_dvl_factor_imu_rotation();
    void add_dvl_odometry_factor(double noise);

    // DVL Suppliers
    void add_dvl_velocity(double time_stamp /* gtsam::Vector3 velocity */);
    void add_dvl_pose(double time_stamp /* gtsam::Pose3 pose*/);
    void add_dvl_rotation(double time_stamp /* gtsam::Rot3 rotation */);

    // Other factors
    void add_visual_constraint_factor(/*gtsam::Pose3 between_pose, double weight, int prev_idx, int curr_idx*/);
    void add_sonar_factor();
    void add_prior_factor(/*gtsam::Pose3 initial_pose, gtsam::Vector initial_vec, double pose_noise*/);
   
    // Creating transforms
    void define_transforms();

    // Adding estimates
    void add_simple_estimate(double dt, double noise);
    void add_initial_estimate(/*gtsam::Pose3 initial_pose, gtsam::Vector3 intitial_vel*/);
    void add_visual_estimate(const std::vector<double> vertex);
  
    // Working with the pose graph
    void initialize_pose_graph();
    void initialize_pose_graph_from_imu(/*gtsam::Rot3 initial_rotation*/);
    void optimize_pose_graph();
    void optimize_pose_graph_smoother();
    void add_edges_to_graph(const std::vector<std::vector<double>> &edges);
    
    // Getters
    double get_depth_measurement();
    void /* gtsam::Vector3 */ get_accelerometer_measurement();
    void /* gtsam::Vector3 */ get_gyroscope_measurement();
    void /* gtsam::Vector3 */ get_velocity_measurement();
    void /* gtsam::Vector3 */ get_position_measurement();
    double get_visual_gap_time();
    void /* gtsam::Rot3 */ find_current_pose_for_dvl_vel(double time_stamp);
    
    // Setters 
    void set_depth_measurement(double W_measurement_z);
    void set_accelerometer_measurement(int /*gtsam::Vector3*/ B_accelerometer_S);
    void set_gyroscope_measurement(int /*gtsam::Vector3*/ B_gyroscope_S);
    void set_velocity_measurement(int /*gtsam::Vector3*/ B_velocity_D);
    void set_position_measurement(int /*gtsam::Vector3*/ W_position_C);
    void set_visual_gap_time(double visualGapTime);
};
} // namespace localization

#endif //__POSEGRAPH_H__