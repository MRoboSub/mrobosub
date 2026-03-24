#ifndef  __PARAMETERS_H__
#define  __PARAMETERS_H__

#include <Eigen/Core> // Matrix4d

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace localization {
struct SensorUsage {
    bool is_dvl_used;
    bool is_imu_used;
    bool is_barometer_used;
    bool is_sonar_used;
    bool are_cams_used;

    SensorUsage(std::shared_ptr<rclcpp::Node> node);
};

struct SensorTopics {
    std::string imu_topic;
    std::string dvl_topic;
    std::string dvl_local_position_topic;
    std::string barometer_topic;
    std::string sonar_topic;
    
    SensorTopics(std::shared_ptr<rclcpp::Node> node);
};

struct ImuParameters {
    double acc_max;
    double gyro_max;
    double gyro_noise_density;
    double acc_noise_density;
    double acc_random_walk;
    double gyro_random_walk;
    double acc_bias_prior;
    double gyro_bias_prior;
    double imu_rate;
    double integration_covariance;
    double g;
    double dt_imu;
    
    ImuParameters(std::shared_ptr<rclcpp::Node>);
};

struct ImuPreintegrationParameters {
    double gap_time;

    ImuPreintegrationParameters(std::shared_ptr<rclcpp::Node> node);
};

struct DvlParameters {
    double prior_bias;
    double fom_threshold;

    DvlParameters(std::shared_ptr<rclcpp::Node> node);
};

struct BarometerParameters {
    double atmospheric_pressure;
    
    BarometerParameters(std::shared_ptr<rclcpp::Node> node);
};

struct OptimizationParameters {
    double lambda_upper_bound;
    double lambda_lower_bound;
    double lambda_initial;
    int max_iterations;
    double relative_error_tolerance;
    double absolute_error_tolerance;
    
    OptimizationParameters(std::shared_ptr<rclcpp::Node> node);
};

struct Extrinsics {
    Eigen::Matrix4d T_SD;   // dvl to imu
    Eigen::Matrix4d T_SSo;  // sonar to imu
    Eigen::Matrix4d T_BS;   // imu to body
    Eigen::Matrix4d T_SBa;  // barometer to imu
    Eigen::Matrix4d T_W_WD; // world to dvl world
    
    Extrinsics(std::shared_ptr<rclcpp::Node> node);
};

class Parameters {
public:
    Parameters(std::shared_ptr<rclcpp::Node> node);
    ~Parameters();

    // About the sensors
    SensorUsage                 _sensor_usage;
    SensorTopics                _sensor_topics;
    
    // Parameters 
    ImuParameters               _imu_params;
    ImuPreintegrationParameters _imu_preintegration_params;
    DvlParameters               _dvl_params;
    BarometerParameters         _barometer_params;
    OptimizationParameters      _optimization_params;

    Extrinsics                  _extrinsics;

    // Why is this not parameterized?
    std::vector<std::string> _rosbag_topics;

    // TODO: Can we remove this?
    bool _using_orbslam;
    int  _num_iters;
    bool _using_smoother;
    double _keyframe_gap_time;
    bool _using_pseudo_dvl;
};
} // namespace localization

#endif //__PARAMETERS_H__