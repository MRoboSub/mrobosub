#include "Parameters.h"

#include <rclcpp/rclcpp.hpp>

namespace localization {
    SensorUsage::SensorUsage(const std::shared_ptr<rclcpp::Node> &node) {
        is_dvl_used = node->declare_parameter<bool>("sensor_usage.is_dvl_used", false);
        is_imu_used = node->declare_parameter<bool>("sensor_usage.is_imu_used", false);
        is_barometer_used = node->declare_parameter<bool>("sensor_usage.is_barometer_used", false);
        is_sonar_used = node->declare_parameter<bool>("sensor_usage.is_sonar_used", false);
        are_cams_used = node->declare_parameter<bool>("sensor_usage.are_cams_used", false);
    }

    SensorTopics::SensorTopics(const std::shared_ptr<rclcpp::Node> &node) {
        imu_topic = node->declare_parameter<std::string>("sensor_topics.imu_topic", "/todo");
        dvl_topic = node->declare_parameter<std::string>("sensor_topics.dvl_topic", "/todo");
        dvl_local_position_topic = node->declare_parameter<std::string>("sensor_topics.dvl_local_position_topic", "/todo");
        barometer_topic = node->declare_parameter<std::string>("sensor_topics.barometer_topic", "/todo");
        sonar_topic = node->declare_parameter<std::string>("sensor_topics.sonar_topic", "/unused");
    }    

    ImuParameters::ImuParameters(const std::shared_ptr<rclcpp::Node> &node) {
        g                      = node->declare_parameter<double>("imu_parameters.g", 0.0);
        acc_max                = node->declare_parameter<double>("imu_parameters.acc_max", 0.0);
        acc_noise_density      = node->declare_parameter<double>("imu_parameters.acc_noise_density", 0.0); 
        acc_bias_prior         = node->declare_parameter<double>("imu_parameters.acc_bias_prior", 0.0);
        acc_random_walk        = node->declare_parameter<double>("imu_parameters.acc_random_walk", 0.0);
        gyro_max               = node->declare_parameter<double>("imu_parameters.gyro_max", 0.0);
        gyro_noise_density     = node->declare_parameter<double>("imu_parameters.gyro_noise_density", 0.0);
        gyro_bias_prior        = node->declare_parameter("imu_parameters.gyro_bias_prior", 0.0);
        gyro_random_walk       = node->declare_parameter<double>("imu_parameters.gyro_random_walk", 0.0);
        integration_covariance = node->declare_parameter<double>("imu_parameters.integration_covariance", 0.0);
        imu_rate               = node->declare_parameter<double>("imu_parameters.imu_rate", 0.0);
        dt_imu = 1.0 / imu_rate;
    }

    ImuPreintegrationParameters::ImuPreintegrationParameters(const std::shared_ptr<rclcpp::Node> &node) {
        gap_time = node->declare_parameter<double>("imu_preintegration_parameters.gap_time", 1.0);
    }

    DvlParameters::DvlParameters(const std::shared_ptr<rclcpp::Node> &node) {
        prior_bias    = node->declare_parameter<double>("dvl_parameters.prior_bias", 0.01);
        fom_threshold = node->declare_parameter<double>("dvl_parameters.fom_threshold", 0.01);
    }

    BarometerParameters::BarometerParameters(const std::shared_ptr<rclcpp::Node> &node) {
        atmospheric_pressure =node->declare_parameter<double>("barometer_parameters.atmospheric_pressure", 993.3999633789062);
    }

    OptimizationParameters::OptimizationParameters(const std::shared_ptr<rclcpp::Node> &node){
        lambda_upper_bound = node->declare_parameter<double>("optimization_parameters.lambda_upper_bound", 1e10);
        lambda_lower_bound = node->declare_parameter<double>("optimization_parameters.lambda_lower_bound", 1e-10);
        lambda_initial = node->declare_parameter<double>("optimization_parameters.lambda_initial", 1e-5);
        max_iterations = node->declare_parameter<int>("optimization_parameters.max_iterations", 500);
        relative_error_tolerance = node->declare_parameter<double>("optimization_parameters.relative_error_tolerance", 1e-4);
        absolute_error_tolerance = node->declare_parameter<double>("optimization_parameters.absolute_error_tolerance", 1e-4);
    }

    Extrinsics::Extrinsics(const std::shared_ptr<rclcpp::Node> &node) {
        auto create_matrix = [](const std::vector<double> &flat, Eigen::Matrix4d &mat) {
            if (flat.size() == 16) {
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        mat(i, j) = flat[i * 4 + j];
                    }
                }
            } else {
                mat = Eigen::Matrix4d::Identity();
            }
        };
        create_matrix(node->declare_parameter<std::vector<double>>("imu_parameters.T_BS", {}), T_BS);
        create_matrix(node->declare_parameter<std::vector<double>>("sonar_parameters.T_SSo", {}), T_SSo);
        create_matrix(node->declare_parameter<std::vector<double>>("dvl_parameters.T_SD", {}), T_SD);
        create_matrix(node->declare_parameter<std::vector<double>>("dvl_parameters.T_W_WD", {}), T_W_WD);
        create_matrix(node->declare_parameter<std::vector<double>>("barometer_parameters.T_SBa", {}), T_SBa);
    }

    Parameters::Parameters(const std::shared_ptr<rclcpp::Node> &node)
        : _sensor_usage(node)
        , _sensor_topics(node)
        , _imu_params(node)
        , _imu_preintegration_params(node)
        , _dvl_params(node)
        , _barometer_params(node)
        , _optimization_params(node)
        , _extrinsics(node) {
        _using_smoother = node->declare_parameter<bool>("using_smoother", true);
        _using_pseudo_dvl = node->declare_parameter<bool>("using_pseudo_dvl", true);
        _using_orbslam = node->declare_parameter<bool>("using_orbslam", false);
        
        _keyframe_gap_time = node->declare_parameter<double>("keyframe_gap_time", 1.0);
        
        _num_iters = node->declare_parameter<int>("num_iters", 1000);

        _rosbag_topics.push_back(_sensor_topics.imu_topic);
        _rosbag_topics.push_back(_sensor_topics.dvl_topic);
        _rosbag_topics.push_back(_sensor_topics.dvl_local_position_topic);
        _rosbag_topics.push_back(_sensor_topics.barometer_topic);
        // _rosbag_topics.push_back(_sensor_topics.sonar_topic);
    }

    Parameters::~Parameters() {}
} // namespace localization