#include "Parameters.h"

#include <rclcpp/rclcpp.hpp>

namespace localization {
    SensorUsage::SensorUsage(std::shared_ptr<rclcpp::Node> node) {
        node->declare_parameter("sensor_usage.is_dvl_used", true);
        is_dvl_used = node->get_parameter("sensor_usage.is_dvl_used").as_bool();
        
        node->declare_parameter("sensor_usage.is_imu_used", true);
        is_imu_used = node->get_parameter("sensor_usage.is_imu_used").as_bool();
        
        node->declare_parameter("sensor_usage.is_barometer_used", true);
        is_barometer_used = node->get_parameter("sensor_usage.is_barometer_used").as_bool();
        
        node->declare_parameter("sensor_usage.is_sonar_used", false);
        is_sonar_used = node->get_parameter("sensor_usage.is_sonar_used").as_bool();
        
        node->declare_parameter("sensor_usage.are_cams_used", false);
        are_cams_used = node->get_parameter("sensor_usage.are_cams_used").as_bool();
    }

    SensorTopics::SensorTopics(std::shared_ptr<rclcpp::Node> node) {
        node->declare_parameter("sensor_topics.imu_topic", "/todo");
        imu_topic = node->get_parameter("sensor_topics.imu_topic").as_string();

        node->declare_parameter("sensor_topics.dvl_topic", "/todo");
        dvl_topic = node->get_parameter("sensor_topics.dvl_topic").as_string();

        node->declare_parameter("sensor_topics.dvl_local_position_topic", "/todo");
        dvl_local_position_topic = node->get_parameter("sensor_topics.dvl_local_position_topic").as_string();

        node->declare_parameter("sensor_topics.barometer_topic", "/todo");
        barometer_topic = node->get_parameter("sensor_topics.barometer_topic").as_string();

        node->declare_parameter("sensor_topics.sonar_topic", "/unused");
        sonar_topic = node->get_parameter("sensor_topics.sonar_topic").as_string();
    }    

    ImuParameters::ImuParameters(std::shared_ptr<rclcpp::Node> node) {
        node->declare_parameter("imu_parameters.g", 0.0);
        g = node->get_parameter("imu_parameters.g").as_double();
        
        node->declare_parameter("imu_parameters.acc_max", 0.0);
        acc_max = node->get_parameter("imu_parameters.acc_max").as_double();
        
        node->declare_parameter("imu_parameters.acc_noise_density", 0.0); 
        acc_noise_density = node->get_parameter("imu_parameters.acc_noise_density").as_double(); 
        
        node->declare_parameter("imu_parameters.acc_bias_prior", 0.0);
        acc_bias_prior = node->get_parameter("imu_parameters.acc_bias_prior").as_double();
        
        node->declare_parameter("imu_parameters.acc_random_walk", 0.0);
        acc_random_walk = node->get_parameter("imu_parameters.acc_random_walk").as_double();
        
        node->declare_parameter("imu_parameters.gyro_max", 0.0);
        gyro_max = node->get_parameter("imu_parameters.gyro_max").as_double();
        
        node->declare_parameter("imu_parameters.gyro_noise_density", 0.0);
        gyro_noise_density = node->get_parameter("imu_parameters.gyro_noise_density").as_double();

        node->declare_parameter("imu_parameters.gyro_bias_prior", 0.0);
        gyro_bias_prior = node->get_parameter("imu_parameters.gyro_bias_prior").as_double();

        node->declare_parameter("imu_parameters.gyro_random_walk", 0.0);
        gyro_random_walk = node->get_parameter("imu_parameters.gyro_random_walk").as_double();
        
        node->declare_parameter("imu_parameters.integration_covarience", 0.0001);
        integration_covariance = node->get_parameter("imu_parameters.integration_covarience").as_double();
        
        node->declare_parameter("imu_parameters.imu_rate", 250.0);
        imu_rate = node->get_parameter("imu_parameters.imu_rate").as_double();

        dt_imu = 1 / imu_rate;
    }

    ImuPreintegrationParameters::ImuPreintegrationParameters(std::shared_ptr<rclcpp::Node> node) {
        node->declare_parameter("imu_preintegration_parameters.gap_time", 1.0);
        gap_time = node->get_parameter("imu_preintegration_parameters.gap_time").as_double();
    }

    DvlParameters::DvlParameters(std::shared_ptr<rclcpp::Node> node) {
        node->declare_parameter("dvl_parameters.prior_bias", 0.01);
        prior_bias = node->get_parameter("dvl_parameters.prior_bias").as_double();

        node->declare_parameter("dvl_parameters.fom_threshold", 0.01);
        fom_threshold = node->get_parameter("dvl_parameters.fom_threshold").as_double();
    }

    BarometerParameters::BarometerParameters(std::shared_ptr<rclcpp::Node> node) {
        node->declare_parameter("barometer_parameters.atmospheric_pressure", 993.3999633789062);
        atmospheric_pressure = node->get_parameter("barometer_parameters.atmospheric_pressure").as_double();
    }

    OptimizationParameters::OptimizationParameters(std::shared_ptr<rclcpp::Node> node){
        node->declare_parameter("optimization_parameters.lambda_upper_bound", 1e10);
        lambda_upper_bound = node->get_parameter("optimization_parameters.lambda_upper_bound").as_double();

        node->declare_parameter("optimization_parameters.lambda_lower_bound", 1e-10);
        lambda_lower_bound = node->get_parameter("optimization_parameters.lambda_lower_bound").as_double();

        node->declare_parameter("optimization_parameters.lambda_initial", 1e-5);
        lambda_initial = node->get_parameter("optimization_parameters.lambda_initial").as_double();

        node->declare_parameter("optimization_parameters.max_iterations", 500);
        max_iterations = node->get_parameter("optimization_parameters.max_iterations").as_double();

        node->declare_parameter("optimization_parameters.relative_error_tolerance", 1e-4);
        relative_error_tolerance = node->get_parameter("optimization_parameters.relative_error_tolerance").as_double();

        node->declare_parameter("optimization_parameters.absolute_error_tolerance", 1e-4);
        absolute_error_tolerance = node->get_parameter("optimization_parameters.absolute_error_tolerance").as_double();
    }

    Extrinsics::Extrinsics(std::shared_ptr<rclcpp::Node> node) {
        node->declare_parameter("imu_parameters.T_BS", std::vector<double>());
        std::vector<double> T_BS_flat = node->get_parameter("imu_parameters.T_BS").as_double_array();
        for(int i = 0; i < 4; ++i) {
            for(int j = 0; j < 4; ++j) {
                T_BS(i, j) = T_BS_flat[i * 4 + j];
            }
        }

        node->declare_parameter("sonar_parameters.T_SSo", std::vector<double>());
        std::vector<double> T_SSo_flat = node->get_parameter("sonar_parameters.T_SSo").as_double_array();
        for(int i = 0; i < 4; ++i) {
            for(int j = 0; j < 4; ++j) {
                T_SSo(i, j) = T_SSo_flat[i * 4 + j];
            }
        }
        
        node->declare_parameter("dvl_parameters.T_SD", std::vector<double>());
        std::vector<double> T_SD_flat = node->get_parameter("dvl_parameters.T_SD").as_double_array();
        for(int i = 0; i < 4; ++i) {
            for(int j = 0; j < 4; ++j) {
                T_SD(i, j) = T_SD_flat[i * 4 + j];
            }
        }
        
        node->declare_parameter("dvl_parameters.T_W_WD", std::vector<double>());
        std::vector<double> T_W_WD_flat = node->get_parameter("dvl_parameters.T_W_WD").as_double_array();
        for(int i = 0; i < 4; ++i) {
            for(int j = 0; j < 4; ++j) {
                T_W_WD(i, j) = T_W_WD_flat[i * 4 + j];
            }
        }
        
        node->declare_parameter("barometer_parameters.T_SBa", std::vector<double>());
        std::vector<double> T_SBa_flat = node->get_parameter("barometer_parameters.T_SBa").as_double_array();
        for(int i = 0; i < 4; ++i) {
            for(int j = 0; j < 4; ++j) {
                T_SBa(i, j) = T_SBa_flat[i * 4 + j];
            }
        }
    }

    Parameters::Parameters(std::shared_ptr<rclcpp::Node> node)
        : _sensor_usage(node)
        , _sensor_topics(node)
        , _imu_params(node)
        , _imu_preintegration_params(node)
        , _dvl_params(node)
        , _barometer_params(node)
        , _optimization_params(node)
        , _extrinsics(node) {
        
        node->declare_parameter("using_orbslam", false);
        _using_orbslam = node->get_parameter("using_orbslam").as_bool();
        
        node->declare_parameter("num_iters", 1000);
        _num_iters = node->get_parameter("num_iters").as_int();
        
        node->declare_parameter("using_smoother", true);
        _using_smoother = node->get_parameter("using_smoother").as_bool();
        
        node->declare_parameter("keyframe_gap_time", 1.0);
        _keyframe_gap_time = node->get_parameter("keyframe_gap_time").as_double();
        
        node->declare_parameter("using_pseudo_dvl", true);
        _using_pseudo_dvl = node->get_parameter("using_pseudo_dvl").as_bool();

        _rosbag_topics.push_back(_sensor_topics.imu_topic);
        _rosbag_topics.push_back(_sensor_topics.dvl_topic);
        _rosbag_topics.push_back(_sensor_topics.dvl_local_position_topic);
        _rosbag_topics.push_back(_sensor_topics.barometer_topic);
        // _rosbag_topics.push_back(_sensor_topics.sonar_topic);
    }

    Parameters::~Parameters() {}
} // namespace localization