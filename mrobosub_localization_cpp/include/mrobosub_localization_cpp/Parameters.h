#ifndef  __PARAMETERS_H__
#define  __PARAMETERS_H__

namespace localization {
struct SensorList {
    bool is_dvl_used;
    bool is_imu_used;
    bool is_barometer_used;
    bool is_sonar_used;
    bool are_cams_used;
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
    double dvl_bias_prior;
    double dt_imu;
};

struct SensorTopics {
    std::string imu_topic;
    std::string dvl_topic;
    std::string barometer_topic;
    std::string sonar_topic;
    std::string dvl_local_position_topic;
};

struct Extrinsics {
    int /*Eigen::Matrix4d */ T_SD;   // dvl to imu
    int /*Eigen::Matrix4d */ T_SSo;  // sonar to imu
    int /*Eigen::Matrix4d */ T_BS;   // imu to body
    int /*Eigen::Matrix4d */ T_SBa;  // barometer to imu
    int /*Eigen::Matrix4d */ T_W_WD; // world to dvl world
};

struct OptimizationParameters {
    double lambda_upper_bound;
    double lambda_lower_bound;
    double lambda_initial;
    int max_iterations;
    double relative_error_tolerance;
    double absolute_error_tolerance;
};

struct ImuPreintegrationParameters {
    double gap_time;
};

class Parameters {
public:
    Parameters();
    ~Parameters();

    ImuParameters           _imu_params;
    Extrinsics              _extrinsics;
    SensorList              _sensor_list;
    SensorTopics            _sensor_topics;
    OptimizationParameters  _optimization_params;
    ImuPreintegrationParams _imu_preintegration_params;

    std::vector<std::string> _rosbag_topics;

    // TODO: Can we remove this?
    bool _using_orbslam;

    // TODO: Parse these into separate parameter classes.
    int  _num_iters;
    double _baro_atm_pressure;
    bool _using_smoother;
    double _keyframe_gap_time;
    double _dvl_fom_threshold;
    bool _using_pseudo_dvl;
}


} // namespace localization

#endif //__PARAMETERS_H__