#include "PreintegratedVelocityHelpers.h"

namespace localization {
    PreintegratedVelocityParameters::PreintegratedVelocityParameters()
        : _bias_velocity_covariance(gtsam::Matrix3::Zero())
        , _bias_initial(gtsam::Matrix3::Zero()) {}

    PreintegratedVelocityParameters::PreintegratedVelocityParameters(
        const gtsam::Matrix3 &bias_velocity_covariance,
        const gtsam::Matrix3 &bias_initial
    ) : _bias_velocity_covariance(bias_velocity_covariance)
      , _bias_initial(bias_initial) {}

    PreintegratedVelocityParameters::PreintegratedVelocityParameters(
        const boost::shared_ptr<PreintegratedVelocityParameters> &p
    ) {
        _bias_velocity_covariance = p->_bias_velocity_covariance;
        _bias_initial = p->_bias_initial;
    }

    // TODO: Make as overloaded= and overloaded<<
    // void PreintegratedVelocityParameters::print(const std::string &s = "") const;
    // bool PreintegratedVelocityParameters::equals(const PreintegratedVelocityParameters &expected, double tolerance = 1e-9) const;
    
    void PreintegratedVelocityParameters::set_bias_velocity_covariance(const gtsam::Matrix3 &covariance) {
        _bias_velocity_covariance = covariance;
    }
    void PreintegratedVelocityParameters::set_bias_initial(const gtsam::Matrix3 &covariance) {
        _bias_initial = covariance;
    }            


    PreintegratedVelocityMeasurementsDvlOnly::PreintegratedVelocityMeasurementsDvlOnly() 
        : _preintegrated_measured_covariance(gtsam::Matrix3::Zero()) {}

    PreintegratedVelocityMeasurementsDvlOnly::~PreintegratedVelocityMeasurementsDvlOnly() {}

    void PreintegratedVelocityMeasurementsDvlOnly::reset_integration() {
        _accumulated_poses = gtsam::Pose3();
        _accumulated_positions = gtsam::Point3(0, 0, 0);
        _delp_delbias_dvl = gtsam::Matrix3::Zero();
        _preintegrated_measured_covariance = gtsam::Matrix3::Zero();
        _velocity_list.clear();
        _rotations_list.clear();
        _dt_list.clear();
    }

    void PreintegratedVelocityMeasurementsDvlOnly::reset_integration_and_bias(const gtsam::imuBias::ConstantBias &bias) {
        reset_integration();
        _dvl_bias_for_imu = bias;
    }

    void PreintegratedVelocityMeasurementsDvlOnly::integrate_measurements(
        const gtsam::Vector3 &linear_velocity,
        const gtsam::Rot3 &interpolated_rotation,
        double &dt // TODO: Is this an out param? Why is this passed by ref?
    ) {
        if (dt <= 0) {
            std::cout << "Warning: dt <= 0\n";
            dt = 0.0001; 
        }

        _velocity_list.push_back(linear_velocity);
        _rotations_list.push_back(interpolated_rotation);
        _dt_list.push_back(dt);
    }
    
    void PreintegratedVelocityMeasurementsDvlOnly::integrate_measurements_noise(
        const gtsam::Vector3 &linear_velocity,
        const gtsam::Rot3 &interpolated_rotation,
        double &dt, // TODO: Is this an out param? Why is this passed by ref?
        const double &fom
    ) {
        if (dt <= 0) {
            std::cout << "Warning: dt <= 0\n";
            dt = 0.0001; 
        }

        // TODO: Investigate this.
        gtsam::Matrix3 B = interpolated_rotation.matrix() * dt;
        gtsam::Matrix3 dvl_covariance = fom * fom * gtsam::I_3x3; // Figure of Merit should be SQUARED
        _preintegrated_measured_covariance += B * dvl_covariance * B.transpose();

        _velocity_list.push_back(linear_velocity);
        _rotations_list.push_back(interpolated_rotation);
        _dt_list.push_back(dt);
    }
    
    gtsam::Point3 PreintegratedVelocityMeasurementsDvlOnly::predict(const gtsam::Point3 &bias, gtsam::Matrix3 &H_bias) const {
        gtsam::Point3 predicted_position;

        // Resets the H_bias
        H_bias = gtsam::Matrix3::Zero();

        // integrate using _velocity_list and _rotations_list
        for (int i = 0; i < _velocity_list.size(); ++i) {
            gtsam::Vector3 velocity = _velocity_list[i] - bias;
            gtsam::Vector3 delta_pos = _rotations_list[i].matrix() * velocity * _dt_list[i]; // Looks like pretty bog-standard integration to me.
            predicted_position += delta_pos;
            H_bias += _rotations_list[i].matrix() * _dt_list[i]; // What is this bias?
        }

        return predicted_position;
    }

    gtsam::Pose3  PreintegratedVelocityMeasurementsDvlOnly::get_accumulated_poses() const {
        return _accumulated_poses;
    }

    gtsam::Point3 PreintegratedVelocityMeasurementsDvlOnly::get_accumulated_positions() const {
        return _accumulated_positions;
    }

    gtsam::Matrix PreintegratedVelocityMeasurementsDvlOnly::get_delpij_delbias_omega() const {
        return _delp_delbias_omega;
    }

    gtsam::Matrix PreintegratedVelocityMeasurementsDvlOnly::get_delpij_delbias_dvl() const {
        return _delp_delbias_dvl;
    }

    gtsam::Matrix PreintegratedVelocityMeasurementsDvlOnly::get_preintegrated_measured_covariance() const {
        return _preintegrated_measured_covariance;
    }

    void PreintegratedVelocityMeasurementsDvlOnly::update_accumulated_stats(const gtsam::imuBias::ConstantBias &bias) {
        gtsam::Matrix3 H;
        _accumulated_positions = predict(bias.accelerometer(), H);
        _delp_delbias_dvl = H; 
    }

} // namespace localization