#include "Posegraph.h"

#include "BluerovBarometerFactor.h"
#include "DvlOnlyFactor.h"

#include <limits>
#include <numeric>
#include <utility>

namespace localization {
// Ctor + Dtor
Posegraph::Posegraph(std::mutex &mtx) 
    : _index(0)
    , _mtx(mtx)
    , _graph(new gtsam::NonlinearFactorGraph())
    , _initial(new gtsam::Values())
    , _result(new gtsam::Values())
    , _preintegrated_velocity_measurements(new PreintegratedVelocityMeasurementsDvlOnly())
    , _prior_imu_bias(gtsam::imuBias::ConstantBias(gtsam::Vector3(0.01, 0.01, 0.01), gtsam::Vector3(0, 0, 0)))
    , _prev_dvl_odometry_time(0.0)
    , _prev_dvl_odometry_rot(gtsam::Rot3())
  {}

Posegraph::~Posegraph() {}

void Posegraph::initialize_parameters(std::shared_ptr<PosegraphNode> node) {
    _pose_graph_params = std::make_unique<Parameters>(node);
    set_smoother_parameters();
    set_imu_parameters();
    define_transforms();
}

void Posegraph::set_smoother_parameters() {
    _smoother_parameters.relinearizeThreshold = 0.01;
    _smoother_parameters.relinearizeSkip = 1;
    _smoother_ISAM2 = gtsam::BatchFixedLagSmoother(_smoother_lag);
}

void Posegraph::set_imu_parameters() {
    _preintegrated_measurement_params = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
        gtsam::Vector3(0, 0, -_pose_graph_params->_imu_params.g) // Z-UP!
    );
    _preintegrated_measurement_params->setAccelerometerCovariance(
        gtsam::I_3x3 * std::pow(_pose_graph_params->_imu_params.acc_noise_density, 2)
    );
    _preintegrated_measurement_params->setGyroscopeCovariance(
        gtsam::I_3x3 * std::pow(_pose_graph_params->_imu_params.gyro_noise_density, 2)
    );
    _preintegrated_measurement_params->setIntegrationCovariance(
        gtsam::I_3x3 * _pose_graph_params->_imu_params.integration_covariance
    );
    _preintegrated_measurement_params->setBiasAccCovariance(
        gtsam::I_3x3 * std::pow(_pose_graph_params->_imu_params.acc_random_walk, 2)
    );
    _preintegrated_measurement_params->setBiasOmegaCovariance(
        gtsam::I_3x3 * std::pow(_pose_graph_params->_imu_params.gyro_random_walk, 2)
    );

    double prior_acc_bias  = _pose_graph_params->_imu_params.acc_bias_prior;
    double prior_gyro_bias = _pose_graph_params->_imu_params.gyro_bias_prior;
    double prior_dvl_bias  = _pose_graph_params->_dvl_params.prior_bias;

    gtsam::Vector3 prior_acc_bias_vec = (gtsam::Vector(3) << prior_acc_bias, prior_acc_bias, prior_acc_bias).finished();
    gtsam::Vector3 prior_gyro_bias_vec = (gtsam::Vector(3) << prior_gyro_bias, prior_gyro_bias, prior_gyro_bias).finished();
    gtsam::Vector3 prior_dvl_bias_vec = (gtsam::Vector(3) << prior_dvl_bias, prior_dvl_bias, prior_dvl_bias).finished();
    gtsam::Vector3 prior_zeros_vec = (gtsam::Vector(3) << 0, 0, 0).finished();

    _prior_imu_bias = gtsam::imuBias::ConstantBias(prior_acc_bias_vec, prior_gyro_bias_vec);
    _prior_dvl_bias = gtsam::imuBias::ConstantBias(prior_dvl_bias_vec, prior_zeros_vec);

    _preintegrated_measurements = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(
        _preintegrated_measurement_params,
        _prior_imu_bias
    );
}

void Posegraph::add_barometric_factor(double W_measurement_z, double measurement_noise, int baro_id) {
    _graph->emplace_shared<BluerovBarometerFactor>(
        gtsam::Symbol('x', baro_id),
        W_measurement_z,
        gtsam::noiseModel::Isotropic::Sigma(1, measurement_noise)
    );
}

void Posegraph::add_imu_factor() {
    _graph->emplace_shared<gtsam::CombinedImuFactor>(
        gtsam::Symbol('x', _index - 1),
        gtsam::Symbol('v', _index - 1),
        gtsam::Symbol('x', _index),
        gtsam::Symbol('v', _index),
        gtsam::Symbol('b', _index - 1),
        gtsam::Symbol('b', _index),
        _preintegrated_measurement_params.get()
    );

    // Must reset integration for next interval
    _preintegrated_measurements->resetIntegrationAndSetBias(_curr_imu_bias);
}

void Posegraph::add_velocity_factor_with_rotation_interpolation(bool using_slerp) {
    // Get the size of the current dvl measurements
    size_t dvl_size = _curr_dvl_timestamps.size();
    size_t dvl_local_size = _curr_dvl_local_timestamps.size();

    // If any of the size is zero, assert error
    if (dvl_size == 0 && dvl_local_size == 0) {
        assert(false);
    }

    std::vector<gtsam::Rot3> interpolated_rotations;
    calculate_interpolated_rotations(using_slerp, interpolated_rotations);

    // Calculate the integrated pose translation by accumulating dvl velocities with the right rotation for each
    // timestamp we have
    gtsam::Point3 integrated_pose_translation = gtsam::Point3(0.0, 0.0, 0.0);
    double dt_dvl = -1;
    gtsam::Matrix3 integrated_rot_matrix = gtsam::Matrix3::Identity();
    for (size_t i = 0; i < dvl_size; ++i) {
        dt_dvl = (dt_dvl == -1)
               ? _curr_dvl_timestamps[i] - _prev_keyframe_time
               : _curr_dvl_timestamps[i] - _curr_dvl_timestamps[i-1]; 
        integrated_rot_matrix = integrated_rot_matrix * interpolated_rotations[i].matrix();
        integrated_pose_translation += integrated_rot_matrix * _curr_dvl_vels[i] * dt_dvl;
    }

    // Calculate the integrated pose rotation by accumulating rotations
    gtsam::Rot3 integrated_pose_rotation;
    calculate_integrated_pose_rotation(integrated_pose_rotation);

    gtsam::Pose3 integrated_pose(integrated_pose_rotation, integrated_pose_translation);

    // Insert the integrated velocity pose into the graph
    if (_index > 0) {
        _initial->insert(gtsam::Symbol('x', _index), _prev_pose * integrated_pose);
        _graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            gtsam::Symbol('x', _index - 1),
            gtsam::Symbol('x', _index),
            integrated_pose,
            gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector6::Constant(0.01)
            )
        );
    }

    // clear variables used above
    reset_dvl_odometry();
    clear_dvl_timestamp_pose_vel();    
}

void Posegraph::add_dvl_factor(bool using_slerp) {
    // Get the size of the current dvl measurements
    size_t dvl_size = _curr_dvl_timestamps.size();
    size_t dvl_local_size = _curr_dvl_local_timestamps.size();

    // If any of the size is zero, assert error
    if (dvl_size == 0 && dvl_local_size == 0) {
        assert(false);
    }

    std::vector<gtsam::Rot3> interpolated_rotations;
    calculate_interpolated_rotations(using_slerp, interpolated_rotations);

    gtsam::Point3 integrated_pose_translation = gtsam::Point3(0.0, 0.0, 0.0);
    double dt_dvl = -1;
    gtsam::Matrix3 integrated_rot_matrix = gtsam::Matrix3::Identity();
    for (size_t i = 0; i < dvl_size; ++i) {
        dt_dvl = (dt_dvl == -1)
               ? _curr_dvl_timestamps[i] - _prev_keyframe_time
               : _curr_dvl_timestamps[i] - _curr_dvl_timestamps[i-1]; 
        integrated_rot_matrix = integrated_rot_matrix * interpolated_rotations[i].matrix();
        _preintegrated_velocity_measurements->integrate_measurements(
            _curr_dvl_vels[i], gtsam::Rot3(integrated_rot_matrix), dt_dvl
        );
        integrated_pose_translation += integrated_rot_matrix * _curr_dvl_vels[i] * dt_dvl;
        std::cout << "[add_dvl_factor] dt_dvl: " << dt_dvl << "\n";
        std::cout << "[add_dvl_factor] _curr_dvl_vels[i]: " << _curr_dvl_vels[i] << "\n";
        std::cout << "[add_dvl_factor] integrated_pose_translation: " << integrated_pose_translation << "\n";
    }

    gtsam::Rot3 integrated_pose_rotation;
    calculate_integrated_pose_rotation(integrated_pose_rotation);

    gtsam::Pose3 integrated_pose(integrated_pose_rotation, integrated_pose_translation);

    if (_index > 0) {
        _initial->insert(gtsam::Symbol('x', _index), _prev_pose * integrated_pose);
        _graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            gtsam::Symbol('x', _index - 1),
            gtsam::Symbol('x', _index),
            integrated_pose,
            gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector6::Constant(0.01)
            )
        );
        _graph->emplace_shared<DvlOnlyFactor>(
            gtsam::Symbol('x', _index - 1),
            gtsam::Symbol('x', _index),
            gtsam::Symbol('d', _index),
            *_preintegrated_velocity_measurements
        );
        _graph->emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
            gtsam::Symbol('d', _index - 1),
            gtsam::Symbol('d', _index),
            gtsam::imuBias::ConstantBias(gtsam::Vector3(0, 0, 0), gtsam::Vector(0, 0, 0)),
            gtsam::noiseModel::Isotropic::Sigma(6, 1e-3)
        );
    }

    reset_dvl_odometry();
    clear_dvl_timestamp_pose_vel();    
    _preintegrated_velocity_measurements->reset_integration();
}

void Posegraph::add_dvl_factor_imu_rotation() {
    int dvl_size = _curr_dvl_timestamps.size();
    assert(_curr_dvl_vels.size() == _imu_rot_list.size());

    double dt_dvl = -1;
    gtsam::Point3 integrated_pose_translation = gtsam::Point3(0.0, 0.0, 0.0);
    gtsam::Matrix3 integrated_rot_matrix = gtsam::Matrix3::Identity();

    for (size_t i = 0; i < dvl_size; ++i) {
        dt_dvl = (dt_dvl == -1)
               ? _curr_dvl_timestamps[i] - _prev_keyframe_time
               : _curr_dvl_timestamps[i] - _curr_dvl_timestamps[i-1]; 
        integrated_rot_matrix = _imu_prev_rot.matrix().inverse() * _imu_rot_list[i].matrix();
        _preintegrated_velocity_measurements->integrate_measurements_noise(
            _curr_dvl_vels[i], 
            gtsam::Rot3(integrated_rot_matrix),
            dt_dvl,
            _curr_dvl_foms[i]
        );
        integrated_pose_translation += integrated_rot_matrix.matrix() * _curr_dvl_vels[i] * dt_dvl;
    }
    gtsam::Rot3 integrated_pose_rotation = _imu_prev_rot.inverse() * _imu_rot_list.back();
    gtsam::Pose3 integrated_pose(integrated_pose_rotation, integrated_pose_translation);

    if (_index > 0) {
        _initial->insert(gtsam::Symbol('x', _index), _prev_pose * integrated_pose);
        if (_index < 10000) { // TODO REMOVE
            _graph->emplace_shared<DvlOnlyFactor>(
                gtsam::Symbol('x', _index - 1),
                gtsam::Symbol('x', _index),
                gtsam::Symbol('d', _index - 1),
                *_preintegrated_velocity_measurements
            );

            _graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                gtsam::Symbol('x', _index - 1),
                gtsam::Symbol('x', _index),
                integrated_pose,
                gtsam::noiseModel::Diagonal::Sigmas(
                    (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1).finished()
                )
            );
        } else {
            std::cout << "[add_dvl_factor_imu_rotation] no dvl factor added as index greater than 10,000\n";
        }

        _graph->emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
            gtsam::Symbol('d', _index - 1),
            gtsam::Symbol('d', _index),
            gtsam::imuBias::ConstantBias(gtsam::Vector3(0, 0, 0), gtsam::Vector(0, 0, 0)),
            gtsam::noiseModel::Isotropic::Sigma(6, 1e-1)
        );
    }

    clear_dvl_timestamp_pose_vel();    
    _curr_dvl_foms.clear();
    reset_imu_rotation();
    _preintegrated_velocity_measurements->reset_integration();
}

void Posegraph::add_velocity_factor() {
    int dvl_size = _curr_dvl_timestamps.size();
    int dvl_local_size = _curr_dvl_local_timestamps.size();

    assert(dvl_size > 0);

    gtsam::Point3 integrated_pose_translation = gtsam::Point3(0.0, 0.0, 0.0);
    double dt;
    for (size_t i = 0; i < dvl_size; ++i) {
        dt = (i == 0) 
           ? _curr_dvl_timestamps[i] - _prev_keyframe_time 
           : _curr_dvl_timestamps[i] - _curr_dvl_timestamps[i - 1];
        
        gtsam::Rot3 curr_rotation = find_current_pose_for_dvl_vel(_curr_dvl_timestamps[i]);
        gtsam::Vector3 curr_velocity = _curr_dvl_vels[i];
        gtsam::Vector3 curr_velocity_world = curr_rotation.matrix() * curr_velocity;
        integrated_pose_translation += curr_velocity_world * dt;
    } 

    gtsam::Rot3 integrated_pose_rotation;
    calculate_integrated_pose_rotation(integrated_pose_rotation);

    gtsam::Pose3 integrated_pose(integrated_pose_rotation, integrated_pose_translation);

    if (_index > 0) {
        _initial->insert(gtsam::Symbol('x', _index), _prev_pose * integrated_pose);
        _graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            gtsam::Symbol('x', _index - 1),
            gtsam::Symbol('x', _index),
            integrated_pose,
            gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector6::Constant(0.01)
            )
        );
    }

    // clear variables used above
    clear_dvl_timestamp_pose_vel();    
}

void Posegraph::add_dvl_odometry_factor(double noise) {
    gtsam::Pose3 between_pose = std::accumulate(
        std::begin(_curr_dvl_poses),
        std::end(_curr_dvl_poses),
        gtsam::Pose3(),
        [] (gtsam::Pose3 total, gtsam::Pose3 pose) {
            return total * pose;
        }
    );
    if (_index > 0) {
        _graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            gtsam::Symbol('x', _index - 1),
            gtsam::Symbol('x', _index),
            between_pose,
            gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector6::Constant(noise)
            )
        );
    }
}

void Posegraph::add_dvl_velocity(double time_stamp, gtsam::Vector3 velocity) {
    _curr_dvl_timestamps.push_back(time_stamp);
    _curr_dvl_vels.push_back(velocity);
}
void Posegraph::add_dvl_pose(double time_stamp, gtsam::Pose3 pose) {
    _curr_dvl_local_timestamps.push_back(time_stamp);
    _curr_dvl_poses.push_back(pose);
}
void Posegraph::add_dvl_rotation(double time_stamp, gtsam::Rot3 rotation) {
    _curr_dvl_local_timestamps.push_back(time_stamp);
    _curr_dvl_rotations.push_back(rotation);
}


void Posegraph::add_visual_constraint_factor(gtsam::Pose3 between_pose, double weight, int prev_idx, int curr_idx) {
    _graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        gtsam::Symbol('x', prev_idx),
        gtsam::Symbol('x', curr_idx),
        between_pose,
        gtsam::noiseModel::Gaussian::Information(
            gtsam::Matrix66::Identity() * weight * 1e-6
        )
    );
}


void Posegraph::add_prior_factor(gtsam::Pose3 initial_pose, gtsam::Vector initial_vel, double pose_noise) {
    _graph->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        gtsam::Symbol('x', 0),
        initial_pose,
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << pose_noise, pose_noise, pose_noise, pose_noise, pose_noise, pose_noise).finished()
        )
    );
    _graph->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        gtsam::Symbol('v', 0),
        initial_vel,
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << pose_noise, pose_noise, pose_noise).finished()
        )
    );
}

// Creating transforms
void Posegraph::define_transforms() {
    _T_SD = _pose_graph_params->_extrinsics.T_SD;
    _T_SB = _pose_graph_params->_extrinsics.T_BS;
    _T_W_WD = _pose_graph_params->_extrinsics.T_W_WD;
}

// Adding estimates
void Posegraph::add_simple_estimate(double dt, double noise) {
    // Create a random vector of dim 3
    gtsam::Vector3 noisy = gtsam::Vector3(_normal_distribution(_rng), _normal_distribution(_rng), _normal_distribution(_rng)) * noise;
    if (_pose_graph_params->_sensor_usage.is_imu_used) {
        gtsam::Pose3 pose_i = _initial->at<gtsam::Pose3>(gtsam::Symbol('x', _index - 1));
        gtsam::Vector3 vel_i = _initial->at<gtsam::Vector3>(gtsam::Symbol('v', _index - 1));
        gtsam::imuBias::ConstantBias bias_i = _initial->at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', _index - 1));

        // Compute translation update
        gtsam::Vector3 W_dp_S = pose_i.rotation().matrix() * (vel_i * dt + 0.5 * std::pow(dt, 2) * noisy);
        gtsam::Vector3 W_p_S = pose_i.translation() + W_dp_S;

        // Compute rotation update
        // Perturb the rotation with a small angle about a random axis
        gtsam::Vector3 axis = gtsam::Vector3(_normal_distribution(_rng), _normal_distribution(_rng), _normal_distribution(_rng));
        axis.normalize();
        double angle = _normal_distribution(_rng);
        gtsam::Rot3 R_W_S = gtsam::Rot3::Rodrigues(axis * angle) * pose_i.rotation();

        // Compute pose
        gtsam::Pose3 pred_pose(R_W_S, W_p_S);

        // Compute velocity update
        gtsam::Vector3 B_v_S = vel_i + dt * noisy;
        gtsam::Vector3 &pred_vel = B_v_S;

        // Create a bias by perturbing the prior imu bias
        gtsam::imuBias::ConstantBias pred_bias(
            _prior_imu_bias.gyroscope() + gtsam::Vector3(_normal_distribution(_rng), _normal_distribution(_rng), _normal_distribution(_rng)), 
            _prior_imu_bias.accelerometer() + gtsam::Vector3(_normal_distribution(_rng), _normal_distribution(_rng), _normal_distribution(_rng))
        );

        // Push to initial
        _initial->insert(gtsam::Symbol('x', _index), pred_pose);
        _initial->insert(gtsam::Symbol('v', _index), pred_vel);
        _initial->insert(gtsam::Symbol('b', _index), pred_bias);
        _initial->insert(gtsam::Symbol('d', _index), _prior_dvl_bias);
    }
}

void Posegraph::add_initial_estimate(gtsam::Pose3 initial_pose, gtsam::Vector3 intitial_vel) {
    _initial->insert(gtsam::Symbol('x', _index), initial_pose);
    if (_pose_graph_params->_sensor_usage.is_imu_used) {
        _initial->insert(gtsam::Symbol('b', _index), _prior_imu_bias);
    }
    if (_pose_graph_params->_sensor_usage.is_dvl_used) {
        _initial->insert(gtsam::Symbol('d', _index), _prior_dvl_bias);
    }
}

void Posegraph::add_visual_estimate(const std::vector<double> vertex) {
    Eigen::Quaterniond quaternion(vertex[8], vertex[5], vertex[5], vertex[7]);
    Eigen::Vector3d translation(vertex[2], vertex[3], vertex[4]);
    Eigen::Vector3d velocity(vertex[9], vertex[10], vertex[11]);
    Eigen::Vector3d bias_acc(vertex[12], vertex[13], vertex[14]);
    Eigen::Vector3d bias_gyro(vertex[15], vertex[16], vertex[17]);

    // Convert quaternion to Rot3
    gtsam::Rot3 R(quaternion.normalized().toRotationMatrix());

    // Convert translation to Point3
    gtsam::Point3 p(translation);

    // Create Pose 3
    gtsam::Pose3 W_visual_estimated_pose(R, p);
    gtsam::imuBias::ConstantBias visual_estimated_bias(bias_acc, bias_gyro);
    gtsam::Point3 B_visual_esimate_velocity(velocity);

    // Set previous pose to be the estimated one
    _prev_pose = W_visual_estimated_pose;

    // Push to initial
    _initial->insert(gtsam::Symbol('x', _index), W_visual_estimated_pose);
    if (_pose_graph_params->_sensor_usage.is_imu_used) {
        _initial->insert(gtsam::Symbol('v', _index), B_visual_esimate_velocity);
        _initial->insert(gtsam::Symbol('b', _index), visual_estimated_bias);
    }
    if (_pose_graph_params->_sensor_usage.is_dvl_used) {
        _initial->insert(gtsam::Symbol('d', _index), _prior_dvl_bias);
    }
}

// Working with the pose graph
void Posegraph::initialize_pose_graph() {
    // If we are not using orbslam, it is okay to keep pose from identity
    gtsam::Pose3 prior_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0));
    if (_pose_graph_params->_using_orbslam) {
        assert(false); // TODO
    }

    add_initial_estimate(prior_pose, gtsam::Vector3());
    _prev_pose = prior_pose;
    add_prior_factor(prior_pose, gtsam::Vector3(), 0.001);
}

void Posegraph::initialize_pose_graph_from_imu(gtsam::Rot3 initial_rotation) {
    gtsam::Pose3 prior_pose = gtsam::Pose3(initial_rotation, gtsam::Point3(0, 0, 0));
    add_initial_estimate(prior_pose, gtsam::Vector3());
    _prev_pose = prior_pose;
    add_prior_factor(prior_pose, gtsam::Vector3(), 0.001);
}

void Posegraph::optimize_pose_graph() {
    gtsam::LevenbergMarquardtParams lm_params;
    gtsam::LevenbergMarquardtOptimizer optimizer(*_graph, *_initial, lm_params);

    *_result = optimizer.optimize();
    std::cout << "[optimize_pose_graph] Initial error = " << _graph->error(*_initial) << "\n";
    *_initial = *_result;
    std::cout << "[optimize_pose_graph] Final error = " << _graph->error(*_result) << "\n";
}

void Posegraph::optimize_pose_graph_smoother() {
    gtsam::FactorIndices delete_slots;
    _smoother_ISAM2.update(*_graph, *_initial, _smoother_timestamps, delete_slots);
    *_result = _smoother_ISAM2.calculateEstimate();

    // Calculate error
    double error = _smoother_ISAM2.getFactors().error(*_result);
    if (std::isnan(error)) {
        std::cout << "[optimize_pose_graph_smoother] Error is NaN \n";
        assert(false);
    }
    std::cout << "[optimize_pose_graph_smoother] Final error = " << _smoother_ISAM2.getFactors().error(*_result) << "\n";

    _smoother_timestamps.clear();
    _initial->clear();
    _graph->resize(0);
}

gtsam::Rot3 Posegraph::find_current_pose_for_dvl_vel(double time_stamp) const {
    // find the difference between previous time and current time
    gtsam::Pose3 current_keyframe_pose = _prev_pose;
    double current_time_diff = time_stamp - _prev_keyframe_time;
    double min_diff = std::numeric_limits<double>::max();

    auto minimum = std::accumulate(
        std::begin(_curr_dvl_local_timestamps), 
        std::end(_curr_dvl_local_timestamps), 
        std::make_tuple<int, double, int>(0, std::numeric_limits<double>::max(), -1),
        [&time_stamp] (std::tuple<int, double, int> mins, double next_time_stamp) {
            auto [index, min_diff, min_index] = mins;
            double difference = abs(time_stamp - next_time_stamp);
            bool less = difference < min_diff;
            return std::make_tuple(
                index + 1,                     
                (less ? difference : min_diff),
                (less ? index : min_index) 
            );
        }
    );
    
    double min_time_diff;
    int min_index;
    std::tie(std::ignore, min_time_diff, min_index) = minimum;

    gtsam::Rot3 accumulated_rotation = gtsam::Rot3();
    if (current_time_diff < min_time_diff) {
        return accumulated_rotation;
    } else {
        return std::accumulate(
            std::begin(_curr_dvl_poses), 
            std::end(_curr_dvl_poses),
            accumulated_rotation,
            [] (gtsam::Rot3 accumulated_rotation, gtsam::Pose3 dvl_pose) {
                return accumulated_rotation * dvl_pose.rotation();
            }
        );
    }
}

void Posegraph::calculate_interpolated_rotations(bool is_using_slerp, std::vector<gtsam::Rot3> &interpolated_rotations) {
    if (_curr_dvl_poses.empty() || _curr_dvl_timestamps.empty()) return;
   
    gtsam::Rot3 prev_rotation = _prev_dvl_odometry_rot; 
    
    gtsam::Rot3 curr_rotation = _curr_dvl_poses[0].rotation();
    double last_local_time = _prev_dvl_odometry_time;
    
    int next_local_idx = 0;

    for (const double &ts : _curr_dvl_timestamps) {
        while (ts > _curr_dvl_local_timestamps[next_local_idx] && 
               next_local_idx < _curr_dvl_local_timestamps.size() - 1) {
            
            next_local_idx++;
            prev_rotation = curr_rotation;
            curr_rotation = _curr_dvl_poses[next_local_idx].rotation();
            last_local_time = _curr_dvl_local_timestamps[next_local_idx - 1];
        }

        double denom = _curr_dvl_local_timestamps[next_local_idx] - last_local_time;
        double dt_interp = (denom > 1e-6) ? (ts - last_local_time) / denom : 0.0;
        
        // Clamp to [0, 1] for safety
        dt_interp = std::max(0.0, std::min(1.0, dt_interp));
        interpolated_rotations.push_back(prev_rotation.slerp(dt_interp, curr_rotation));
    }
}

void Posegraph::calculate_integrated_pose_rotation(gtsam::Rot3 &integrated_pose_rotation) {
    // Calculate the integrated pose rotation by accumulating rotations
    integrated_pose_rotation = std::accumulate(
        std::begin(_curr_dvl_poses),
        std::end(_curr_dvl_poses),
        gtsam::Rot3(),
        [](gtsam::Rot3 total, gtsam::Pose3 pose) {
            return total * pose.rotation();
        }
    );
}
   
void Posegraph::reset_dvl_odometry() {
    _prev_dvl_odometry_time = _curr_dvl_local_timestamps.back();
    _prev_dvl_odometry_rot = _curr_dvl_poses.back().rotation();
}

void Posegraph::clear_dvl_timestamp_pose_vel() {
    _curr_dvl_timestamps.clear();
    _curr_dvl_vels.clear();
    _curr_dvl_poses.clear();
    _curr_dvl_local_timestamps.clear();
}


void Posegraph::reset_imu_rotation() {
    _imu_prev_rot = _imu_rot_list.back();
    _imu_rot_list.clear();
}

// Getters
double Posegraph::get_depth_measurement() const {
    return _W_measurement_z;
}

gtsam::Vector3 Posegraph::get_accelerometer_measurement() const {
    return _B_accelerometer_S;
}

gtsam::Vector3 Posegraph::get_gyroscope_measurement() const {
    return _B_gyroscope_S;
}

gtsam::Vector3 Posegraph::get_velocity_measurement() const{
    return _B_velocity_D;
}

gtsam::Vector3 Posegraph::get_position_measurement() const {
    return _W_position_C;
}

double Posegraph::get_visual_gap_time() const {
    return _visual_gap_time;
}

// Setters 
void Posegraph::set_depth_measurement(double W_measurement_z) {
    _W_measurement_z = W_measurement_z;
}

void Posegraph::set_accelerometer_measurement(gtsam::Vector3 B_accelerometer_S) {
    _B_accelerometer_S = B_accelerometer_S;
}

void Posegraph::set_gyroscope_measurement(gtsam::Vector3 B_gyroscope_S) {
    _B_gyroscope_S = B_gyroscope_S;
}

void Posegraph::set_velocity_measurement(gtsam::Vector3 B_velocity_D) {
    _B_velocity_D = B_velocity_D;
}

void Posegraph::set_position_measurement(gtsam::Vector3 W_position_C) {
    _W_position_C = W_position_C;
}

void Posegraph::set_visual_gap_time(double visual_gap_time) {
    _visual_gap_time = visual_gap_time;
}

} // namespace localization