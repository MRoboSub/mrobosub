#include "Posegraph.h"

#include <limits>
#include <numeric>
#include <utility>

namespace localization {
    // Ctor + Dtor
    Posegraph::Posegraph() 
        : _index(0)
        , _graph(new gtsam::NonlinearFactorGraph())
        , _initial(new gtsam::Values())
        , _result(new gtsam::Values())
        , _preintegrated_velocity_measurements(new PreintegratedVelocityMeasurementsDvlOnly())
        , _prior_imu_bias(gtsam::imuBias::ConstantBias(gtsam::Vector3(0.01, 0.01, 0.01), gtsam::Vector3(0, 0, 0))) {
    }

    Posegraph::~Posegraph() {}

    void Posegraph::initialize_parameters(std::shared_ptr<PosegraphNode> node) {
        _pose_graph_params = std::make_unique<Parameters>(node);
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

    gtsam::Rot3 Posegraph::find_current_pose_for_dvl_vel(double time_stamp) const {
        // find the difference between previous time and current time
        gtsam::Pose3 current_keyframe_pose = _prev_pose;
        double current_time_diff = time_stamp - _prev_keyframe_time;
        double min_diff = std::numeric_limits<double>::max();

        auto minimum = std::accumulate(
            std::begin(_current_dvl_local_timestamps), 
            std::end(_current_dvl_local_timestamps), 
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
                std::begin(_current_dvl_poses), 
                std::end(_current_dvl_poses),
                accumulated_rotation,
                [] (gtsam::Rot3 accumulated_rotation, gtsam::Pose3 dvl_pose) {
                    return accumulated_rotation * dvl_pose.rotation();
                }
            );
        }
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