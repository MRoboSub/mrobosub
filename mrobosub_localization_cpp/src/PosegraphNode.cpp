#include "PosegraphNode.h"
#include "mrobosub_msgs/msg/Imu.hpp"
#include "mrobosub_msgs/msg/Dvl.hpp"
#include "std_msgs/msg/float64.hpp"

namespace localization {

PosegraphNode::PosegraphNode() 
    : Node("posegraph")
    , _first_depth(0.0)
    , _prev_dvl_time(0.0)
    , _prev_dvl_local_time(0.0)
    , _prev_keyframe_time(0.0)
    , _current_keyframe_time(0.0)
    , _is_new_keyframe(false)
    , _imu_init_count(0)
    , _imu_count(0)
    , _is_using_dvl_v2_factor(true)
    , _is_rot_initialized(false) {

    RCLCPP_INFO(this->get_logger(), "'posegraph' node has started"); 

    // Create posegraph variables
    _posegraph = std::make_unique<Posegraph>(); 
    _preintegrated_measurements_dvl = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
        boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
            _posegraph->_preintegrated_measurement_params
        ),
        _posegraph->_prior_imu_bias
    );
    _keyframe_gap_time = _posegraph->_pose_graph_params->_keyframe_gap_time;

    // Set up the transform
    _tf_broadcaster = std::make_unique<tf2_ros2::TransformBroadcaster>(*this);

    // Set up callback groups
    _imu_callback_group = this->create_callback_group(rclcpp::CallbackGroup::MutuallyExclusive);
    _low_freq_sensor_callback_group = this->create_callback_group(rclcpp::CallbackGroup::MutuallyExclusive);
    _posegraph_callback_group = this->create_callback_group(rclcpp::CallbackGroup::MutuallyExclusive);
    _keyframe_callback_group = this->create_callback_group(rclcpp::CallbackGroup::MutuallyExclusive);

    // Create ROS subscriptions
    auto imu_options = rclcpp::SubscriptionOptions();
    imu_options.callback_group = _imu_callback_group;

    auto low_freq_options = rclcpp::SubscriptionOptions();
    low_freq_options.callback_group = _low_freq_callback_group;

    _imu_sub = this->create_subscription<mrobosub_msgs::msg::Imu>(
        "/imu", 10,
        std::bind(&PosegraphNode::imu_callback, this, std::placeholders::_1),
        imu_options
    );

    _dvl_sub = this->create_subscription<mrobosub_msgs::msg::Dvl>(
        "/dvl", 10,
        std::bind(&PosegraphNode::dvl_callback, this, std::placeholders::_1),
        low_freq_options
    );

    _baro_sub = this->create_subscription<std_msgs::msg::Float64>(
        "/depth", 10,
        std::bind(&PosegraphNode::baro_callback, this, std::placeholders::_1),
        low_freq_options
    );

    _dvl_local_sub = this->create_subscription<mrobosub_msgs::msg::Dvl>(
        "/dvl_local", 10,
        std::bind(&PosegraphNode::dvl_local_callback, this, std::placeholders::_1),
        low_freq_options
    );

    // Create ROS publishers
    _pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", 1000);
    _dvl_local_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/dvl_local_pose", 1000);
    _path_pub = this->create_publisher<nav_msgs::msg::Path>("/localization/path", 1000);

    // Create timers
    _posegraph_timer = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&PosegraphNode::main_loop, this),
        _posegraph_callback_group
    );

    // TODO: Should this be on a separate keyframe_callback_group?
    _keyframe_timer = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&PosegraphNode::kf_loop, this),
        _keyframe_callback_group
    );
}

void PosegraphNode::main_loop() {
    if (_is_new_keyframe) {
        _is_new_keyframe = false;
        _posegraph->add_barometric_factor(_posegraph->get_depth_measurement(), 0.1, _posegraph->_index);
        if (_is_using_dvl_v2_factor) {
            _posegraph->add_dvl_factor(true); // using slerp true
        } else {
            throw std::runtime_error("Not implemented yet!");
        }

        if (_posegraph->_index > 0) {
            _posegraph->optimize_pose_graph();
            _latest_keyframe_pose = _posegraph->_result->at<gtsam::Pose3>(gtsam::Symbol('x', _posegraph->_index));
            _latest_publish_pose = _latest_keyframe_pose;
            _posegraph->_prev_pose = _latest_keyframe_pose;
            if (_is_using_dvl_v2_factor) {
                _posegraph->_prior_dvl_bias = _posegraph->_result->at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('d', _posegraph->_index));
                _posegraph->_preintegrated_velocity_measurements->reset_integration_and_bias(_posegraph->_prior_dvl_bias);
            } else {
                throw std::runtime_error("Not implemented yet!");
            }
        }

        _posegraph->_prev_keyframe_time = _current_keyframe_time;
        _posegraph->_index++;
        _posegraph->_initial->insert(gtsam::('d', _posegraph->_index), _posegraph->_prior_dvl_bias);
    }
} 

void PosegraphNode::kf_loop() {
    _posegraph->_index++;

    _keyframe_timestamps.push_back(_current_keyframe_time);
    double time = _current_keyframe_time - _first_keyframe_time;
    _posegraph->_initial->insert(gtsam::Symbol('d', _posegraph->_index), _posegraph->_prior_dvl_bias);
    _posegraph->_initial->insert(gtsam::Symbol('b', _posegraph->_index), _posegraph->_prior_imu_bias);

    // Add smoothing timestamps
    _posegraph->_smoother_timestamps[gtsam::Symbol('d', _posegraph->_index)] = time;
    _posegraph->_smoother_timestamps[gtsam::Symbol('b', _posegraph->_index)] = time;
    _posegraph->_smoother_timestamps[gtsam::Symbol('x', _posegraph->_index)] = time;

    // Start to process new kf
    if (_posegraph->_index < 10000) {
        _posegraph->add_barometric_factor(_posegraph->get_depth_measurement(), 0.005, _posegraph->_index);
    }

    if (_posegraph->_index > 1) {
        _posegraph->_initial->insert(
            gtsam::Symbol('v', _posegraph->_index),
            _posegraph->_result->at<gtsam::Vector3>(
                gtsam::Symbol('v', _posegraph->_index-1),
            )
        );
    } else {
        _posegraph->_initial->insert(
            gtsam::Symbol('v', _posegraph->_index),
            gtsam::Vector3(0, 0, 0)
        );
    }
    _posegraph->_smoother_timestamps[gtsam::Symbol('v', _posegraph->_index)] = time;

    _posegraph->add_imu_factor();
    if (_is_using_dvl_v2_factor) {
        _posegraph->add_dvl_factor_imu_rotation();
    } else {
        throw std::runtime_error("Not implemented yet!");
    }

    if (_posegraph->_index > 0) {
        if (!_posegraph->_pose_graph_params->_using_smoother) {
            _posegraph->optimize_pose_graph();
        } else {
            _posegraph->optimize_pose_graph_smoother();
        }

        _posegraph->_prior_imu_bias = _posegraph->_result->at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', _posegraph->_index));
        _posegraph->_preintegrated_measurements->resetIntegrationAndSetBias(_posegraph->_prior_imu_bias);
        _preintegrated_measurements_dvl->resetIntegrationAndSetBias(_posegraph->_prior_imu_bias);

        _latest_keyframe_pose  = _posegraph->_result->at<gtsam::Pose3>(gtsam::Symbol('x', _posegraph->_index));
        _latest_publish_pose = _latest_keyframe_pose;
        _latest_dvl_pose = _latest_keyframe_pose;
        _latest_dvl_vel = _posegraph->_result->at<gtsam::Vector3>(gtsam::Symbol('v', _posegraph->_index));
        _posegraph->_prev_pose = _latest_keyframe_pose;

        if (_is_using_dvl_v2_factor) {
            _posegraph->_prior_dvl_bias = _posegraph->_result->at<gtsam::Vector3>(gtsam::Symbol('d', _posegraph->_index))
            gtsam::Point3 dvl_bias_point3 = _posegraph->_prior_dvl_bias.accelerometer();
            dvl_bias_point3 = _posegraph->_prev_pose.rotation() * dvl_bias_point3;
            _posegraph->_preintegrated_velocity_measurements->reset_integration_and_bias();
        } else {
            throw std::runtime_error("Not implemented yet");
        }
    }

    _posegraph->_prev_keyframe_time = _current_keyframe_time;
    _is_new_keyframe = false;
}

void PosegraphNode::imu_callback(const mrobosub_msgs::msg::Imu::SharedPtr msg) {
    // This function is guaranteed to be mutually exclusive to all other 
    // functions since it is on a separate timer.

    if (!_is_rot_initialized) {
        _imu_init_count++;
        // get the rotation from the first imu message
        gtsam::Rot3 imu_rot = gtsam::Rot3(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
        _imu_init_rot.push_back(imu_rot.xyz());
    } else {
        gtsam::Vector3 imu_acc = gtsam::Vector3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
        gtsam::Vector3 imu_gyro = gtsam::Vector3(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
        _imu_latest_rot = gtsam::Rot3(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
        _posegraph->_preintegrated_measurements->integrateMeasurement(imu_acc, imu_gyro, _posegraph->_pose_graph_params->_imu_params.dt_imu);
        _preintegrated_measurements_dvl->integrateMeasurement(imu_acc, imu_gyro, _posegraph->_pose_graph_params->_imu_params.dt_imu);
        _imu_count++;

        if (_imu_count >= 5) {
            _latest_imu_prop_state = _preintegrated_measurements_dvl->predict(gtsam::NavState(
                _latest_dvl_pose,
                _latest_dvl_pose.rotation() * _latest_dvl_vel,
                _posegraph->_prior_imu_bias
            ))

            gtsam::Pose3 latest_pose = _latest_imu_prop_state.pose();;
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = imu_msg->header.stamp;
            pose_msg.header.frame_id = "NED_imu";
            pose_msg.pose.position.x = latest_pose.translation().x();
            pose_msg.pose.position.y = latest_pose.translation().y();
            pose_msg.pose.position.z = latest_pose.translation().z();
            pose_msg.pose.orientation.w = latest_pose.rotation().toQuaternion().w();
            pose_msg.pose.orientation.x = latest_pose.rotation().toQuaternion().x();
            pose_msg.pose.orientation.y = latest_pose.rotation().toQuaternion().y();
            pose_msg.pose.orientation.z = latest_pose.rotation().toQuaternion().z();
            _pose_pub.publish(pose_msg);

            // Transform
            broadcast_imu_transform();
            _imu_count = 0;
        }
    }
}

void PosegraphNode::dvl_callback(const mrobosub_msgs::msg::Dvl::SharedPtr msg) {
    if(!_is_rot_initialized) return;

    const std::unique_lock<std::mutex> lock(_mtx);
    double dt_dvl;
    double dvl_current_time = msg->header.stamp.toSec();
    gtsam::Vector3 dvl_vel = _posegraph->_T_SD.block(0, 0, 3, 3) *
                            gtsam::Vector3(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    double fom = msg->fom;
    bool is_valid = msg->velocity_valid;

    // Handle initial measurement
    if (_prev_dvl_time == 0.0) {
        _prev_dvl_time = dvl_current_time;
        dt_dvl = dvl_current_time - _current_keyframe_time;
        _prev_dvl_rot = _imu_latest_rot;
        _first_dvl_vel = dvl_vel;

        _posegraph->_initial->insert(
            gtsam::Symbol('v', _posegraph->_index),
            _first_dvl_vel
        );
        _posegraph->_graph->add(gtsam::PriorFactor<gtsam::Vector3>(
            gtsam::Symbol('v', 0),
            _first_dvl_vel,
            gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(3) << 0.1, 0.1, 0.1).finished()
            )
        ))

        // Update for preintegrated_measurements_dvl
        _latest_dvl_vel = _first_dvl_vel;
        _latest_dvl_pose = _latest_publish_pose;
    } else {
        dt_dvl = dvl_current_time - _prev_dvl_time;
    }

    if (fom >= _posegraph->_pose_graph_params->_dvl_params.fom_threshold && !is_valid) {
        gtsam::NavState imu_prop_state = _preintegrated_measurements_dvl->predict(
            gtsam::NavState(_latest_dvl_pose),
            _latest_dvl_pose.rotation() * _latest_dvl_vel,
            _posegraph->_prior_imu_bias
        )

        gtsam::Vector3 old_dvl_vel = dvl_vel;
        dvl_vel = imu_prop_state.pose().rotation().inverse() * imu_prop_state.velocity();

        // Compute difference vector
        gtsam::Vector3 dvl_vel_diff = dvl_vel - old_dvl_vel;
        
        RCLCPP_WARN(this->get_logger(), "Invalid DVL Measurement with diff", dvl_vel_diff); 
    }

    // Communicate current dvl to posegraph
    _posegraph->add_dvl_velocity(dvl_current_time, dvl_vel);

    gtsam::Rot3 d_rot = _prev_dvl_rot.inverse() * _imu_latest_rot;
    gtsam::Point3 d_position = d_rot.matrix() * gtsam::Point3(
        dvl_vel.x() * dt_dvl, 
        dvl_vel.y() * dt_dvl, 
        dvl_vel.z() * dt_dvl, 
    );
    gtsam::Pose3 d_pose(d_rot, d_position);
    _latest_publish_pose = _latest_publish_pose * d_pose;

    // Update latest
    _latest_dvl_vel = dvl_vel;
    _latest_dvl_pose = _latest_publish_pose;
    _preintegrated_measurements_dvl->resetIntegration();

    _posegraph->_curr_dvl_foms.push_back(fom * 4); // TDOO: multiply fom by 4 to get actual?
    _posegraph->_imu_rot_list.push_back(_imu_latest_rot);
    _prev_dvl_time = dvl_current_time;

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frome_id = "NED_imu";
    pose_msg.pose.position.x = _latest_publish_pose.translation().x();
    pose_msg.pose.position.y = _latest_publish_pose.translation().y();
    pose_msg.pose.position.z = _latest_publish_pose.translation().z();
    pose_msg.pose.orientation.w = _latest_publish_pose.rotation().toQuaternion().w();
    pose_msg.pose.orientation.x = _latest_publish_pose.rotation().toQuaternion().x();
    pose_msg.pose.orientation.y = _latest_publish_pose.rotation().toQuaternion().y();
    pose_msg.pose.orientation.z = _latest_publish_pose.rotation().toQuaternion().z();
    _pose_pub.publish(pose_msg);

    broadcast_imu_transform();

    geometry_msgs::msg::TransformStamped world_tf_msg;
    world_tf_msg.header.stamp = msg->header.stamp;
    world_tf_msg.header.frame_id = "world";
    world_tf_msg.child_frame_id = "NED_imu";
    world_tf_msg.transform.translation.x = 0.0;
    world_tf_msg.transform.translation.y = 0.0;
    world_tf_msg.transform.translation.z = 0.0;
    world_tf_msg.transform.rotation.w = 0.0;
    world_tf_msg.transform.rotation.x = 1.0;
    world_tf_msg.transform.rotation.y = 0.0;
    world_tf_msg.transform.rotation.z = 0.0;
    br.sendTransform(world_tf_msg);

    _prev_dvl_rot = _imu_latest_rot;
}

void PosegraphNode::baro_callback() {
    std::unique_lock<std::mutex> lock(_mtx);
    if (!_is_rot_initialized) {
        _cond_var.wait(lock, [_imu_init_rot] { return _imu_init_rot.size() < 5; });
        gtsam::Vector3 imu_rot_mean(0.0, 0.0, 0.0);
        int latest_imu_rot_number = 5;
        for (int i = _imu_init_rot.size() - latest_imu_rot_number; i < _imu_init_rot.size(); ++i) {
            imu_init_mean += _imu_init_rot[i];
        }
        imu_rot_mean /= latest_imu_rot_number;

        gtsam::Rot3 imu_rot = gtsam::Rot3::RzRyRx(imu_rot_mean[0], imu_rot_mean[1], imu_rot_mean[2]);
        _imu_latest_rot = imu_rot;
        _posegraph->_imu_prev_rot = imu_rot;
        _imu_init_rot.clear();

        _posegraph->initialize_pose_graph_from_imu(imu_rot);
        _current_keyframe_time = msg->header.stamp.toSec();
        if (_posegraph->_pose_graph_params->_using_smoother) {
            _posegraph->_smoother_timestamps[gtsam::Symbol('x', _posegraph->_index)] = 0.0;
            _posegraph->_smoother_timestamps[gtsam::Symbol('v', _posegraph->_index)] = 0.0;
            _posegraph->_smoother_timestamps[gtsam::Symbol('b', _posegraph->_index)] = 0.0;
            _posegraph->_smoother_timestamps[gtsam::Symbol('d', _posegraph->_index)] = 0.0;
        }

        _first_keyframe_time = _current_keyframe_time;
        _posegraph->_prev_keyframe_time = _current_keyframe_time;
        _keyframe_timestamps.push_back(_current_keyframe_time);
        _latest_keyframe_pose = _posegraph->_initial->at<gtsam::Pose3>(gtsam::Symbol('x', 0));
        _latest_publish_pose = _posegraph->_initial->at<gtsam::Pose3>(gtsam::Symbol('x', 0));

        _is_rot_initialized = true;

        // TODO: what is this calculation???
        double depth = (msg->fluid_pressure - _posegraph->_pose_graph_params->_barometer_params.atmospheric_pressure) * 100 / 9.81 * 997.0;
        _first_depth = depth;
        return;
    }
        
    double depth = (msg->fluid_pressure - _posegraph->_pose_graph_params->_barometer_params.atmospheric_pressure) * 100 / 9.81 * 997.0;
    if (_first_depth == 0.0) {
        _first_depth = depth;
    }
    _posegraph->set_depth_measurement(depth - _first_depth);

    double baro_current_time = msg->header.stamp.toSec();
    if (baro_current_time - _current_keyframe_time > _keyframe_gap_time) {
        if (_posegraph->_pose_graph_params->_using_pseudo_dvl) {
            gtsam::NavState pseudo_state = _preintegrated_measurements_dvl->predict(
                gtsam::NavState(
                    _latest_dvl_pose,
                    _latest_dvl_pose.rotation() * _latest_dvl_vel
                ),
                _posegraph->_prior_imu_bias
            );

            gtsam::Vector3 pseudo_dvl_vel = pseudo_state.pose().rotation().inverse() * pseudo_state.velocity();

            _posegraph->add_dvl_velocity(baro_current_time, pseudo_dvl_vel);
            _posegraph->_imu_rot_list.push_back(_imu_latest_rot);
            _posegraph->_curr_dvl_foms.push_back(0.02);
            _prev_dvl_time = baro_current_time;
        }
        _current_keyframe_time = baro_current_time;
        _new_keyframe_flag = true;
        _cond_var.notify_one();
    }
}

void PosegraphNode::dvl_local_callback(const mrobosub_msgs::msg::Dvl::SharedPtr msg) {
    std::unique_lock<std::mutex> lock(_mtx);
    double dt_dvl_local;
    double dvl_local_current_time = msg->header.stamp.toSec();
    gtsam::Pose3 dvl_local_pose = gtsam::Pose3(
        gtsam::Rot3(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        ),
        gtsam::Point3(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        )
    );

    if (_prev_dvl_local_time == 0.0) {
        _prev_dvl_local_time = dvl_local_current_time;
        dt_dvl_local = dvl_local_current_time - _current_keyframe_time;
        _prev_dvl_local_pose = dvl_local_pose;
        _T_w_wd = _latest_publish_pose * gtsam::Pose3(_posegraph->_T_SD) * _prev_dvl_local_pose.inverse();
    } else {
        dt_dvl_local = dvl_local_current_time - _prev_dvl_local_time;
    }

    Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
    gtsam:Rot3 dvl_pose = gtsam::Pose3(
        gtsam::Rot3(q.normalized().toRotationMatrix())
        gtsam::Point3(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        )
    );

    gtsam::Pose3 d_pose = _prev_dvl_local_pose.inverse() * dvl_pose;
    _posegraph->add_dvl_pose(dvl_local_current_time, d_pose);
    dvl_local_pose = _T_w_wd * _dvl_local_pose * gtsam::Pose3(_posegraph->_T_SD).inverse();

    gtsam::Pose3 T_sensor_zed = gtsam::Pose3(
        gtsam::Rot3(0, 0, 0, 0),
        gtsam::Point3(0, 0, 0)
    )
    dvl_local_pose = dvl_local_pose * T_sensor_zed.inverse();

    geometry_msgs::msg::TransformStamped dvl_local_pose_msg;
    dvl_local_pose_msg.header.stamp = msg->header.stamp;
    dvl_local_pose_msg.header.frame_id = "NED_imu";
    dvl_local_pose_msg.pose.position.x = dvl_local_pose.translation().x();
    dvl_local_pose_msg.pose.position.y = dvl_local_pose.translation().y();
    dvl_local_pose_msg.pose.position.z = dvl_local_pose.translation().z();
    dvl_local_pose_msg.pose.orientation.w = dvl_local_pose.rotation().toQuaternion().w();
    dvl_local_pose_msg.pose.orientation.x = dvl_local_pose.rotation().toQuaternion().x();
    dvl_local_pose_msg.pose.orientation.y = dvl_local_pose.rotation().toQuaternion().y();
    dvl_local_pose_msg.pose.orientation.z = dvl_local_pose.rotation().toQuaternion().z();
    _dvl_local_pose_publisher.publish(dvl_local_pose_msg);

    _prev_dvl_local_time = dvl_local_current_time;
    _prev_dvl_local_pose = dvl_pose;
}

void PosegraphNode::broadcast_imu_transform() {
    // Back calculate where the base_link must be based off of where the sensor says it is.
    // T_sensor_zed is the static offset of the sensor relative to the robot's origin
    // TODO!
    gtsam::Pose3 T_sensor_zed = gtsam::Pose3(
        gtsam::Rot3(0, 0, 0, 0),
        gtsam::Point3(0, 0, 0)
    )
    gtsam::Pose3 latest_publish_pose_base_link = latest_pose * T_sensor_zed.inverse();

    geometry_msgs::msg::TransformStamped pose_tf_msg;
    pose_tf_msg.header.stamp = imu_msg->header.stamp;
    pose_tf_msg.header.frame_id = "NED_imu";
    pose_tf_msg.child_frame_id = "base_link";
    pose_tf_msg.pose.position.x = latest_pose.translation().x();
    pose_tf_msg.pose.position.y = latest_pose.translation().y();
    pose_tf_msg.pose.position.z = latest_pose.translation().z();
    pose_tf_msg.pose.orientation.w = latest_pose.rotation().toQuaternion().w();
    pose_tf_msg.pose.orientation.x = latest_pose.rotation().toQuaternion().x();
    pose_tf_msg.pose.orientation.y = latest_pose.rotation().toQuaternion().y();
    pose_tf_msg.pose.orientation.z = latest_pose.rotation().toQuaternion().z();
    _tf_broadcaster.sendTransform(pose_tf_msg);
}



} // namespace localization