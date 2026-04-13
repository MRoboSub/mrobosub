#ifndef  __POSEGRAPH_NODE_H__
#define  __POSEGRAPH_NODE_H__

#include "Posegraph.h"
#include "Parameters.h"
#include "PreintegratedVelocityHelpers.h"
#include "DvlOnlyFactor.h"
#include "BluerovBarometerFactor.h"

#include <gtsam/base/Vector.h>                   // Vector
#include <gtsam/geometry/Pose3.h>                // Pose3
#include <gtsam/geometry/Rot3.h>                 // Rot3
#include <gtsam/navigation/CombinedImuFactor.h>  // PreintegratedCombinedMeasurements
#include <gtsam/navigation/NavState.h>           // NavState

#include <boost/thread/mutex.hpp>                // boost::mutex
#include <boost/thread/condition_variable.hpp>   // boost::condition_variable
#include <boost/shared_ptr.hpp>                  // boost::shared_ptr

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace localization {
class PosegraphNode : public rclcpp::Node {
public:
    PosegraphNode();

private: // Members
    std::unique_ptr<Posegraph> _posegraph;

    // Define subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _imu_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _dvl_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _baro_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _dvl_local_sub;

    // Define publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _dvl_local_pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;

    // Define callback groups
    rclcpp::CallbackGroupType::MutuallyExclusive::SharedPtr _imu_callback_group;
    rclcpp::CallbackGroupType::MutuallyExclusive::SharedPtr _low_freq_sensor_callback_group;
    rclcpp::CallbackGroupType::MutuallyExclusive::SharedPtr _posegraph_callback_group;
    rclcpp::CallbackGroupType::MutuallyExclusive::SharedPtr _keyframe_callback_group;

    rclcpp::TimerBase::SharedPtr _posegraph_timer;
    rclcpp::TimerBase::SharedPtr _keyframe_timer;

    uint64_t _frame_count;

    // Barometer state variables
    double _first_depth;

    // DVL state
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> _preintegrated_measurements_dvl;
    // First 
    gtsam::Vector3 _first_dvl_vel;
    // Previous
    double _prev_dvl_time;
    double _prev_dvl_local_time;
    gtsam::Pose3   _prev_dvl_local_pose;
    gtsam::Rot3    _prev_dvl_rot;
    // Latest
    gtsam::Vector3 _latest_dvl_vel;
    gtsam::Pose3   _latest_dvl_pose;

    // Keyframe states
    double _first_keyframe_time;
    double _prev_keyframe_time;
    double _current_keyframe_time;
    bool _is_new_keyframe; // Used to be new_kf_flag
    double _keyfram_gap_time;
    gtsam::Pose3 _latest_keyframe_pose;
    gtsam::Pose3 _latest_publish_pose; // what does this mean
    
    // ??? State 
    gtsam::Pose3 _T_w_wd; // what does this mean
   
    // IMU State
    gtsam::Rot3 _imu_latest_rot;
    gtsam::NavState _latest_imu_prop_state;
    uint32_t _imu_init_count; 
    uint32_t _imu_count; 
    std::vector<gtsam::Vector3> _imu_init_acc;
    std::vector<gtsam::Vector3> _imu_init_rot;

    // Transforms
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    std::mutex _mtx;
    std::condition_variable _cond_var;

    // Get from config file.
    bool _is_using_dvl_v2_factor;
    bool _is_rot_initialized;

    /* TODO: What is save trajectory?? */
    // ros::ServiceServer _save_trajectory_service;
    std::vector<double> _keyframe_timestamps;
    std::vector<double> _trajectory_timestamps;
    std::vector<gtsam::Pose3> _trajectory_poses;

private: // methods
    /* TODO: What is save trajectory?? */
    // bool save_trajectory(turtlmap::save_trajectory::Request &req, turtlmap::save_trajectory::Response &res);

    void imu_callback(const mrobosub_msgs::msg::Imu::SharedPtr msg);
    void dvl_callback(const mrobosub_msgs::msg::Dvl::SharedPtr msg);
    void baro_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void dvl_local_callback(const mrobosub_msgs::msg::Dvl::SharedPtr msg);

    void main_loop();
    void kf_loop();

    void broadcast_imu_transform();
};
} // namespace localization

#endif //__POSEGRAPH_H__