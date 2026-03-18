#ifndef  __POSEGRAPH_NODE_H__
#define  __POSEGRAPH_NODE_H__

#include <memory>
#include <mutex>
#include <condition_variable>

#include "Posegraph.h"
#include "Parameters.h"
#include "PreintegratedVelocityHelpers.h"
#include "DvlOnlyFactor.h"
#include "BluerovBarometerFactor.h"


namespace localization {
class PosegraphNode {
public:
    PosegraphNode();
    PosegraphNode(std::string config_file);
    ~PosegraphNode();

private: // Members
    Posegraph *_posegraph;

    // Define subscribers
    int /*ros::Subscriber*/ _imu_sub;
    int /*ros::Subscriber*/ _dvl_sub;
    int /*ros::Subscriber*/ _baro_sub;
    int /*ros::Subscriber*/ _dvl_local_sub;

    // Define publishers
    int /*ros::Publisher*/ _pose_pub;
    int /*ros::Publisher*/ _dvl_local_pose_pub;
    int /*ros::Publisher*/ _path_pub;
    int /*nav_msgs::Path*/ _path_msg;

    // TODO: I think these might have to do with the MultiLevelExecutors and stuff like that.
    std::unique_ptr<int /*ros::AsyncSpinner*/> _imu_async_spinner;
    std::unique_ptr<int /*ros::AsyncSpinner*/> _async_spinner;

    int /* ros::CallbackQueue */ _imu_queue;

    // TODO: Can this be unsigned?
    int64_t _frame_count;

    // Node Handles
    // TODO: I don't think these exist for ROS2
    int /* ros::NodeHandle */ _nh;
    int /* ros::NodeHandle */ _nh_private;

    // Barometer state variables
    double _first_depth = 0.0;

    // DVL state
    int /* gtsam::PreintegratedCombinedMeasurements */ *_preintegrated_measurements_dvl;
    double _prev_dvl_time = 0.0;
    double _prev_dvl_local_time = 0.0;
    int /* gtsam::Pose3 */   _prev_dvl_local_pose;
    int /* gtsam::Rot3 */    _dvl_prev_rot;
    int /* gtsam::Vector3 */ _first_dvl_vel;

    // Keyframe states
    double _first_keyframe_time;
    double _prev_keyframe_time = 0.0;
    double _current_keyframe_time = 0.0;
    bool _is_new_keyframe = false; // Used to be new_kf_flag
    double _keyfram_gap_time;
    int /* gtsam::Pose3 */ _latest_keyframe_pose;
    int /* gtsam::Pose3 */ _latest_publish_pose; // what does this mean
    
    // ??? State 
    int /* gtsam::Pose3 */ _T_w_wd; // what does this mean
   
    // IMU State
    int /* gtsam::Rot3 */ _imu_latest_rot;
    int /* gtsam::NavState*/ _latest_imu_prop_state;
    int _imu_init_count = 0; // Can this be unsigned?
    int _imu_count = 0; // Can this be unsigned?

    std::mutex _mtx;
    std::condition_variable _keyframe_cv; 

    // Get from config file.
    bool _is_using_dvl_v2_factor = true;
    bool _is_rot_initialized = false;

    /* TODO: What is save trajectory?? */
    // ros::ServiceServer _save_trajectory_service;
    std::vector<double> _keyframe_timestamps;
    std::vector<double> _trajectory_timestamps;
    std::vector<int /*gtsam::Pose3*/> _trajectory_poses;

private: // methods
    /* TODO: What is save trajectory?? */
    // bool save_trajectory(turtlmap::save_trajectory::Request &req, turtlmap::save_trajectory::Response &res);
};
} // namespace localization

#endif //__POSEGRAPH_H__