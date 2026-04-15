#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include "../inertial-sense-sdk/src/InertialSense.h"
#include "../inertial-sense-sdk/src/data_sets.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <thread>

class TimeManager
{
public:
    TimeManager(std::shared_ptr<rclcpp::Node> node) : got_first_message_(false), nh_(node) {}
    rclcpp::Time ros_time_from_start_time(const double time)
    {
        // Otherwise, estimate the IMX boot time and offset the messages
        if (!got_first_message_)
        {
            got_first_message_ = true;
            INS_local_offset_ = nh_->now().seconds() - time;
        }
        else // low-pass filter offset to account for drift
        {
            double y_offset = nh_->now().seconds() - time;
            INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
        }
        // Publish with ROS time
        return rclcpp::Time(static_cast<int64_t>((INS_local_offset_ + time) * 1e9));
    }

private:
    bool got_first_message_;
    double INS_local_offset_;
    std::shared_ptr<rclcpp::Node> nh_;
};

struct PimuStorage {
    pimu_t pimu;
    bool has_pimu = false;
    double tow_offset = -1;
};

PimuStorage pimu_storage;

#define TIME_EPSILON 0.01

/** 
 * Publishes:
 *   /imu    Imu
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("imu_node");

    TimeManager timeManager(node);

    // By default, ROS2 will pass in some ROS-specific arguments through the command line as well.
    // We only want to read our specific argument.
    std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
    if (non_ros_args.size() != 2)
    {
        std::cout << "Usage: " << non_ros_args[0] << " <port>\n";
        return 1;
    }

    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu", 1);
    if (!imu_pub) {
        return -1;
    }

    // We need to attach a global callback to all DID messages returned by the 
    // sensor so that we can sync the INS (orientation) and PIMU (linear acc and angular vel) measurements
    InertialSense is([&](InertialSense *is_ptr, p_data_t *data, int port_handle) {
        switch(data->hdr.id) {
            case DID_GPS1_POS: {
                auto gps = reinterpret_cast<gps_pos_t*>(data->ptr);
                pimu_storage.tow_offset = gps->towOffset;
                break;
            }

            case DID_PIMU: {
                pimu_storage.pimu = *reinterpret_cast<pimu_t*>(data->ptr);
                pimu_storage.has_pimu = true;
                break;
            }
            case DID_INS_2: {
                if (!pimu_storage.has_pimu) return; // We want to sync the DID_PIMU and DID_INS_2 messages.

                auto ins = reinterpret_cast<ins_2_t*>(data->ptr);

                double pimu_time_to_tow = pimu_storage.pimu.time + pimu_storage.tow_offset;

                if (std::abs(pimu_time_to_tow - ins->timeOfWeek) < TIME_EPSILON) {
                    auto time_stamp = timeManager.ros_time_from_start_time(ins->timeOfWeek);
                    const auto div = 1.0f/pimu_storage.pimu.dt;

                    sensor_msgs::msg::Imu msg;
                    msg.header.stamp    = time_stamp;
                    msg.header.frame_id = "imu_link"; 

                    msg.orientation.w = ins->qn2b[0];
                    msg.orientation.x = ins->qn2b[1];
                    msg.orientation.y = ins->qn2b[2];
                    msg.orientation.z = ins->qn2b[3];
                    
                    msg.angular_velocity.x = pimu_storage.pimu.theta[0] * div;
                    msg.angular_velocity.y = pimu_storage.pimu.theta[1] * div;
                    msg.angular_velocity.z = pimu_storage.pimu.theta[2] * div;

                    msg.linear_acceleration.x = pimu_storage.pimu.vel[0] * div;
                    msg.linear_acceleration.y = pimu_storage.pimu.vel[1] * div;
                    msg.linear_acceleration.z = pimu_storage.pimu.vel[2] * div;

                    double acc_var = 3.45e-7; // Acc noise density is 0.000588 -> var is noise^2
                    double gyro_var = 7.61e-9; // Gyro noise density is 8.72e-5 -> var is noise^2

                    msg.linear_acceleration_covariance[0] = acc_var;
                    msg.linear_acceleration_covariance[4] = acc_var;
                    msg.linear_acceleration_covariance[8] = acc_var;
                    
                    msg.angular_velocity_covariance[0] = gyro_var;
                    msg.angular_velocity_covariance[4] = gyro_var;
                    msg.angular_velocity_covariance[8] = gyro_var;

                    imu_pub->publish(msg);
                }
                pimu_storage.has_pimu = false;
                break;
            }
        }
    });

    is.Open(non_ros_args[1].c_str());


    auto pimu_registered = is.BroadcastBinaryData(DID_PIMU, 1);
    if (!pimu_registered)
    {
        return 1;
    }

    auto ins_registered = is.BroadcastBinaryData(DID_INS_2, 1);
    if (!ins_registered)
    {
        return 1;
    }
   
    // Needed to get the offset to time of week
    auto gps_registered = is.BroadcastBinaryData(DID_GPS1_POS, 1);
    if (!gps_registered)
    {
        return 1;
    }

    while (rclcpp::ok())
    {
        const auto success = is.Update();
        if (!success)
        {
            return 1;
        }
        rclcpp::spin_some(node);
        
        // Try to get the publishing rate closer to the rate ascertained by the datasheet
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
