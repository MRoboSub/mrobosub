#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include "../inertial-sense-sdk/src/InertialSense.h"
#include "../inertial-sense-sdk/src/data_sets.h"

#include <rclcpp/rclcpp.hpp>
#include <mrobosub_msgs/msg/imu_ins.hpp>
#include <mrobosub_msgs/msg/imu_pimu.hpp>
#include <mrobosub_msgs/msg/imu.hpp>

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

void global_message_callback(void *ctx, p_data_t *data, port_handle_t port) {
    auto node = static_cast<rclcpp::Node *>(ctx);
}


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

    auto imu_pub = node->create_publisher<mrobosub_msgs::msg::Imu>("/imu", 1);
    if (!imu_pub) {
        return -1;
    }

    // We need to attach a global callback to all DID messages returned by the 
    // sensor so that we can sync the INS (rotation) and PIMU (linear acc and angular vel) measurements
    InertialSense is([&](InertialSense *is_ptr, p_data_t *data, int port_handle) {
        switch(data->hdr.id) {
            case DID_GPS1_POS: {
                auto gps = reinterpret_cast<gps_pos_t*>(data->ptr);
                pimu.tow_offset = gps->towOffset;
                break;
            }

            case DID_PIMU: {
                pimu_storage.pimu = reinterpret_cast<pimu_t*>(data->ptr);
                pimu_storage.has_pimu = true;
                break;
            }

            case DID_INS_1: {
                if (pimu_storage.has_pimu) return; // We want to sync the DID_PIMU and DID_INS_1 messages.

                auto ins = reinterpret_cast<ins_1_t*>(data->ptr);

                double pimu_time_to_tow = pimu_storage.pimu.time + pimu_storage.tow_offset;

                if (std::abs(pimu_time_to_tow - ins->timeOfWeek) < TIME_EPSILON) {
                    auto time_stamp = timeManager.ros_time_from_start_time(ins->timeOfWeek);
                    const auto div = 1.0f/data->dt;

                    mrobosub_msgs::msg::Imu msg;
                    msg.header.stamp = time_stamp;
                    msg.dt = data->dt;
                    msg.angular_velocity.x = data->theta[0] * div;
                    msg.angular_velocity.y = data->theta[1] * div;
                    msg.angular_velocity.z = data->theta[2] * div;
                    msg.linear_acceleration.x = data->vel[0] * div;
                    msg.linear_acceleration.y = data->vel[1] * div;
                    msg.linear_acceleration.z = data->vel[2] * div;
                    msg.theta.x = data->theta[0];
                    msg.theta.y = data->theta[1];
                    msg.theta.z = data->theta[2];
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

    auto ins_registered = is.BroadcastBinaryData(DID_INS_1, 1);
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
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
