#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include "../inertial-sense-sdk/src/InertialSense.h"
#include "../inertial-sense-sdk/src/data_sets.h"

#include <rclcpp/rclcpp.hpp>
#include <mrobosub_msgs/msg/imu_ins.hpp>
#include <mrobosub_msgs/msg/imu_pimu.hpp>

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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("imu_node");

    TimeManager timeManager(node);

    // By default, ROS2 will pass in some ROS-specific arguments through the command line as well.
    // We only want to read our specific argument.

    std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);

    std::cout << "The size of non_ros_args was " << non_ros_args.size() << "\n";    
    for (const auto& arg : non_ros_args) {
        std::cout << arg << "\n";
    }

    if (non_ros_args.size() != 2)
    {
        std::cout << "Usage: " << non_ros_args[0] << " <port>\n";
        return 1;
    }

    auto pub_ins = node->create_publisher<mrobosub_msgs::msg::ImuINS>("/imu_INS", 1);
    auto pub_pimu = node->create_publisher<mrobosub_msgs::msg::ImuPIMU>("/imu_PIMU", 1);
    if (!pub_ins || !pub_pimu)
    {
        return -1;
    }

    InertialSense is;
    is.Open(argv[1]);

    auto pimu_registered = is.BroadcastBinaryData(
        DID_PIMU, 
        1, 
        [&](InertialSense *is, p_data_t *_data, int pHandle)
        {
            const auto data = reinterpret_cast<const pimu_t*>(_data->ptr);

            mrobosub_msgs::msg::ImuPIMU msg;
            const auto div = 1.0f/data->dt;

            msg.header.stamp = timeManager.ros_time_from_start_time(data->time);
            msg.dt = data->dt;
            msg.angular_velocity.x = data->theta[0] * div;
            msg.angular_velocity.y = data->theta[1] * div;
            msg.angular_velocity.z = data->theta[2] * div;
            msg.linear_acceleration.x = data->vel[0] * div;
            msg.linear_acceleration.y = data->vel[1] * div;
            msg.linear_acceleration.z = data->vel[2] * div;
            pub_pimu->publish(msg); 
        }
    );

    if (!pimu_registered)
    {
        return 1;
    }

    auto ins_registered = is.BroadcastBinaryData(
        DID_INS_1, 
        1, 
        [&](InertialSense *is, p_data_t *_data, int pHandle)
        {
            const auto data = reinterpret_cast<const ins_1_t*>(_data->ptr); 

            mrobosub_msgs::msg::ImuINS msg;

            msg.header.stamp = timeManager.ros_time_from_start_time(data->timeOfWeek);
            msg.theta.x = data->theta[0];
            msg.theta.y = data->theta[1];
            msg.theta.z = data->theta[2];
            pub_ins->publish(msg); 
        }
    );

    if (!ins_registered)
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
