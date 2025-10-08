#include <cstdio>
#include <iostream>
#include "../inertial-sense-sdk/src/InertialSense.h"
#include "../inertial-sense-sdk/src/data_sets.h"

#include <rclcpp/rclcpp.hpp>
#include <mrobosub_msgs/msg/imu_ins.hpp>
#include <mrobosub_msgs/msg/imu_pimu.hpp>

#include <chrono>
#include <thread>

struct Defer
{
    std::function<void()> f;

    Defer(std::function<void()> f) : f(std::move(f)) {}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("imu_node");
    Defer ros_shutdown(std::function<void()>([]()
                                             { rclcpp::shutdown(); }));

    if (argc != 2)
    {
        std::printf("Usage: %s <port>\n", argv[0]);
        return 1;
    }

    auto pub_ins = node->create_publisher<mrobosub_msgs::msg::ImuINS>("/imu/data", 1);
    auto pub_pimu = node->create_publisher<mrobosub_msgs::msg::ImuPIMU>("/imu/data", 1);
    if (!pub_ins)
    {
        return -1;
    }

    InertialSense is;
    is.Open(argv[1]);

    auto pimu_registered = is.BroadcastBinaryData(DID_PIMU, 1, [&](InertialSense *is, p_data_t *_data, int pHandle)
                                                  {
        const auto data = reinterpret_cast<const pimu_t*>(_data->ptr);

        mrobosub_msgs::msg::ImuPIMU msg;
        // const auto div = 1.0f/data->dt; //! This is from the old code and is definitely wrong (not in any of the sdk code)

        msg.header.stamp = node->get_clock()->now(); // This is technically wrong as there is  way to estimate the time from msg->time
        msg.dt = data->dt;
        msg.dtheta.x = data->theta[0];
        msg.dtheta.y = data->theta[1];
        msg.dtheta.z = data->theta[2];
        msg.dvel.x = data->vel[0];
        msg.dvel.y = data->vel[1];
        msg.dvel.z = data->vel[2];
        pub_pimu->publish(msg); });
    if (!pimu_registered)
    {
        return 1;
    }

    auto ins_registered = is.BroadcastBinaryData(DID_INS_1, 1, [&](InertialSense *is, p_data_t *_data, int pHandle)
                                                 {
        const auto data = reinterpret_cast<const ins_1_t*>(_data->ptr);

        mrobosub_msgs::msg::ImuINS msg;

        msg.header.stamp = node->get_clock()->now(); // This is technically wrong as there is  way to estimate the time from msg->time
        msg.theta.x = data->theta[0];
        msg.theta.y = data->theta[1];
        msg.theta.z = data->theta[2];
        pub_ins->publish(msg); });
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
