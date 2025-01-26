#include <cstdio>
#include <iostream>
#include "../inertial-sense-sdk/src/InertialSense.h"
#include "../inertial-sense-sdk/src/data_sets.h"

#include <ros/ros.h>
#include <mrobosub_msgs/Imu.h>
#include <mrobosub_msgs/Magnetometer.h>
#include <mrobosub_msgs/Barometer.h>

#include <chrono>
#include <thread>

struct Defer {
    std::function<void()> f;

    Defer(std::function<void()> f) : f(std::move(f)) {}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_node");
    Defer ros_shutdown(std::function<void()>([]() {
        ros::shutdown();
    }));

    ros::NodeHandle nh;

    if (argc != 2) {
        std::printf("Usage: %s <port>\n", argv[0]);
        return 1;
    }

    ros::Publisher pub = nh.advertise<mrobosub_msgs::Imu>("/imu/data", 1);
    if (!pub) {
        return -10;
    }
    ros::Publisher mag_pub = nh.advertise<mrobosub_msgs::Magnetometer>("/imu/mag", 1);
    if (!mag_pub) {
        return -11;
    }
    ros::Publisher bar_pub = nh.advertise<mrobosub_msgs::Barometer>("/imu/bar", 1);
    if (!bar_pub) {
        return -12;
    }

    InertialSense is;
    if (!is.Open(argv[1])) {
        return -1;
    }

    auto succ = is.BroadcastBinaryData(DID_PIMU, 1, [&](InertialSense* is, p_data_t* _data, int pHandle) {
        const auto data = reinterpret_cast<const pimu_t*>(_data->ptr);

        mrobosub_msgs::Imu msg;
        const auto div = 1.0f/data->dt;
        msg.time = data->time;
        msg.linAccA = data->vel[0] * div;
        msg.linAccB = data->vel[1] * div;
        msg.linAccC = data->vel[2] * div;
        msg.angVelA = data->theta[0] * div;
        msg.angVelB = data->theta[1] * div;
        msg.angVelC = data->theta[2] * div;
        msg.dt = data->dt;
        pub.publish(msg);
    });
    if (!succ) {
        return -20;
    }

    succ = is.BroadcastBinaryData(DID_MAGNETOMETER, 1, [&](InertialSense* is, p_data_t* _data, int pHandle) {
        const auto data = reinterpret_cast<const magnetometer_t*>(_data->ptr);

        mrobosub_msgs::Magnetometer msg;
        msg.time = data->time;
        msg.magA = data->mag[0];
        msg.magB = data->mag[1];
        msg.magC = data->mag[2];
        mag_pub.publish(msg);
    });
    if (!succ) {
        return -21;
    }

    succ = is.BroadcastBinaryData(DID_BAROMETER, 1, [&](InertialSense* is, p_data_t* _data, int pHandle) {
        const auto data = reinterpret_cast<const barometer_t*>(_data->ptr);

        mrobosub_msgs::Barometer msg;
        msg.time = data->time;
        msg.bar = data->bar;
        msg.mslBar = data->mslBar;
        msg.barTemp = data->barTemp;
        msg.humidity = data->humidity;
        bar_pub.publish(msg);
    });
    if (!succ) {
        return -22;
    }

    while (ros::ok()) {
        const auto success = is.Update();
        if (!success) {
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

