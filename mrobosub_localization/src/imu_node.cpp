#include <cstdio>
#include <iostream>
#include "../inertial-sense-sdk/src/InertialSense.h"
#include "../inertial-sense-sdk/src/data_sets.h"

#include <ros/ros.h>
#include <mrobosub_msgs/Imu.h>

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
        return -1;
    }

    InertialSense is;
    is.Open(argv[1]);

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
        return 1;
    }

    while (ros::ok()) {
        const auto success = is.Update();
        if (!success) {
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

