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
	std::printf("Received:");
	for (int i = 0; i < argc; i++) {
            std::printf(" %s", argv[i]);
        }
	std::printf("\n");
        return 1;
    }

    ros::Publisher pub = nh.advertise<mrobosub_msgs::Imu>("/imu/data", 1);
    if (!pub) return -1;

    InertialSense is;
    is.Open(argv[1]);

    auto succ = is.BroadcastBinaryData(DID_PIMU, 1, [&](InertialSense* is, p_data_t* _data, int pHandle) {
        const auto data = reinterpret_cast<const pimu_t*>(_data->ptr);

        mrobosub_msgs::Imu msg;
	msg.time = data->time;
        msg.velocityA = data->vel[0];
        msg.velocityB = data->vel[1];
        msg.velocityC = data->vel[2];
	msg.thetaA = data->theta[0];
	msg.thetaB = data->theta[1];
	msg.thetaC = data->theta[2];
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

