#include <cstdio>
#include <iostream>
#include "../inertial-sense-sdk/src/InertialSense.h"
#include "../inertial-sense-sdk/src/data_sets.h"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <mrobosub_msgs/Dvl.h>

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

    ros::Publisher time_pub = nh.advertise<std_msgs::Float64>("/imu/time", 1);
    if (!time_pub) return -1;
    ros::Publisher vel_pub = nh.advertise<mrobosub_msgs::Dvl>("/imu/vel", 1);
    if (!vel_pub) return -1;
    ros::Publisher theta_pub = nh.advertise<mrobosub_msgs::Dvl>("/imu/theta", 1);
    if (!theta_pub) return -1;
    ros::Publisher dt_pub = nh.advertise<std_msgs::Float64>("/imu/dt", 1);
    if (!dt_pub) return -1;

    InertialSense is;
    is.Open(argv[1]);

    auto succ = is.BroadcastBinaryData(DID_PIMU, 1, [&](InertialSense* is, p_data_t* _data, int pHandle) {
        const auto data = reinterpret_cast<const pimu_t*>(_data->ptr);

        std_msgs::Float64 time;
        time.data = data->time;
        time_pub.publish(time);

        mrobosub_msgs::Dvl vel;
        vel.velocityA = data->vel[0];
        vel.velocityB = data->vel[1];
        vel.velocityC = data->vel[2];
        vel_pub.publish(vel);

        mrobosub_msgs::Dvl theta;
        theta.velocityA = data->theta[0];
        theta.velocityB = data->theta[1];
        theta.velocityC = data->theta[2];
        theta_pub.publish(theta);

        std_msgs::Float64 dt;
        dt.data = data->dt;
        dt_pub.publish(dt);
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

