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

void imu_callback(const inertial_sense_ros::did_ins1& msg){
    ROS_INFO("got reading!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_pub");
    Defer ros_shutdown(std::function<void()>([]() {
        ros::shutdown();
    }));

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/did_ins1", 1, imu_callback);

    ros::spin();

    // ros::Publisher pub = nh.advertise<inertial_sense_ros/did_ins1>("/did_ins1", 1);
    // if (!pub) {
    //     return -1;
    // }
}