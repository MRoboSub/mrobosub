#include <cstdio>
#include <iostream>
#include <string_view>
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

class Imu_node{
    private:
        std::chrono::time_point<std::chrono::system_clock> last_update;
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Publisher mag_pub;
        ros::Publisher bar_pub;
        InertialSense is;
        const char * device_name;

        int start_imu(){
            if (!is.Open(device_name)) {
                return -1;
            }

            bool succ = is.BroadcastBinaryData(DID_PIMU, 1, [&](InertialSense* is, p_data_t* _data, int pHandle) {
                const auto data = reinterpret_cast<const pimu_t*>(_data->ptr);
                last_update = std::chrono::system_clock::now();

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

            return 0;
        }

    public:

        Imu_node(const char * device_name){
            this->device_name = device_name;
            last_update = std::chrono::system_clock::now();

            pub = nh.advertise<mrobosub_msgs::Imu>("/imu/data", 1);
            if (!pub) {
                throw -10;
            }
            mag_pub = nh.advertise<mrobosub_msgs::Magnetometer>("/imu/mag", 1);
            if (!mag_pub) {
                throw -11;
            }
            bar_pub = nh.advertise<mrobosub_msgs::Barometer>("/imu/bar", 1);
            if (!bar_pub) {
                throw -12;
            }

            int success = start_imu();
            if(success!=0){
                throw success;
            }
        }

        int run(){

            int i = 0;
            while (ros::ok()) {
                const auto success = is.Update();
                i++;
                if(i%10==0){
                    std::chrono::duration<double> elapsed = std::chrono::system_clock::now() -last_update;
                    if(elapsed.count()>5){
                        std::cout << "Haven't received any data in 5 sec :( Attempting reconnection to IMU." << std::endl;
                        is.Close();
                        start_imu(); // ignore return val cuz our solution in case still disconnected is
                        // just to try again anyway
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            return 0;
        }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_node");
    Defer ros_shutdown(std::function<void()>([]() {
        ros::shutdown();
    }));

    if (argc != 2) {
        std::printf("Usage: %s <port>\n", argv[0]);
        return 1;
    }

    try{
        Imu_node imu(argv[1]);
        imu.run();
    }
    catch(int e){
        return e;
    }
    return 0;
}

    