#include <cstdio>
#include <iostream>
#include "../inertial-sense-sdk/src/InertialSense.h"
#include "../inertial-sense-sdk/src/data_sets.h"

#include <chrono>
#include <thread>

void data_cb(InertialSense* is, p_data_t* _data, int pHandle) {
    const auto data = reinterpret_cast<const pimu_t*>(_data->ptr);
    std::printf("Time: %.2f\t", data->time);
    std::printf("x y z: %.3f\t%.3f\t%.3f\t", data->vel[0], data->vel[1], data->vel[2]);
    std::printf("dt:%.3f\r", data->dt);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::printf("Usage: %s <port>\n", argv[0]);
        return 1;
    }
    InertialSense is;
    is.Open(argv[1]);
    auto succ = is.BroadcastBinaryData(DID_PIMU, 1, data_cb);
    std::cout << succ << std::endl;
    for (int i = 0; i < 1000; i++) {
        const auto success = is.Update();
        if (!success) {
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "end" << std::endl;
}

