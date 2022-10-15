#include <iostream>

#include <gtest/gtest.h>

#include "ThrusterManager.hpp"

using namespace std;

TEST(TestSuite, testCase1) {
    ThrusterManager manager {
        {
            {0, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 1100, 1900, false, {1, 1, 1}}
        }
    };

    ASSERT_EQ(manager.get_thrusters().size(), 1);
    manager.calculate_thruster_matrices();

    auto out = manager.calculate_pwm_output(Screw<double>{5.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    ASSERT_EQ(out.size(), 1);

    cout << "[          ] PWM[0] = " << out[0] << endl;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}