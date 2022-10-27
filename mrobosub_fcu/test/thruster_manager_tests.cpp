#include <iostream>

#include <gtest/gtest.h>

#include "ThrusterManager.hpp"

using namespace std;

TEST(TestSuite, testCase1) {
    Thruster::PWMFit params{1,{1,1,1},{1,1,1}};
    ThrusterManager manager {
        {
            {0, { 0.156,  0.111, 0.085, 0, 0,     -M_PI / 4}, 1, 1500, 1500-27, 1500+27, 1100, 1900, false, params},
            {1, { 0.156, -0.111, 0.085, 0, 0,      M_PI / 4}, 1, 1500, 1500-27, 1500+27, 1100, 1900, false, params},
            {2, {-0.156,  0.111, 0.085, 0, 0, -(3*M_PI) / 4}, 1, 1500, 1500-27, 1500+27, 1100, 1900, false, params},
            {3, {-0.156, -0.111, 0.085, 0, 0,  (3*M_PI) / 4}, 1, 1500, 1500-27, 1500+27, 1100, 1900, false, params},
            {4, { 0.120,  0.218,      0, 0,  M_PI / 2, 0   }, 1, 1500, 1500-27, 1500+27, 1100, 1900, false, params},
            {5, { 0.120, -0.218,      0, 0, -M_PI / 2, 0   }, 1, 1500, 1500-27, 1500+27, 1100, 1900, false, params},
            {6, {-0.120,  0.218,      0, 0, -M_PI / 2, 0   }, 1, 1500, 1500-27, 1500+27, 1100, 1900, false, params},
            {7, {-0.120, -0.218,      0, 0,  M_PI / 2, 0   }, 1, 1500, 1500-27, 1500+27, 1100, 1900, false, params},
        }
    };

    //cout << manager.get_thrusters()[0].get_contribution().as_vector6() << endl;
    //cout << manager.get_thrusters()[4].get_contribution().as_vector6() << endl;

    EXPECT_EQ(manager.get_thrusters().size(), 8);

    auto out = manager.calculate_thrusts(Wrench<double>{4, 0, 0.5, 0.0, 0.0, 0});

    EXPECT_EQ(out.size(), 8);

    for(size_t i = 0; i < out.size(); ++i) {
        cout << "[          ] Thrusts[" << i << "] = " << out[i] << endl;
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}