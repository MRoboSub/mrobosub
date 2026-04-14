#include "BluerovBarometerFactor.h"

namespace localization {
BluerovBarometerFactor::BluerovBarometerFactor() {}

BluerovBarometerFactor::BluerovBarometerFactor(gtsam::Key key, double measured, const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor1(model, key), _measured(measured) {}

BluerovBarometerFactor::~BluerovBarometerFactor() {}

// Expected by the parent class to be evaluateError
gtsam::Vector BluerovBarometerFactor::evaluateError(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> H
) const {
    // If the H is requested, get the Jacobian of the Z-translation
    if (H) {
        gtsam::Matrix16 H_z;
        double z = pose.z(H_z);
        *H = H_z;
    }

    return (gtsam::Vector(1) << pose.z() - _measured).finished();
}
} // namespace localization