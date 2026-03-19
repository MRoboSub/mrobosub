#include "BluerovBarometerFactor.h"

namespace localization {
BluerovBarometerFactor::BluerovBarometerFactor() {}

BluerovBarometerFactor::BluerovBarometerFactor(gtsam::Key key, double measured, const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor1(model, key), _measured(measured) {}

BluerovBarometerFactor::~BluerovBarometerFactor() {}

gtsam::Vector BluerovBarometerFactor::evaluate_error(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> H
) const {
    gtsam::Matrix tH;
    gtsam::Vector ret = 
        (gtsam::Vector(1) << (pose.translation(tH).z() - _measured)).finished();

    if (H) {
        *H = tH.block<1, 6>(2, 0);
    }

    double error = (pose.z() - _measured);

    return (gtsam::Vector() << error).finished();
}
} // namespace localization