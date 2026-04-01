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
    if (H) {
        gtsam::Matrix36 Jt;
        pose.translation(Jt);
        *H = Jt.row(2);
    }

    return (gtsam::Vector(1) << pose.z() - _measured).finished();
}
} // namespace localization