#include "DvlOnlyFactor.h"

namespace localization {
DvlOnlyFactor::DvlOnlyFactor() {}

DvlOnlyFactor::DvlOnlyFactor(
    gtsam::Key pose_i, 
    gtsam::Key pose_j, 
    gtsam::Key vbias_i,
    const PreintegratedVelocityMeasurementsDvlOnly &pvm)   
    : gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::imuBias::ConstantBias>(
        gtsam::noiseModel::Gaussian::Covariance(
            pvm._preintegrated_measured_covariance
        ),
        pose_i,
        pose_j,
        vbias_i
    )
    , _pvm(pvm) {}

DvlOnlyFactor::~DvlOnlyFactor() {}

gtsam::Vector DvlOnlyFactor::evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::imuBias::ConstantBias &vbias_i,
    boost::optional<gtsam::Matrix &> H1,
    boost::optional<gtsam::Matrix &> H2,
    boost::optional<gtsam::Matrix &> H3
) const {
    gtsam::Rot3 R_i = pose_i.rotation();
    gtsam::Point3 p_i = pose_i.translation();
    
    gtsam::Rot3 R_j = pose_j.rotation();
    gtsam::Point3 p_j = pose_j.translation();

    gtsam::Matrix3 H_vbias;
    gtsam::Point3 vbias = vbias_i.accelerometer();
    gtsam::Point3 accumulated_translation = _pvm.predict(vbias, H_vbias);


    // residual = R_i^T * (delta_p_world) - integrated_dvl_body
    gtsam::Matrix3 RiT = R_i.transpose().matrix();
    gtsam::Vector3 residual = RiT * (p_j - p_i) - accumulated_translation;

    if (H1) {
        H1->resize(3, 6);
        H1->block<3, 3>(0, 0) = gtsam::Matrix3::Zero();
        H1->block<3, 3>(0, 3) = -RiT; // deriv w.r.t. p_i
    }
    if (H2) {
        H2->resize(3, 6);
        H2->block<3, 3>(0, 0) = gtsam::Matrix3::Zero();
        H2->block<3, 3>(0, 3) = RiT; // deriv w.r.t. p_j
    }
    if (H3) {
        H3->resize(3, 6);
        H3->block<3, 3>(0, 0) = -H_vbias; // acceleration bias affects prediction negatively
        H3->block<3, 3>(0, 3) = gtsam::Matrix3::Zero();
    }
    return residual;
}


} // namespace localization