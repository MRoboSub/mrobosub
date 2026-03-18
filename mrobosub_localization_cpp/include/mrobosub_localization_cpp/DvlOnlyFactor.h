#ifndef  __DVL_ONLY_FACTOR_H__
#define  __DVL_ONLY_FACTOR_H__

#include "PreintegratedVelocityHelpers.h"

class DvlOnlyFactor /* : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::imuBias::ConstantBias>*/ {
public: // Methods
    DvlOnlyFactor();
    DVLOnlyFactor(int /*gtsam::Key*/ pose_i, int /*gtsam::Key*/ pose_j, int /*gtsam::Key*/ vbias_i,
                  const PreintegratedVelocityMeasurementsDvlOnly &pvm);
    virtual ~DvlOnlyFactor();

    int /*gtsam::Vector*/ evaluate_error(
        const int /*gtsam::Pose3*/ &pose_i,
        const int /*gtsam::Pose3*/ &pose_j,
        const int /*gtsam::imuBias::ConstantBias*/ &vbias_i,
        int /*boost::optional<gtsam::Matrix &>*/ H1,
        int /*boost::optional<gtsam::Matrix &>*/ H2,
        int /*boost::optional<gtsam::Matrix &>*/ H3
    ) const;

private: // Members
    PreintegratedVelocityMeasurementsDvlOnly _pvm;
};

#endif //__DVL_ONLY_FACTOR_H__