#ifndef  __DVL_ONLY_FACTOR_H__
#define  __DVL_ONLY_FACTOR_H__

#include "PreintegratedVelocityHelpers.h"

#include <gtsam/base/Vector.h>               // Vector
#include <gtsam/base/Matrix.h>               // Matrix
#include <gtsam/geometry/Pose3.h>            // Pose3
#include <gtsam/inference/Key.h>             // Key
#include <gtsam/navigation/ImuBias.h>        // imuBias::ConstantBias
#include <gtsam/nonlinear/NonlinearFactor.h> // NoiseModelFactorN

#include <boost/optional.hpp>

namespace localization {
class DvlOnlyFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::imuBias::ConstantBias> {
public: // Methods
    DvlOnlyFactor();
    DvlOnlyFactor(gtsam::Key pose_i, gtsam::Key pose_j, gtsam::Key vbias_i,
                  const PreintegratedVelocityMeasurementsDvlOnly &pvm);
    virtual ~DvlOnlyFactor();

    gtsam::Vector evaluateError(
        const gtsam::Pose3 &pose_i,
        const gtsam::Pose3 &pose_j,
        const gtsam::imuBias::ConstantBias &vbias_i,
        boost::optional<gtsam::Matrix &> H1,
        boost::optional<gtsam::Matrix &> H2,
        boost::optional<gtsam::Matrix &> H3
    ) const;

private: // Members
    PreintegratedVelocityMeasurementsDvlOnly _pvm;
};
} // namespace localization

#endif //__DVL_ONLY_FACTOR_H__