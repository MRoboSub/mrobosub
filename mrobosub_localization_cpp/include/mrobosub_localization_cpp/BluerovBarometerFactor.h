#ifndef  __BLUEROV_BAROMETER_FACTOR_H__
#define  __BLUEROV_BAROMETER_FACTOR_H__

#include <gtsam/base/Vector.h>     // gtsam::Vector
#include <gtsam/base/Matrix.h>     // gtsam::Matrix
#include <gtsam/geometry/Pose3.h>  // gtsam::Pose3
#include <gtsam/inference/Key.h>   // gtsam::Key
#include <gtsam/nonlinear/NonlinearFactor.h> // gtsam::NoiseModelFactor1
#include <gtsam/linear/NoiseModel.h> // gtsam::SharedNoiseModel

namespace localization {
class BluerovBarometerFactor : public NoiseModelFactor1<Pose3> {
private: 
    double _measured;

public:
    BluerovBarometerFactor();
    virtual ~BluerovBarometerFactor();
    BluerovBarometerFactor(gtsam::Key key, double measured, const gtsam::SharedNoiseModel &model);
  
    int gtsam::Vector evaluate_error(
        const gtsam::Pose3 &pose, 
        boost::optional<gtsam::Matrix &> H
    ) const;
};
} // namespace localization


#endif //__BLUEROV_BAROMETER_FACTOR_H__