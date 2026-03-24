#ifndef  __PREINTEGRATED_VELOCITY_HELPERS_H__
#define  __PREINTEGRATED_VELOCITY_HELPERS_H__

#include <gtsam/base/types.h>             // GTSAM_MAKE_ALIGNED_OPERATOR_NEW
#include <gtsam/base/Matrix.h>            // gtsam::Matrix3
#include <gtsam/base/Vector.h>            // gtsam::Vector
#include <gtsam/geometry/Rot3.h>          // gtsam::Rot3
#include <gtsam/geometry/Pose3.h>         // gtsam::Pose3
#include <gtsam/geometry/Point3.h>        // gtsam::Point3
#include <gtsam/navigation/ImuBias.h>     // gtsam::imuBias::ConstantBias

#include <Eigen/Core>                     // Eigen::Matrix

#include <boost/serialization/access.hpp> // boost::serialize::access, serialize
#include <boost/shared_ptr.hpp>           // boost::shared_ptr

#include <string>
#include <vector>

namespace localization {
class PreintegratedVelocityParameters {
public: // Members
    gtsam::Matrix3 _bias_velocity_covariance; // Continuous time "covariance" describing velocity measurement bias for random walk
    gtsam::Matrix3 _bias_initial;             // Covariance of bias used as an initial estimate

public: //  Methods
    PreintegratedVelocityParameters();
    PreintegratedVelocityParameters(
        const gtsam::Matrix3 &bias_velocity_covariance,
        const gtsam::Matrix3 &bias_initial
    );
    PreintegratedVelocityParameters(
        const boost::shared_ptr<PreintegratedVelocityParameters> &p
    );

    // TODO: Make as overloaded= and overloaded<<
    void print(const std::string &s = "") const;
    bool equals(const PreintegratedVelocityParameters &expected, double tolerance = 1e-9) const;
    
    void set_bias_velocity_covariance(const gtsam::Matrix3 &covariance);
    void set_bias_initial(const gtsam::Matrix3 &covariance);            

private: // Methods
    // Allows boost serialization to access private members.
    friend class boost::serialization::access;

    // Needed to save data and load data
    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version);

public:
    // GTSAM uses advanced math with Eigen
    // This is to ensure that the Matrices and other stuff
    // is aligned how GTSAM wants it to be.
    GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

class PreintegratedVelocityMeasurementsDvlOnly {
public: // Members
    Eigen::Matrix<double, 3, 3> _preintegrated_measured_covariance;
    std::vector<gtsam::Vector3> _velocity_list;
    std::vector<gtsam::Rot3> _rotations_list;
    std::vector<double> _dt_list;

protected: // Members
   friend class DvlOnlyFactor; 

private: // Members
    double delta_T_ij;
    gtsam::Pose3 _accumulated_poses;
    gtsam::Point3 _accumulated_positions;
    gtsam::imuBias::ConstantBias _dvl_bias_for_imu; // Used for the dvl bias
    gtsam::Matrix3 _delp_delbias_omega;
    gtsam::Matrix3 _delp_delbias_dvl; // Default initialize as gtsam::Matrix3::Zero();

public: // Methods
    PreintegratedVelocityMeasurementsDvlOnly();
    ~PreintegratedVelocityMeasurementsDvlOnly();

    void reset_integration();
    void reset_integration_and_bias(const gtsam::imuBias::ConstantBias &bias);
    void integrate_measurements(
        const gtsam::Vector3 &linear_velocity,
        const gtsam::Rot3 &interpolated_rotation,
        double &dt // TODO: Is this an out param? Why is this passed by ref?
    );
    
    void integrate_measurements_noise(
        const gtsam::Vector3 &linear_velocity,
        const gtsam::Rot3 &interpolated_rotation,
        double &dt, // TODO: Is this an out param? Why is this passed by ref?
        const double &fom
    );

    gtsam::Pose3  get_accumulated_poses() const;
    gtsam::Point3 get_accumulated_positions() const;
    gtsam::Matrix get_delpij_delbias_omega() const;
    gtsam::Matrix get_delpij_delbias_dvl() const;
    gtsam::Matrix get_preintegrated_measured_covariance() const;

    gtsam::Point3 predict(const gtsam::Point3 &bias, gtsam::Matrix3 &H_bias) const;
};

} // namespace localization

#endif //__PREINTEGRATED_VELOCITY_PARAMETERS_H__