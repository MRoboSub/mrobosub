#ifndef  __PREINTEGRATED_VELOCITY_HELPERS_H__
#define  __PREINTEGRATED_VELOCITY_HELPERS_H__

#include <string>
#include <vector>

#include <boost/serialization/access.hpp>
#include <gtsam/base/types.h>

namespace localization {
class PreintegratedVelocityParameters {
public: // Members
    int /* gtsam::Matrix3 */ _bias_velocity_covariance; // Continuous time "covariance" describing velocity measurement bias for random walk
    int /* gtsam::Matrix3 */ _bias_initial;             // Covariance of baias used as an initial estimate

public: //  Methods
    PreintegratedVelocityParameters();
    PreintegratedVelocityParameters(
        const int /* gtsam::Matrix3 */ &bias_velocity_covariance,
        const int /* gtsam::Matrix3 */ &bias_initial
    );
    PreintegratedVelocityParameters(
        const int /*boost::shared_ptr<PreintegratedVelocityParams> &p*/
    );

    // TODO: Make as overloaded= and overloaded<<
    void print(const std::string &s = "") const;
    bool equals(const PreintegratedVelocityParameters &expected, double tolerance = 1e-9) const;
    
    void set_bias_velocity_covariance();
    void set_bias_initial();            

private: // Methods
    // TODO: What is this?
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
    int /*Eigen::Matrix<double, 3, 3> */ _preintegrated_measured_covariance;
    std::vector<int /*gtsam::Vector3*/> _velocity_list;
    std::vector<int /*gtsam::Rot3*/> _rotations_list;
    std::vector<double> _dt_list;

protected: // Members
   friend class DvlOnlyFactor; 

private: // Members
    double delta_T_ij;
    int /*gtsam::Pose3*/ _accumulated_poses;
    int /*gtsam::Point3*/ _accumulated_positions;
    int /*gtsam::imuBais::ConstantBias*/ _dvl_bias_for_imu; // Used for the dvl bias
    int /*gtsam::Matrix3*/ _delp_delbias_omega;
    int /*gtsam::Matrix3*/ _delp_delbias_dvl; // Default initialize as gtsam::Matrix3::Zero();

public: // Methods
    PreintegratedVelocityMeasurementsDvlOnly();
    ~PreintegratedVelocityMeasurementsDvlOnly();

    void reset_integration();
    void reset_integration_and_bias(const int /*gtsam::imuBias::ConstantBias*/ &bias);
    void integrateMeasurements(
        const int /*gtsam::Vector3*/ &linear_velocity,
        const int /*gtsam::Rot3*/ &interpolated_rotation,
        double &dt // TODO: Is this an out param? Why is this passed by ref?
    );
    
    void integrateMeasurementsNoise(
        const int /*gtsam::Vector3*/ &linear_velocity,
        const int /*gtsam::Rot3*/ &interpolated_rotation,
        double &dt, // TODO: Is this an out param? Why is this passed by ref?
        const double &fom
    );

    int /*gtsam::Pose3*/  get_accumulated_poses() const;
    int /*gtsam::Point3*/ get_accumulated_positions() const;
    int /*gtsam::Matrix*/ get_delpij_delbias_omega() const;
    int /*gtsam::Matrix*/ get_delpij_delbias_dvl() const;
    int /*gtsam::Point3*/ predict(const int /*gtsam::Point3*/ &bias, int /*gtsam::Matrix3*/ &H_bias) const;
};

} // namespace localization

#endif //__PREINTEGRATED_VELOCITY_PARAMETERS_H__