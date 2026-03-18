#ifndef  __BLUEROV_BAROMETER_FACTOR_H__
#define  __BLUEROV_BAROMETER_FACTOR_H__

namespace localization {
class BluerovBarometerFactor /*: public NoiseModelFactor1<Pose3> */ {
private: 
    double _measured;

public:
    BluerovBarometerFactor();
    virtual ~BluerovBarometerFactor();
    BluerovBarometerFactor(int /*gtsam::Key*/ key, double measured, const int /*gtsam::SharedNoiseModle*/ &model);
    int /*gtsam::Vector*/ evaluate_error(const int /*gtsam::Pose3*/ &pose, int /*boost::optional<gtsam::Matrix &>*/ H) const;
};
} // namespace localization


#endif //__BLUEROV_BAROMETER_FACTOR_H__