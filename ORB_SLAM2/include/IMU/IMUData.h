#ifndef IMUDATA_H
#define IMUDATA_H

#include <Eigen/Dense>

namespace ORB_SLAM2
{

using namespace Eigen;

class IMUData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // covariance of measurement
    static Matrix3d _gyrMeasCov;
    static Matrix3d _accMeasCov;
    static Matrix3d getGyrMeasCov() {return _gyrMeasCov;}
    static Matrix3d getAccMeasCov() {return _accMeasCov;}

    // covariance of bias random walk
    static Matrix3d _gyrBiasRWCov;
    static Matrix3d _accBiasRWCov;
    static Matrix3d getGyrBiasRWCov() {return _gyrBiasRWCov;}
    static Matrix3d getAccBiasRWCov() {return _accBiasRWCov;}

    static double _gyrBiasRw2;
    static double _accBiasRw2;
    static double getGyrBiasRW2() {return _gyrBiasRw2;}
    static double getAccBiasRW2() {return _accBiasRw2;}


    IMUData(const double& gx, const double& gy, const double& gz,
            const double& ax, const double& ay, const double& az,
            const double& t);

    // Raw data of imu's
    Vector3d _gyr; //gyr data
    Vector3d _acc; //acc data
    double _time; //timestamp
};

} // namespace ORB_SLAM2

#endif // IMUDATA_H
