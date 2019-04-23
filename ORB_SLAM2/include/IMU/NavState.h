#ifndef NAVSTATE_H
#define NAVSTATE_H

#include "IMU/so3.h"

#include "Eigen/Geometry"

namespace ORB_SLAM2
{

using namespace Eigen;

typedef Matrix<double, 15, 1> Vector15d;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 6, 1> Vector6d;

class NavState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    NavState();
    NavState(const NavState& _ns);

    Sophus::SO3 Get_R() const{return _R;}
    Matrix3d Get_RotMatrix() const{return _R.matrix();}
    Vector3d Get_P() const{return _P;}         // position
    Vector3d Get_V() const{return _V;}         // velocity
    void Set_Pos(const Vector3d &pos){_P = pos;}
    void Set_Vel(const Vector3d &vel){_V = vel;}
    void Set_Rot(const Matrix3d &rot){_R = Sophus::SO3(rot);}
    void Set_Rot(const Sophus::SO3 &rot){_R = rot;}

    Vector3d Get_BiasGyr() const{return _BiasGyr;}   // bias of gyroscope, keep unchanged after init and during optimization
    Vector3d Get_BiasAcc() const{return _BiasAcc;}   // bias of accelerometer
    void Set_BiasGyr(const Vector3d &bg){_BiasGyr = bg;}
    void Set_BiasAcc(const Vector3d &ba){_BiasAcc = ba;}

    Vector3d Get_dBias_Gyr() const{return _dBias_g;}  // delta bias of gyroscope, init as 0, change during optimization
    Vector3d Get_dBias_Acc() const{return _dBias_a;}  // delta bias of accelerometer
    void Set_DeltaBiasGyr(const Vector3d &dbg){_dBias_g = dbg;}
    void Set_DeltaBiasAcc(const Vector3d &dba){_dBias_a = dba;}

    // incremental addition, delta = [dP, dV, dPhi, dBa, dBg]
    void IncSmall(Vector15d delta);
    void IncSmallPVR(Vector9d dPVR);
    void IncSmallBias(Vector6d dBias);

private:

    Vector3d _P;  // position
    Vector3d _V;  // velocity
    Sophus::SO3 _R;

    // keep unchanged during optimization
    Vector3d _BiasGyr;   // bias of gyroscope
    Vector3d _BiasAcc;   // bias of accelerometer

    // update below term during optimization
    Vector3d _dBias_g;  // delta bias of gyroscope, correction term computed in optimization
    Vector3d _dBias_a;  // delta bias of accelerometer
};

} // namespave ORB_SLAM2

#endif // NAVSTATE_H
