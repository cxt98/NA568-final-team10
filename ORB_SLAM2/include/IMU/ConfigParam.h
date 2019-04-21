#ifndef CONFIGPARAM_H
#define CONFIGPARAM_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{

class ConfigParam
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConfigParam(std::string configfile);

    static Eigen::Matrix4d GetEigTbc();
    static cv::Mat GetMatTbc();
    static Eigen::Matrix4d GetEigT_cb();
    static cv::Mat GetMatT_cb();
    static int GetLocalWindowSize();
    static double GetG(){return _g;}
    static std::string getTmpFilePath();

    static std::string _tmpFilePath;

    cv::Mat  _K_l, _K_r, _P_l, _P_r, _R_l, _R_r, _D_l, _D_r;
    int  _rows_l, _cols_l, _rows_r, _cols_r;

private:
    static Eigen::Matrix4d _EigTbc;
    static cv::Mat _MatTbc;
    static Eigen::Matrix4d _EigTcb;
    static cv::Mat _MatTcb;
    static int _LocalWindowSize;

    static double _g;
};

} // namespace ORB_SLAM2

#endif // CONFIGPARAM_H
