#include "IMU/ConfigParam.h"

namespace ORB_SLAM2
{
double ConfigParam::_g = 9.810;

Eigen::Matrix4d ConfigParam::_EigTbc = Eigen::Matrix4d::Identity();
cv::Mat ConfigParam::_MatTbc = cv::Mat::eye(4,4,CV_32F);
Eigen::Matrix4d ConfigParam::_EigTcb = Eigen::Matrix4d::Identity();
cv::Mat ConfigParam::_MatTcb = cv::Mat::eye(4,4,CV_32F);
int ConfigParam::_LocalWindowSize = 10;
std::string ConfigParam::_tmpFilePath = "";

ConfigParam::ConfigParam(const std::string configfile)
{
    cv::FileStorage fSettings(configfile, cv::FileStorage::READ);

    fSettings["test.InitVIOTmpPath"] >> _tmpFilePath;

    _LocalWindowSize = fSettings["LocalMapping.LocalWindowSize"];

    {
        cv::FileNode Tbc_ = fSettings["Camera.Tbc"];
        Eigen::Matrix<double,3,3> R;
        R << Tbc_[0], Tbc_[1], Tbc_[2],
                Tbc_[4], Tbc_[5], Tbc_[6],
                Tbc_[8], Tbc_[9], Tbc_[10];
        Eigen::Quaterniond qr(R);
        R = qr.normalized().toRotationMatrix();
        Eigen::Matrix<double,3,1> t( Tbc_[3], Tbc_[7], Tbc_[11]);
        _EigTbc = Eigen::Matrix4d::Identity();
        _EigTbc.block<3,3>(0,0) = R;
        _EigTbc.block<3,1>(0,3) = t;
        _MatTbc = cv::Mat::eye(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                _MatTbc.at<float>(i,j) = _EigTbc(i,j);

        _EigTcb = Eigen::Matrix4d::Identity();
        _EigTcb.block<3,3>(0,0) = R.transpose();
        _EigTcb.block<3,1>(0,3) = -R.transpose()*t;
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                _MatTcb.at<float>(i,j) = _EigTcb(i,j);

    }

    fSettings["LEFT.K"] >> _K_l;
    fSettings["RIGHT.K"] >> _K_r;

    fSettings["LEFT.P"] >> _P_l;
    fSettings["RIGHT.P"] >> _P_r;

    fSettings["LEFT.R"] >> _R_l;
    fSettings["RIGHT.R"] >> _R_r;

    fSettings["LEFT.D"] >> _D_l;
    fSettings["RIGHT.D"] >> _D_r;

    _rows_l = fSettings["LEFT.height"];
    _cols_l = fSettings["LEFT.width"];
    _rows_r = fSettings["RIGHT.height"];
    _cols_r = fSettings["RIGHT.width"];
}

std::string ConfigParam::getTmpFilePath()
{
    return _tmpFilePath;
}

Eigen::Matrix4d ConfigParam::GetEigTbc()
{
    return _EigTbc;
}

cv::Mat ConfigParam::GetMatTbc()
{
    return _MatTbc.clone();
}

Eigen::Matrix4d ConfigParam::GetEigT_cb()
{
    return _EigTcb;
}

cv::Mat ConfigParam::GetMatT_cb()
{
    return _MatTcb.clone();
}

int ConfigParam::GetLocalWindowSize()
{
    return _LocalWindowSize;
}


} // namespace ORB_SLAM2
