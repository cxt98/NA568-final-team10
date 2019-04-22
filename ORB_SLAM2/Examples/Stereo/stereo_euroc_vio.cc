/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "System.h"
#include "IMU/IMUData.h"
#include "IMU/ConfigParam.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <boost/foreach.hpp>
#include <fstream>
#include <time.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <atomic>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

typedef struct ImageData
{
    double timeStamp = 0.;
    string imgName;
} Image;

void loadImage(char *imagePath, std::vector<Image> &iListData);

void loadIMUData(char *imuPath, std::vector<ORB_SLAM2::IMUData> &vimuData);

int main(int argc, char **argv)
{
    if (argc != 8)
    {
        cerr << endl << "Usage: ./Examples/Stereo/stereo_euroc_vio path_to_vocabulary "
                        "path_to_euroc.yaml path_to_imu/data.csv path_to_cam0/data.csv "
                        "path_to_cam0/data path_to_cam1/data output_file_name" << endl;
        cerr << "Example:" << endl;
        cerr << "./Examples/Stereo/stereo_euroc_vio Vocabulary/ORBvoc.bin "
                "Examples/Stereo/EuRoC_Stereo.yaml /home/deyangd/Downloads/EuRoC/V1_02/mav0/imu0/data.csv "
                "/home/deyangd/Downloads/EuRoC/V1_02/mav0/cam0/data.csv "
                "/home/deyangd/Downloads/EuRoC/V1_02/mav0/cam0/data "
                "/home/deyangd/Downloads/EuRoC/V1_02/mav0/cam1/data V1_02_medium" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);

    ORB_SLAM2::ConfigParam config(argv[2]);

    char *fullPath = new char[500];
    char *fullPathR = new char[500];
    memset(fullPath, 0, 500);
    memset(fullPath, 0, 500);

    std::vector<ORB_SLAM2::IMUData> vAllIMUData;
    std::vector<Image> iListData;

    loadIMUData(argv[3], vAllIMUData);
    loadImage(argv[4], iListData);

    double ImgFirstTime = iListData[0].timeStamp;
    for (int j = 0; j < int(vAllIMUData.size()) - 1; j++)
    {
        if (ImgFirstTime - vAllIMUData[j]._time < 1 / 1e4)
        {

            vAllIMUData.erase(vAllIMUData.begin(), vAllIMUData.begin() + j);
            break;
        }
    }

    cout << std::setprecision(13) << "first Img time, first Imu timeStamp: " << iListData[0].timeStamp << ",     " << vAllIMUData[0]._time << endl;
    if (iListData[0].timeStamp - vAllIMUData[0]._time > 1 / 1e4)
        cerr << "the timestamp of first Imu is not equal to the first Img!" << endl;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(iListData.size());

    int nImages = iListData.size();
    if (nImages < 1)
    {
        cerr << endl << "There is no enough images." << endl;
    }

    if (config._K_l.empty() || config._K_r.empty() || config._P_l.empty() || config._P_r.empty() || config._R_l.empty() || config._R_r.empty() || config._D_l.empty() || config._D_r.empty() ||
            config._rows_l == 0 || config._rows_r == 0 || config._cols_l == 0 || config._cols_r == 0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l, M2l, M1r, M2r;
    cv::initUndistortRectifyMap(config._K_l, config._D_l, config._R_l, config._P_l.rowRange(0, 3).colRange(0, 3), cv::Size(config._cols_l, config._rows_l), CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(config._K_r, config._D_r, config._R_r, config._P_r.rowRange(0, 3).colRange(0, 3), cv::Size(config._cols_r, config._rows_r), CV_32F, M1r, M2r);

    cv::Mat im, imR, imLeftRect, imRightRect;

    for (int j = 0; j < int(iListData.size()) - 1; j++)
    {
        std::vector<ORB_SLAM2::IMUData> vimuData;
        for (unsigned int i = 0; i < 10; i++)
        {
            int index = 10 * j + i;
            vimuData.push_back(vAllIMUData[index]);
        }

        string temp = iListData[j + 1].imgName.substr(0, iListData[j].imgName.size() - 1);

        sprintf(fullPath, "%s/%s", argv[5], temp.c_str());
        sprintf(fullPathR, "%s/%s", argv[6], temp.c_str());
        im = cv::imread(fullPath, 0);
        imR = cv::imread(fullPathR, 0);

        static double startT = -1;
        if (startT < 0)
            startT = iListData[j + 1].timeStamp;
        if (iListData[j + 1].timeStamp < startT)
        {
            im = cv::Mat::zeros(im.rows, im.cols, im.type());
            imR = cv::Mat::zeros(im.rows, im.cols, im.type());
        }

        if (im.empty())
        {
            cerr << endl << "Failed to load image at: " << fullPath << endl;
            return 1;
        }

        if (imR.empty())
        {
            cerr << endl << "Failed to load image at: " << fullPathR << endl;
            return 1;
        }
        memset(fullPath, 0, 500);
        memset(fullPathR, 0, 500);

        // Rectification
        cv::remap(im, imLeftRect, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imR, imRightRect, M1r, M2r, cv::INTER_LINEAR);

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        SLAM.TrackStereoVIO(imLeftRect, imRightRect, vimuData, iListData[j + 1].timeStamp);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[j] = ttrack;

        while (!SLAM.bLocalMapAcceptKF())
        {
            usleep(1);
        }
    }
    delete [] fullPath;
    delete [] fullPathR;

    // from body(IMU) to world.
    SLAM.SaveKeyFrameTrajectoryNavState(ORB_SLAM2::ConfigParam::_tmpFilePath +argv[7]+ "_StereoVioKeyframe.txt");

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM(ORB_SLAM2::ConfigParam::_tmpFilePath + argv[7]+"_StereoVio.txt"); //from cam to world.

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    cout << endl << endl << "press any key to shutdown" << endl;
    cin.get();

    // Stop all threads
    SLAM.Shutdown();
    return 0;
}

void loadImage(char * imagePath, std::vector<Image> &iListData)
{
    ifstream inf;
    inf.open(imagePath, ifstream::in);
    const int cnt = 2;

    string line;
    int j = 0;
    size_t comma = 0;
    size_t comma2 = 0;
    Image temp;
    getline(inf, line);
    while (!inf.eof())
    {
        getline(inf, line);

        comma = line.find(',', 0);
        stringstream ss;
        ss << line.substr(0, comma).c_str();
        ss >> temp.timeStamp ;
        temp.timeStamp = temp.timeStamp / 1e9;

        while (comma < line.size() && j != cnt - 1)
        {

            comma2 = line.find(',', comma + 1);
            temp.imgName = line.substr(comma + 1, comma2 - comma - 1);
            ++j;
            comma = comma2;
        }
        iListData.push_back(temp);
        j = 0;
    }

    iListData.pop_back();
    inf.close();
}

void loadIMUData(char *imuPath, std::vector<ORB_SLAM2::IMUData> &vimuData)
{
    ifstream inf;
    inf.open(imuPath, ifstream::in);
    const int cnt = 7;

    string line;
    int j = 0;
    size_t comma = 0;
    size_t comma2 = 0;

    double acc[3] = {0.0};
    double grad[3] = {0.0};
    double imuTimeStamp = 0;

    getline(inf, line);
    while (!inf.eof())
    {
        getline(inf, line);

        comma = line.find(',', 0);
        stringstream ss;
        ss << line.substr(0, comma).c_str();
        ss >> imuTimeStamp;
        while (comma < line.size() && j != cnt - 1)
        {

            comma2 = line.find(',', comma + 1);
            switch (j)
            {
                case 0:
                    grad[0] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
                case 1:
                    grad[1] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
                case 2:
                    grad[2] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
                case 3:
                    acc[0] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
                case 4:
                    acc[1] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
                case 5:
                    acc[2] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
                default:
                    break;
            }
            ++j;
            comma = comma2;
        }
        ORB_SLAM2::IMUData tempImu(grad[0], grad[1], grad[2], acc[0], acc[1], acc[2], imuTimeStamp / 1e9);
        vimuData.push_back(tempImu);
        j = 0;
    }

    vimuData.pop_back();

    inf.close();
}