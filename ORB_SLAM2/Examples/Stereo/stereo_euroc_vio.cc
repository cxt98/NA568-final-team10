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

void loadImage(char *imageFolderPath, std::vector<string> &vImageFilePath, std::vector<double> &vImageTime);

void loadIMUData(char *imuFolderPath, std::vector<ORB_SLAM2::IMUData> &vIMUData);

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
                "/home/deyangd/Downloads/EuRoC/V1_02/mav0/cam1/data EuRoC_V1_02" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);

    ORB_SLAM2::ConfigParam config(argv[2]);

    std::vector<ORB_SLAM2::IMUData> vAllIMUData;
    std::vector<string> vImagePath;
    std::vector<double> vImageTime;

    loadIMUData(argv[3], vAllIMUData);
    loadImage(argv[4], vImagePath, vImageTime);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    int nImages = vImagePath.size();
    vTimesTrack.resize(nImages);

    if (nImages < 1)
    {
        cerr << endl << "There is no enough images." << endl;
    }

    cv::Mat M1l, M2l, M1r, M2r;
    cv::initUndistortRectifyMap(config._K_l, config._D_l, config._R_l, config._P_l.rowRange(0, 3).colRange(0, 3), cv::Size(config._cols_l, config._rows_l), CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(config._K_r, config._D_r, config._R_r, config._P_r.rowRange(0, 3).colRange(0, 3), cv::Size(config._cols_r, config._rows_r), CV_32F, M1r, M2r);

    cv::Mat imLeft, imRight, imLeftRect, imRightRect;

    for (int j = 0; j < nImages - 1; j++)
    {
        std::vector<ORB_SLAM2::IMUData> vimuData;
        for (unsigned int i = 0; i < 10; i++)
        {
            int index = 10 * j + i;
            vimuData.push_back(vAllIMUData[index]);
        }

        string leftImgPath = argv[5] + vImagePath[j+1];
        string rightImgPath = argv[6] + vImagePath[j+1];

        imLeft = cv::imread(leftImgPath, 0);
        imRight = cv::imread(rightImgPath, 0);

        if (imLeft.empty())
        {
            cerr << endl << "Failed to load image at: " << leftImgPath << endl;
            return 1;
        }

        if (imRight.empty())
        {
            cerr << endl << "Failed to load image at: " << rightImgPath << endl;
            return 1;
        }

        // Rectification
        cv::remap(imLeft, imLeftRect, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightRect, M1r, M2r, cv::INTER_LINEAR);

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        SLAM.TrackStereoVIO(imLeftRect, imRightRect, vimuData, vImageTime[j + 1]);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[j] = ttrack;

        while (!SLAM.bLocalMapAcceptKF())
        {
            usleep(1);
        }
    }

    // Twb
    SLAM.SaveKeyFrameTrajectoryNavState(ORB_SLAM2::ConfigParam::_tmpFilePath +argv[7]+ "_StereoVioKeyframe.txt");

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM(ORB_SLAM2::ConfigParam::_tmpFilePath + argv[7]+"_StereoVio.txt"); // Twc

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

    cout << "Press any key to shutdown" << endl;
    cin.get();

    // Stop all threads
    SLAM.Shutdown();
    return 0;
}

void loadImage(char *imageFolderPath, std::vector<string> &vImageFilePath, std::vector<double> &vImageTime)
{
    ifstream in_file;
    in_file.open(imageFolderPath, ifstream::in);

    string str;
    getline(in_file, str);
    while (!in_file.eof())
    {
        getline(in_file, str);
        stringstream ss(str);

        getline(ss, str, ',');
        vImageTime.push_back(atof(str.c_str()) / 1e9);
        ss >> str;
        vImageFilePath.push_back('/' + str);
    }

    vImageTime.pop_back();
    vImageFilePath.pop_back();
    in_file.close();
}

void loadIMUData(char *imuFolderPath, std::vector<ORB_SLAM2::IMUData> &vIMUData)
{
    ifstream in_file;
    in_file.open(imuFolderPath, ifstream::in);

    string str;
    getline(in_file, str);
    while (!in_file.eof())
    {
        double data[7] = {0.0};

        getline(in_file, str);
        stringstream ss(str);

        for (int i = 0; getline(ss, str, ','); i++)
        {
            data[i] = atof(str.c_str());
        }

        ORB_SLAM2::IMUData imuData(data[1], data[2], data[3], data[4], data[5], data[6], data[0] / 1e9);
        vIMUData.push_back(imuData);
    }

    vIMUData.pop_back();
    in_file.close();
}