# Group 10 Final Project - NAVARCH 568 W19
###### Xiaotong Chen, Deyang Dai, Yizhou Lu, and Joseph Yates
This project is created by Team 10, as its the final project for University of Michigan's NAVARCH 568, Winter 2019. The aim of this work is to replicate the IMU preintegration factor work completed in [Forster, Carlone, Dellaert, and Scaramuzza (2016)](https://ieeexplore.ieee.org/document/7557075). We then seek to validate this against the paper's implementation in the [Georgia Tech Smoothing and Mapping (GTSAM) library](https://bitbucket.org/gtborg/gtsam/overview). Additionally, we seek to test the IMU preintegration factor methodology on two relatively new datasets: IMU data from the UMich Biped Robotics Lab's [Cassie Blue](https://www.biped.solutions/research) robot, and the [KAIST Urban Dataset](http://irap.kaist.ac.kr/dataset/). With the KAIST dataset implementation, we also demonstrate the improvement that IMU preintegration offers over a baseline [ORB-SLAM2](https://arxiv.org/abs/1610.06475) implementation.

We implement a [stereo version of ORB-SLAM2 with IMU Preintegration](https://github.com/cxt98/NA568-final-team10/tree/master/ORB_SLAM2). The IMU preintegration module refers to [LearnVIORB](https://github.com/jingpang/LearnVIORB).

## Dependencies
This repo relies on two MATLAB toolboxes: MATLAB's own [Robotics System Toolbox](https://www.mathworks.com/products/robotics.html) and the [GTSAM Toolbox 3.2.0](https://borg.cc.gatech.edu/download.html#download). The latter is the MATLAB wrapper form of the [GTSAM library](https://bitbucket.org/gtborg/gtsam/overview). We include the GTSAM github repo as a submodule in our repo here; it may be compiled into the toolbox using [CMake](https://cmake.org/) (with [Boost](https://www.boost.org/) as a dependency to that compilation).

## Directories
1. `imu_preint_matlab` - initial pass at MATLAB implementation of IMU factors, and a GTSAM solution for an IMU-only factor graph over the Cassie Blue data.
2. `IMU_Preintegration` - second pass at MATLAB implementation of IMU factors and solvers. Has a partially-working implementation that we developed, as well as a GTSAM reference. Uses Cassie and KAIST Urban data.
3. `ORB-SLAM2` - Modified ORB-SLAM2 implementation with IMU factors demonstrating successful improvement of ORB-SLAM2 due to those IMU factors and Stereo vision using the EuRoC data.
4. `gtsam` - GTSAM submodule used for compiling the MATLAB wrapper.
5. `KAIST_urban29_data` - KAIST Urban dataset (#29).
6. `KITTI_00_data` - KITTI dataset (#00).

## Executables and Scripts
1. `imu_preint_matlab/IMU_PreIntegration.m` runs a simple forward integration of the Cassie Blue dataset, and compares it with ground truth.
2. `imu_preint_matlab/imu_preint_test.m` demonstrates the working IMU factor calculations implemented in `imu_preint_matlab/lib`. However, because it is comparing against truth data, it is not calculating a trajectory.
3. `imu_preint_matlab/imu_cassie_compare.m` runs a GTSAM IMU factor graph solution over the Cassie Blue data, and compares it with ground truth. The original intent of this file was to also compare this with our implementation of the same factor graph; however, because the Gauss-Newton solver does not yet properly function, this is not yet implemented.
4. `IMU_Preintegration/IMU_Preintegration_cassie.m` runs our solver using IMU factors over the Cassie Blue data, and compares it with ground truth. However, it only works successfully over the state rotation matrices. It is implemented in a separate library due to significant code rewrites during debugging of the Gauss-Newton solver.
