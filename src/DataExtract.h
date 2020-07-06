//
// Created by sensetime on 20-4-27.
//

#ifndef SENSEDATAEXTRACT_DATAEXTRACT_H
#define SENSEDATAEXTRACT_DATAEXTRACT_H

#include <iostream>
#include <vector>
#include <fstream>
#include <eigen3/Eigen/Eigen>
#include "SensetimeRGBDReconCore.h"

#define DEAULT_EXPOSURE_TIME 0.02f

struct old_IMUData {
    Eigen::Vector3d acc, gyr;
    double t;
};

struct IMUData {
    Eigen::Vector3d acc, gyr;
    Eigen::Vector3d acc_bias = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d gyr_bias = Eigen::Vector3d(0,0,0);
    double t;
};

struct Attitude {
    Eigen::Vector3d g;  ///< gravity
    Eigen::Quaterniond q;  ///< orientation
    double t = -1.0;
};


bool ExtractInputDataFromTxt(std::string fileName, STColorImage &rgb, STDepthImage &depth, STAttitude &att);

bool ExtractIMUData(std::string fileName, std::vector<STAcceleration> &accelerations,
                    std::vector<STGyroscope> &gyros,
                    STGravity &g, STRotationVector &r);


bool ExtractInputDataFromBin(
        std::string fileName,
        STColorImage &rgb, STDepthImage &depth, float *matrix,
        std::vector<STAcceleration> &accelerations,
        std::vector<STGyroscope> &gyros,
        STGravity &g, STRotationVector &r);

#endif //SENSEDATAEXTRACT_DATAEXTRACT_H
