//
// Created by sensetime on 20-4-27.
//

#include "CalibReader.h"

bool CalibBinReader::ReadCalib(std::string fileName){
    fp = fopen(fileName.c_str(), "rb");
    if (fp == NULL) {
        fprintf(stderr, "%s fopen error!\n", fileName.c_str());
        return false;
    } else {
        printf("open %s OK\n", fileName.c_str());
    }
    return true;
}

void CalibBinReader::CloseCalib(){
    if (fp) {
        fclose(fp);
        fp = NULL;
    }
}

bool CalibSamsungBinReader::GetRT(Eigen::Matrix4f &RT){
    RT.setIdentity();
    fseek(fp, 4, SEEK_SET);
    Eigen::Vector3d eular;
    Read<double>(eular(0));
    Read<double>(eular(1));
    Read<double>(eular(2));

    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    Read<double>(R(0, 0));
    Read<double>(R(0, 1));
    Read<double>(R(0, 2));
    Read<double>(R(1, 0));
    Read<double>(R(1, 1));
    Read<double>(R(1, 2));
    Read<double>(R(2, 0));
    Read<double>(R(2, 1));
    Read<double>(R(2, 2));

    // R = Eigen::AngleAxisd(eular(0), Eigen::Vector3d::UnitX())
    //     * Eigen::AngleAxisd(eular(1), Eigen::Vector3d::UnitY())
    //     * Eigen::AngleAxisd(eular(2), Eigen::Vector3d::UnitZ());
    R.transposeInPlace();

    fseek(fp, 100, SEEK_SET);
    Read<double>(t(0));
    Read<double>(t(1));
    Read<double>(t(2));
    t = -R * t;

    RT.block<3,3>(0,0) = R.cast<float>();
    RT.block<3,1>(0,3) = t.cast<float>();
    return true;
}

bool CalibSamsungBinReader::GetRGB_K(Eigen::Vector4f& K, int& rgb_w, int& rgb_h){
    fseek(fp, 564, SEEK_SET);
    Read<int>(rgb_w);
    Read<int>(rgb_h);
    K.setIdentity();

    fseek(fp, 124, SEEK_SET);
    double fx, fy, cx, cy;
    double value;

    Read<double>(fx);
    Read<double>(value);
    Read<double>(cx);
    Read<double>(value);

    Read<double>(fy);
    Read<double>(cy);

    K(0) = fx;
    K(1) = fy;
    K(2) = cx;
    K(3) = cy;
    return true;
}

bool CalibSamsungBinReader::GetRGB_K(Eigen::Matrix3f& K, int& rgb_w, int& rgb_h){
    Eigen::Vector4f K_vec;
    GetRGB_K(K_vec, rgb_w, rgb_h);
    K.setIdentity();
    K(0,0) = K_vec(0);
    K(1,1) = K_vec(1);
    K(0,2) = K_vec(2);
    K(1,2) = K_vec(3);

    return true;
}

bool CalibSamsungBinReader::GetToF_K(Eigen::Vector4f& K, int& tof_w, int& tof_h){
    fseek(fp, 572, SEEK_SET);
    Read<int>(tof_w);
    Read<int>(tof_h);
    K.setIdentity();

    fseek(fp, 196, SEEK_SET);
    double fx, fy, cx, cy;
    double value;

    Read<double>(fx);
    Read<double>(value);
    Read<double>(cx);
    Read<double>(value);

    Read<double>(fy);
    Read<double>(cy);

    K(0) = fx;
    K(1) = fy;
    K(2) = cx;
    K(3) = cy;
    return true;
}

bool CalibSamsungBinReader::GetToF_K(Eigen::Matrix3f& K, int& tof_w, int& tof_h){
    Eigen::Vector4f K_vec;
    GetToF_K(K_vec, tof_w, tof_h);
    K.setIdentity();
    K(0,0) = K_vec(0);
    K(1,1) = K_vec(1);
    K(0,2) = K_vec(2);
    K(1,2) = K_vec(3);
    return true;
}

bool CalibOPPOBinReader::GetRT(Eigen::Matrix4f &RT){
    RT.setIdentity();
    fseek(fp, RT_START, SEEK_SET);
    Eigen::Vector3f eular;
    Read<float>(eular(0));
    Read<float>(eular(1));
    Read<float>(eular(2));

    Eigen::Matrix3f R;
    Eigen::Vector3f t;

    R = Eigen::AngleAxisf(eular(0), Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(eular(1), Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(eular(2), Eigen::Vector3f::UnitZ());
    R.transposeInPlace();

    Read<float>(t(0));
    Read<float>(t(1));
    Read<float>(t(2));
    t = -R * t;

    RT.block<3,3>(0,0) = R;
    RT.block<3,1>(0,3) = t;
    return true;
}

bool CalibOPPOBinReader::GetRGB_K(Eigen::Vector4f& K, int& rgb_w, int& rgb_h){
    fseek(fp, RGB1_START, SEEK_SET);
    Read<int>(rgb_h);
    Read<int>(rgb_w);
    K.setIdentity();
    Read<float>(K(0));
    Read<float>(K(1));
    Read<float>(K(2));
    Read<float>(K(3));
    return true;
}

bool CalibOPPOBinReader::GetRGB_K(Eigen::Matrix3f& K, int& rgb_w, int& rgb_h){
    Eigen::Vector4f K_vec;
    GetRGB_K(K_vec, rgb_w, rgb_h);
    K.setIdentity();
    K(0,0) = K_vec(0);
    K(1,1) = K_vec(1);
    K(0,2) = K_vec(2);
    K(1,2) = K_vec(3);

    return true;
}

bool CalibOPPOBinReader::GetToF_K(Eigen::Vector4f& K, int& tof_w, int& tof_h){
    fseek(fp, TOF_START, SEEK_SET);
    Read<int>(tof_h);
    Read<int>(tof_w);
    K.setIdentity();
    Read<float>(K(0));
    Read<float>(K(1));
    Read<float>(K(2));
    Read<float>(K(3));
    return true;
}

bool CalibOPPOBinReader::GetToF_K(Eigen::Matrix3f& K, int& tof_w, int& tof_h){
    Eigen::Vector4f K_vec;
    GetToF_K(K_vec, tof_w, tof_h);
    K.setIdentity();
    K(0,0) = K_vec(0);
    K(1,1) = K_vec(1);
    K(0,2) = K_vec(2);
    K(1,2) = K_vec(3);
    return true;
}


CalibTxtReader::CalibTxtReader(){}
CalibTxtReader::CalibTxtReader(std::string path){
    calib_path = path;
    char e = calib_path[calib_path.size()-1];
//    printf("%c\n", e);
    if(e!='/') calib_path += '/';
}


bool CalibTxtReader::GetRT(Eigen::Matrix4f &RT){
    std::ifstream file;
    file.open(calib_path  + "RT.txt");
    if (!file.is_open()) return false;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            file >> RT(i, j);
    }
    return true;
}
bool CalibTxtReader::GetRGB_K(Eigen::Matrix3f& K, int& rgb_w, int& rgb_h){
    std::ifstream file(calib_path  + "K_rgb.txt");
    if (!file.is_open()) return false;
    file >> rgb_w >> rgb_h;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
            file >> K(i, j);
    }
    file.close();
    return true;
}
bool CalibTxtReader::GetToF_K(Eigen::Matrix3f& K, int& tof_w, int& tof_h){
    std::ifstream file(calib_path  + "K_tof.txt");
    if (!file.is_open()) return false;
    file >> tof_w >> tof_h;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
            file >> K(i, j);
    }
    file.close();
    return true;
}