//
// Created by sensetime on 20-4-27.
//

#ifndef SENSEDATAEXTRACT_CALIBREADER_H
#define SENSEDATAEXTRACT_CALIBREADER_H

#include <fstream>
#include <eigen3/Eigen/Eigen>

class CalibBinReader {

public:
    FILE* fp;

    template<typename type>
    void Read(type& data, const int N = 1) {
        fread(&data, sizeof(type), N, fp);
    }

public:
    bool ReadCalib(std::string fileName);
    void CloseCalib();
    virtual bool GetRT(Eigen::Matrix4f &RT) = 0;
    virtual bool GetRGB_K(Eigen::Vector4f& K, int& rgb_w, int& rgb_h) = 0;
    virtual bool GetRGB_K(Eigen::Matrix3f& K, int& rgb_w, int& rgb_h) = 0;

    virtual bool GetToF_K(Eigen::Vector4f& K, int& tof_w, int& tof_h) = 0;
    virtual bool GetToF_K(Eigen::Matrix3f& K, int& tof_w, int& tof_h) = 0;
};

class CalibOPPOBinReader : public CalibBinReader {
    /// 316
    const static int RGB1_START = 18;
    const static int TOF_START = 6324;
    const static int RT_START = 7909;

    /// 516
//    const static int RGB1_START = 55;
//    const static int TOF_START = 1572;
//    const static int RT_START = 3074;
public:
    virtual bool GetRT(Eigen::Matrix4f &RT);
    virtual bool GetRGB_K(Eigen::Vector4f& K, int& rgb_w, int& rgb_h);
    virtual bool GetRGB_K(Eigen::Matrix3f& K, int& rgb_w, int& rgb_h);

    virtual bool GetToF_K(Eigen::Vector4f& K, int& tof_w, int& tof_h);
    virtual bool GetToF_K(Eigen::Matrix3f& K, int& tof_w, int& tof_h);
};

class CalibSamsungBinReader : public CalibBinReader {
public:
    virtual bool GetRT(Eigen::Matrix4f &RT);
    virtual bool GetRGB_K(Eigen::Vector4f& K, int& rgb_w, int& rgb_h);
    virtual bool GetRGB_K(Eigen::Matrix3f& K, int& rgb_w, int& rgb_h);

    virtual bool GetToF_K(Eigen::Vector4f& K, int& tof_w, int& tof_h);
    virtual bool GetToF_K(Eigen::Matrix3f& K, int& tof_w, int& tof_h);
};

class CalibTxtReader{
public:
    CalibTxtReader();
    CalibTxtReader(std::string path);


    std::string calib_path;

    std::string RT_name = "RT.txt";
    std::string KRGB_name = "K_rgb.txt";
    std::string KTOF_name = "K_tof.txt";

    bool GetRT(Eigen::Matrix4f &RT);
    bool GetRGB_K(Eigen::Matrix3f& K, int& rgb_w, int& rgb_h);
    bool GetToF_K(Eigen::Matrix3f& K, int& tof_w, int& tof_h);
};


#endif //SENSEDATAEXTRACT_CALIBREADER_H
