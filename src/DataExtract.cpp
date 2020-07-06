//
// Created by sensetime on 20-4-27.
//

#include "DataExtract.h"

bool ExtractInputDataFromTxt(std::string fileName, STColorImage &rgb, STDepthImage &depth, STAttitude &att) {
    //std::cout << fileName << std::endl;
    std::ifstream file(fileName, std::ios::binary);
    if (!file.is_open()) {
        std::cout << "fail to open data file" << std::endl;
        return false;
    }

    file.seekg (0, file.end);
    int length = file.tellg();
    file.seekg (0, file.beg);


    int width, height;
    double timestamp;
    file.read((char *) &width, sizeof(width));
    file.read((char *) &height, sizeof(height));
    file.read((char *) &timestamp, sizeof(timestamp));

    rgb.width = width;
    rgb.height = height;
    rgb.time_stamp = timestamp;

    int yuv_size = width * height * 3 / 2;

    if (rgb.data == nullptr) {
        rgb.data = new unsigned char[yuv_size];
    }

    file.read((char *) (rgb.data), sizeof(char) * yuv_size);

    file.read((char *) &width, sizeof(width));

    file.read((char *) &height, sizeof(height));

    depth.width = width;
    depth.height = height;

    if (depth.data == nullptr) {
        depth.data = new ushort[width * height];
    }
    file.read((char *) depth.data, sizeof(ushort) * width * height);

    int current_lenth = file.tellg();
    if(length - current_lenth < sizeof(STAttitude)){
        memset(&att, 0 ,sizeof(STAttitude));
        return true;
    }
    file.read((char *) &att, sizeof(STAttitude) *1);
    return true;
}

bool ExtractIMUData(std::string fileName, std::vector<STAcceleration> &accelerations,
                    std::vector<STGyroscope> &gyros,
                    STGravity &g, STRotationVector &r){

    accelerations.clear();
    gyros.clear();
    std::ifstream file(fileName, std::ios::binary);
    if (!file.is_open()) {
        std::cout << "fail to open imu file: " <<fileName<< std::endl;
        return false;
    }

    int n_imu = 0;
    file.read((char *) &n_imu, sizeof(int) *1);

    bool use_old = true;
    if(use_old){
        old_IMUData imu_data;
        for(int i = 0; i < n_imu; ++i) {
            file.read((char *) &imu_data, sizeof(imu_data) *1);
            STAcceleration acc;
            acc.time_stamp = imu_data.t;
            memcpy( acc.data, imu_data.acc.data(), sizeof(double) * 3);
            acc.data[3] = 0; acc.data[4] = 0; acc.data[5] = 0;
            accelerations.push_back(acc);
            STGyroscope gyr;
            gyr.time_stamp = imu_data.t;
            memcpy(gyr.data, imu_data.gyr.data(), sizeof(double) * 3);
            gyr.data[3] = 0; gyr.data[4] = 0; gyr.data[5] = 0;
            gyros.push_back(gyr);
        }
    }else{
        IMUData imu_data;
        for(int i = 0; i < n_imu; ++i) {
            file.read((char *) &imu_data, sizeof(imu_data) *1);
            STAcceleration acc;
            acc.time_stamp = imu_data.t;
            memcpy( acc.data, imu_data.acc.data(), sizeof(double) * 3);
            memcpy( acc.data+3, imu_data.acc_bias.data(), sizeof(double) * 3);
            accelerations.push_back(acc);
            STGyroscope gyr;
            gyr.time_stamp = imu_data.t;
            memcpy(gyr.data, imu_data.gyr.data(), sizeof(double) * 3);
            memcpy(gyr.data+3, imu_data.gyr_bias.data(), sizeof(double) * 3);
            gyros.push_back(gyr);
        }
    }


    Attitude attitude;

    double qx, qy, qz, qw;
    file.read((char*) &qx, sizeof(double));
    file.read((char*) &qy, sizeof(double));
    file.read((char*) &qz, sizeof(double));
    file.read((char*) &qw, sizeof(double));
    attitude.q = Eigen::Quaterniond(qw, qx, qy, qz);

    file.read((char*) &attitude.g, sizeof(double) * 3);
    file.read((char*) &attitude.t, sizeof(double) * 1);

    g.time_stamp = attitude.t;
    g.data[0] = attitude.g[0];
    g.data[1] = attitude.g[1];
    g.data[2] = attitude.g[2];

    r.time_stamp = attitude.t;
    r.data[0] = qx;
    r.data[1] = qy;
    r.data[2] = qz;
    r.data[3] = qw;

    file.close();
    return true;

}

bool ExtractInputDataFromBin(
        std::string fileName,
        STColorImage &rgb, STDepthImage &depth, float *matrix,
        std::vector<STAcceleration> &accelerations,
        std::vector<STGyroscope> &gyros,
        STGravity &g, STRotationVector &r)
{
    std::cout << fileName << std::endl;
    std::ifstream file(fileName, std::ios::binary);
    if (!file.is_open()) {
        std::cout << "fail to open data file" << std::endl;
        return false;
    }

    file.seekg (0, file.end);
    int length = file.tellg();
    file.seekg (0, file.beg);

    /// rgb
    int width, height;
    int format, stride;
    int orientation;
    double timestamp;
    file.read((char *) &width, sizeof(width));
    file.read((char *) &height, sizeof(height));
    file.read((char *) &format, sizeof(format));
    file.read((char *) &stride, sizeof(stride));
    file.read((char *) &orientation, sizeof(orientation));
    file.read((char *) &timestamp, sizeof(timestamp));

    rgb.width = width;
    rgb.height = height;
    rgb.exposure_time = DEAULT_EXPOSURE_TIME;
    rgb.time_stamp = timestamp;
    int yuv_size = width * height * 3 / 2;
    if (rgb.data == nullptr) {
        rgb.data = new unsigned char[yuv_size];
    }
    file.read((char *) (rgb.data), sizeof(char) * yuv_size);

    /// depth
    file.read((char *) &width, sizeof(width));
    file.read((char *) &height, sizeof(height));
    file.read((char *) &format, sizeof(format));
    file.read((char *) &stride, sizeof(stride));
    file.read((char *) &orientation, sizeof(orientation));
    file.read((char *) &timestamp, sizeof(timestamp));
    depth.width = width;
    depth.height = height;
    depth.exposure_time = DEAULT_EXPOSURE_TIME;
    depth.time_stamp = timestamp;
    if (depth.data == nullptr) {
        depth.data = new ushort[width * height];
    }
    file.read((char *) depth.data, sizeof(ushort) * width * height);

    /// model pose
    file.read((char *) matrix, sizeof(float) * 16);

    /// imu data
    int imu_size;
    file.read((char *) &imu_size, sizeof(imu_size));

    for (int i = 0; i < imu_size; ++ i)
    {
        STAcceleration acc;
        file.read((char *) acc.data, sizeof(double) * 6);

        STGyroscope gyr;
        file.read((char *) gyr.data, sizeof(double) * 6);

        file.read((char *) &timestamp, sizeof(timestamp));

        acc.time_stamp = timestamp;
        gyr.time_stamp = timestamp;

        accelerations.emplace_back(acc);
        gyros.emplace_back(gyr);
    }

    file.read((char *)r.data, sizeof(double) * 4);
    file.read((char *)g.data, sizeof(double) * 3);

    file.read((char *) &timestamp, sizeof(double));

    r.time_stamp = timestamp;
    g.time_stamp = timestamp;

    return true;
}



