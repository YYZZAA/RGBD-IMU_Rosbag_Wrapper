/*modified on 2020-7-2
 only suitable for data format of SenseData*/
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/program_options.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include<cstring>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "PrettyIFStream.hpp"
#include "DataExtract.h"
#include "CalibReader.h"

// Macros used to copy a cv::Mat into an array
#define CP_MAT_TO_ARRAY(m, a) { for(auto i=0; i<(m).rows; i++) for(auto j=0; j<(m).cols; j++) (a)[i*(m).cols + j] = (m).ptr<double>(i)[j]; }
#define CP_ARRAY_TO_MAT(a, m) { for(auto i=0; i<(m).rows; i++) for(auto j=0; j<(m).cols; j++) (m).ptr<double>(i)[j] = (a)[i*(m).cols + j]; }

Eigen::Vector3d TIC (0.0, 0.0, 0.0) ;
Eigen::Quaterniond QIC = Eigen::Quaterniond{0.0, 0.707107, -0.707107, 0.0};


#define ORIG_INVALID_DEPTH 5000
#define TOF_VALID_DEPTH_THRES  1200

/// preprocess for warp raw depth map
std::vector<Eigen::Vector3f> warp_Vs_;
Eigen::Vector3f warp_T_;


void Diffusion(cv::Mat &raw_depth) {
    int pixels = raw_depth.rows * raw_depth.cols;
    ushort *raw_depth_ptr = (ushort*)raw_depth.data;

    for (int i = 0; i < pixels; ++ i, ++ raw_depth_ptr){
        if (*raw_depth_ptr >= TOF_VALID_DEPTH_THRES)
            *raw_depth_ptr = 0;
    }
    // return;

    cv::Mat confidence(raw_depth.size(), CV_32FC1, cv::Scalar(0.0f));
    float *confidence_ptr = (float*)confidence.data;
    raw_depth_ptr = (ushort*)raw_depth.data;
    for (int i = 0; i < pixels; ++ i, ++ confidence_ptr, ++ raw_depth_ptr) {
        *confidence_ptr = *raw_depth_ptr == 0 ? 0.0f : 1.0f;
    }
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * 1 + 1, 2 * 1 + 1));
    cv::dilate(confidence, confidence, element);

    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * 2 + 1, 2 * 2 + 1));
    cv::erode(confidence, confidence, element);

    confidence_ptr = (float*)confidence.data;
    raw_depth_ptr = (ushort*)(raw_depth.data);
    for (int i = 0; i < pixels; ++ i, ++ raw_depth_ptr, ++ confidence_ptr){
        if (*confidence_ptr < 0.8f) *raw_depth_ptr = 0;
    }
}
std::string myint2str(int a) {
    std::string ret = "";
    while (a != 0) {
        ret.push_back(char(a % 10+'0'));
        a /= 10;
    }
    return ret;
}
int mystr2int(std::string a){
    int ret=0;
    int p=1;
    int len=a.length();
    for(int i=len-1;i>=0;--i){
        ret+=int(a[i]-'0')*p;
        p*=10;
    }
    return ret;
}
void FastWarpRawDepthMap(cv::Mat &result, const cv::Mat &depth, int width, int height,
        const Eigen::Matrix3f & depth_K,
        const Eigen::Matrix3f & rgb_K,
        const Eigen::Matrix4f & RT, bool is_first) {
    /// 预处理ToF到RGB的旋转平移
    if(is_first){
        Eigen::Matrix3f warp_R = rgb_K * RT.block<3,3>(0,0) * depth_K.inverse();
        warp_T_ = rgb_K * RT.block<3,1>(0,3);

        warp_Vs_.resize(depth.cols * depth.rows);
        for (int y = 0; y < depth.rows; ++y) {
            for (int x = 0; x < depth.cols; ++x) {
                int id = y * depth.cols + x;
                warp_Vs_[id] = warp_R * Eigen::Vector3f(x, y, 1);
            }
        }
    }

    if (result.cols != width || result.rows != height || result.type() != CV_16UC1) {
        result = cv::Mat(height, width, CV_16UC1);
    }
    result.setTo(0);

    int pixel = depth.rows * depth.cols;
    ushort *depth_ptr = (ushort*)depth.data;

    int c = 0;
#if __ARM_NEON
    for ( ; c + 4 <= pixel; c += 4, depth_ptr += 4) {
        uint16x4_t depth_u16_0123 = vld1_u16(depth_ptr);
        uint64_t depth_u64_packed = vget_lane_u64(vreinterpret_u64_u16(depth_u16_0123), 0);

        uint32x4_t c_0_u32_0123 = vdupq_n_u32(0);
        uint32x4_t depth_u32_0123 = vmovl_u16(depth_u16_0123);
        uint32x4_t flag_valid_0123 = vcgtq_u32(depth_u32_0123, c_0_u32_0123);
        flag_valid_0123 = vandq_u32(flag_valid_0123, vcleq_u32(depth_u32_0123, vdupq_n_u32(ORIG_INVALID_DEPTH)));
        if (depth_u64_packed == 0) continue;

        int idx = c;
        float32x4x3_t proj_pt_xyz_0123 = vld3q_f32((float *)(&warp_Vs_[idx]));
        float32x4_t depth_f32_0123 = vcvtq_f32_u32(depth_u32_0123);
        float32x4_t warp_T_x_0123 = vdupq_n_f32(warp_T_[0]);
        float32x4_t warp_T_y_0123 = vdupq_n_f32(warp_T_[1]);
        float32x4_t warp_T_z_0123 = vdupq_n_f32(warp_T_[2]);
        float32x4_t proj_pt_x_0123 = vmlaq_f32(warp_T_x_0123, proj_pt_xyz_0123.val[0], depth_f32_0123);
        float32x4_t proj_pt_y_0123 = vmlaq_f32(warp_T_y_0123, proj_pt_xyz_0123.val[1], depth_f32_0123);
        float32x4_t proj_pt_z_0123 = vmlaq_f32(warp_T_z_0123, proj_pt_xyz_0123.val[2], depth_f32_0123);

        float32x4_t inv_proj_pt_z_0123 = vrecpeq_f32(proj_pt_z_0123);
        inv_proj_pt_z_0123 = vmulq_f32(vrecpsq_f32(proj_pt_z_0123, inv_proj_pt_z_0123), inv_proj_pt_z_0123);
        inv_proj_pt_z_0123 = vmulq_f32(vrecpsq_f32(proj_pt_z_0123, inv_proj_pt_z_0123), inv_proj_pt_z_0123);

        float32x4_t c_0_5_x4 = vdupq_n_f32(0.5f);
        float32x4_t x_f32_0123 = vmlaq_f32(c_0_5_x4, proj_pt_x_0123, inv_proj_pt_z_0123);
        float32x4_t y_f32_0123 = vmlaq_f32(c_0_5_x4, proj_pt_y_0123, inv_proj_pt_z_0123);
        int32x4_t x_0123 = vcvtq_s32_f32(x_f32_0123);
        int32x4_t y_0123 = vcvtq_s32_f32(y_f32_0123);

        int32x4_t c_0_x4 = vdupq_n_s32(0);
        flag_valid_0123 = vandq_u32(flag_valid_0123, vcgeq_s32(x_0123, c_0_x4));
        flag_valid_0123 = vandq_u32(flag_valid_0123, vcgeq_s32(y_0123, c_0_x4));
        flag_valid_0123 = vandq_u32(flag_valid_0123, vcltq_s32(x_0123, vdupq_n_s32(width)));
        flag_valid_0123 = vandq_u32(flag_valid_0123, vcltq_s32(y_0123, vdupq_n_s32(height)));

        uint16x4_t proj_pt_z_u16_0123 = vmovn_u32(vcvtq_u32_f32(proj_pt_z_0123));
        if (vgetq_lane_u32(flag_valid_0123, 0)) {
            int y = vgetq_lane_s32(y_0123, 0), x = vgetq_lane_s32(x_0123, 0);
            uint16_t old_val = result.at<ushort>(y, x);
            uint16_t new_val = vget_lane_u16(proj_pt_z_u16_0123, 0);
            if (old_val == 0 || old_val > new_val){
                result.at<ushort>(y, x) = new_val;
            }
        }
        if (vgetq_lane_u32(flag_valid_0123, 1)) {
            int y = vgetq_lane_s32(y_0123, 1), x = vgetq_lane_s32(x_0123, 1);
            uint16_t old_val = result.at<ushort>(y, x);
            uint16_t new_val = vget_lane_u16(proj_pt_z_u16_0123, 1);
            if (old_val == 0 || old_val > new_val){
                result.at<ushort>(y, x) = new_val;
            }
        }
        if (vgetq_lane_u32(flag_valid_0123, 2)) {
            int y = vgetq_lane_s32(y_0123, 2), x = vgetq_lane_s32(x_0123, 2);
            uint16_t old_val = result.at<ushort>(y, x);
            uint16_t new_val = vget_lane_u16(proj_pt_z_u16_0123, 2);
            if (old_val == 0 || old_val > new_val){
                result.at<ushort>(y, x) = new_val;
            }
        }
        if (vgetq_lane_u32(flag_valid_0123, 3)) {
            int y = vgetq_lane_s32(y_0123, 3), x = vgetq_lane_s32(x_0123, 3);
            uint16_t old_val = result.at<ushort>(y, x);
            uint16_t new_val = vget_lane_u16(proj_pt_z_u16_0123, 3);
            if (old_val == 0 || old_val > new_val){
                result.at<ushort>(y, x) = new_val;
            }
        }
    }
#endif

    for( ; c < pixel; c++, ++ depth_ptr){
        const ushort &d = *depth_ptr;
        if (d == 0) continue;
        if( d > ORIG_INVALID_DEPTH ) continue;

        int idx = c;
        Eigen::Vector3f proj_pt = warp_Vs_[idx] * d + warp_T_;

        int x = proj_pt[0]/proj_pt[2] + 0.5;
        int y = proj_pt[1]/proj_pt[2] + 0.5;

        if(x<0||x>=width) continue;
        if(y<0||y>=height) continue;

        if(result.at<ushort>(y,x)==0 || result.at<ushort>(y,x)>proj_pt[2]){
            result.at<ushort>(y,x) = proj_pt[2];
        }
    }
}

int main(int argc, char** argv)
{
    rosbag::Bag bag;

    std::string base_data_path =argv[1];
    std::string str_num=argv[2];
    int num_data=mystr2int(str_num);
    std::string output = argv[3];
    bag.open(output, rosbag::bagmode::Write);

    //Calib Reader 
    CalibOPPOBinReader calibReader;
    calibReader.ReadCalib(base_data_path + "/calib.bin");

    Eigen::Matrix3f rgb_k;
    int rgb_width, rgb_height;
    calibReader.GetRGB_K(rgb_k, rgb_width, rgb_height);

    std::cout << "rgb width and height is: " << rgb_width << " " << rgb_height << std::endl;
    std::cout << rgb_k << std::endl;

    Eigen::Matrix3f tof_k;
    int tof_width, tof_height;
    calibReader.GetToF_K(tof_k, tof_width, tof_height);

    std::cout << "tof width and height is: " << tof_width << " " << tof_height << std::endl;
    std::cout << tof_k << std::endl;

    Eigen::Matrix4f RT;
    calibReader.GetRT(RT);
    std::cout << "extrinsic is: " << std::endl;
    std::cout << RT << std::endl;
    calibReader.CloseCalib();


    //Load Images
    std::string strPath = base_data_path;
    std::cout<<"Loading images from "<<strPath<<std::endl;
    STColorImage inputRGBImage;
    STDepthImage inputDepthImage;
    STAttitude attitude;

    char buffer[256];

    int width = 512;
    int height = 384;

    Eigen::Matrix3f resize_k = Eigen::Matrix3f::Identity();
    resize_k.row(0) = rgb_k.row(0) * tof_width / float(rgb_width);
    resize_k.row(1) = rgb_k.row(1) * tof_height / float(rgb_height);

    cv::Mat nan_depth;
    cv::Mat resize_depth_f32;
    cv::Mat resize_depth(height, width, CV_16UC1);
    for (int iFrame = 0; iFrame < num_data; iFrame += 1)
    {
        sprintf(buffer, "000_%.6d.txt", iFrame);
        std::string fileName(buffer);
        if (!ExtractInputDataFromTxt(strPath +"/"+fileName, inputRGBImage,
                                     inputDepthImage, attitude)) {
            break;
        }
        cv::Mat yuv_nv21(inputRGBImage.height*3/2, inputRGBImage.width,  CV_8UC1, inputRGBImage.data);
        cv::Mat depth(inputDepthImage.height, inputDepthImage.width,  CV_16UC1, inputDepthImage.data);

        Diffusion(depth);
        cv::Mat warped_depth;


        FastWarpRawDepthMap(warped_depth, depth, depth.cols, depth.rows, tof_k, resize_k, RT, iFrame==0);

        warped_depth.convertTo(nan_depth, CV_32FC1, 0.001);
        int pix_size = nan_depth.rows * nan_depth.cols;
        float *f_ptr = (float*)(nan_depth.data);
        for (int i = 0; i < pix_size; ++ i, ++ f_ptr){
            if (*f_ptr < FLT_EPSILON) *f_ptr = 100000.f;
        }
        cv::resize(nan_depth, resize_depth_f32, cv::Size(width, height));
        ushort *u_ptr = (ushort*)resize_depth.data;
        f_ptr=(float*)resize_depth_f32.data;
        pix_size = resize_depth_f32.rows * resize_depth_f32.cols;
        for (int c = 0 ; c < pix_size; c++, ++ f_ptr, ++ u_ptr) {
            *u_ptr = std::min(65535.0f, 0.5f + *f_ptr * 1000.0f);
        }

        cv::Mat rgb;
        cv::cvtColor(yuv_nv21, rgb, CV_YUV2RGB_NV21);
        cv::resize(rgb, rgb,cv::Size(width, height));

//        cv::imshow("rgb", rgb);
//        cv::waitKey();
//        cv::imshow("resize_depth", resize_depth*35);
//        cv::waitKey();
        Eigen::Map<Eigen::Vector3d> gravity(attitude.gravity);
        Eigen::Map<Eigen::Quaterniond> atti(attitude.quaternion);

        //std::cout<<"gravity is: "<<gravity.transpose()<<std::endl;
        //std::cout<<"atti is: "<<atti.x()<<" "<<atti.y()<<" "<<atti.z()<<" "<<atti.w()<<std::endl;
        cv_bridge::CvImage imgrgb;
        imgrgb.image = rgb;
        imgrgb.encoding = "rgb8";
        cv_bridge::CvImage imgdepth;
        imgdepth.image=resize_depth*35;
        imgrgb.encoding = "mono16";

        double t=inputRGBImage.time_stamp;
        ros::Time stamp;
        stamp.fromSec(t);

        sensor_msgs::ImagePtr ros_rgb_msg;
        ros_rgb_msg = imgrgb.toImageMsg();
        ros_rgb_msg->header.seq = iFrame;
        ros_rgb_msg->header.stamp = stamp;
        ros_rgb_msg->header.frame_id = iFrame;
        bag.write("cam0/image_raw", stamp,ros_rgb_msg);

        sensor_msgs::ImagePtr ros_depth_msg;
        ros_depth_msg = imgdepth.toImageMsg();
        ros_depth_msg->header.seq = iFrame;
        ros_depth_msg->header.stamp = stamp;
        ros_depth_msg->header.frame_id = iFrame;
        bag.write("cam0/depth", stamp, ros_depth_msg);

        cv_bridge::CvImageConstPtr ptr1;
        ptr1 = cv_bridge::toCvCopy(ros_rgb_msg, sensor_msgs::image_encodings::RGB8);
        cv::imshow("test bag1",ptr1->image);
        cv::waitKey(0);

        /*cv_bridge::CvImageConstPtr ptr2;
        ptr2 = cv_bridge::toCvCopy(ros_depth_msg, sensor_msgs::image_encodings::MONO16);
        cv::imshow("test bag2",ptr2->image);
        cv::waitKey(0);*/
    }
    
    
    //Load IMUdata
    std::cout<<"Loading IMU from "<<strPath<<std::endl;
    std::vector<STAcceleration> accelerations;
    std::vector<STGyroscope> gyros;
    STGravity g; STRotationVector r;
    for (int iFrame = 0; iFrame < num_data; iFrame += 1) {
        sprintf(buffer, "000_%.6d.imu", iFrame);
        std::string fileName(buffer);
        ExtractIMUData(strPath + "/"+fileName, accelerations, gyros, g, r);
        int imu_len=accelerations.size();
        for(int imu_cnt=0;imu_cnt<imu_len;imu_cnt++){
            const STAcceleration tmpacc=accelerations[imu_cnt];
            const STGyroscope tmpgyros=gyros[imu_cnt];
            ros::Time stamp; 
            double t=tmpacc.time_stamp;
            stamp.fromSec(t);
        //std::cout<<"acc size: "<<accelerations.size()<<std::endl;
        //std::cout<<"gyr size: "<<gyros.size()<<std::endl;

        //Eigen::Map<Eigen::Vector3d> gravity(g.data);
        //Eigen::Map<Eigen::Quaterniond> atti(r.data);
        //std::cout<<"gravity is: "<<gravity.transpose()<<std::endl;
        //std::cout<<"atti is: "<<atti.x()<<" "<<atti.y()<<" "<<atti.z()<<" "<<atti.w()<<std::endl;
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = stamp;
            imu_msg.header.seq = iFrame;
            imu_msg.header.frame_id = "imu";
            imu_msg.angular_velocity.x = tmpgyros.data[0];
            imu_msg.angular_velocity.y = tmpgyros.data[1];
            imu_msg.angular_velocity.z = tmpgyros.data[2];
            imu_msg.linear_acceleration.x = tmpacc.data[0];
            imu_msg.linear_acceleration.y = tmpacc.data[1];
            imu_msg.linear_acceleration.z = tmpacc.data[2];

            imu_msg.angular_velocity_covariance[0] = 0.001f;
            imu_msg.angular_velocity_covariance[1] = 0.0f;
            imu_msg.angular_velocity_covariance[2] = 0.0f;
            imu_msg.angular_velocity_covariance[3] = 0.0f;
            imu_msg.angular_velocity_covariance[4] = 0.001f;
            imu_msg.angular_velocity_covariance[5] = 0.0f;
            imu_msg.angular_velocity_covariance[6] = 0.0f;
            imu_msg.angular_velocity_covariance[7] = 0.0f;
            imu_msg.angular_velocity_covariance[8] = 0.001f;
            imu_msg.linear_acceleration_covariance = imu_msg.orientation_covariance = imu_msg.angular_velocity_covariance;
            bag.write("/imu0", stamp, imu_msg);
        }
    }
    std::cout<<"Created Bag Successfully"<<std::endl;       
    bag.close();
}
