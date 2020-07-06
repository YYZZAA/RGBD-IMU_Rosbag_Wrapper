#ifndef _SensetimeRGBDReconCore_H_
#define _SensetimeRGBDReconCore_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/// 句柄定义
typedef void *Handle;

/// 图像格式定义
typedef enum {
    ST_PIX_FMT_BGR888,      /// BGR  8:8:8   24bpp ( 3通道24bit BGR 像素 )
    ST_PIX_FMT_RGB888,      /// RGB  8:8:8   24bpp ( 3通道24bit RGB 像素 )
    ST_PIX_FMT_RGBA,        /// RGBA 8:8:8:8 32bpp ( 4通道32bit RGBA 像素 )
    ST_PIX_FMT_YUV420P,     /// YUV  4:2:0   12bpp ( 3通道, 一个亮度通道, 另两个为U分量和V分量通道, 所有通道都是连续的 )
    ST_PIX_FMT_GRAY,        /// GRAY
    ST_PIX_FMT_YUVNV21      /// 
} STImageFormat;

/// 图像方向定义
typedef enum
{
    ST_ORIENTATION_0 = 0,   /// 不做旋转预处理
    ST_ORIENTATION_90 = 1,  /// 预处理图像, 将输入图像顺时针旋转90度, 再执行后续操作
    ST_ORIENTATION_180 = 2, /// 预处理图像, 将输入图像顺时针旋转180度, 再执行后续操作
    ST_ORIENTATION_270 = 3, /// 预处理图像, 将输入图像顺时针旋转270度, 再执行后续操作
} STImageOrientation;

/// rgb图像定义
typedef struct STColorImage
{
    unsigned char *data = nullptr;              /// 图像数据指针
    STImageFormat format;             /// 像素格式
    int width;                        /// 宽度(以像素为单位)
    int height;                       /// 高度(以像素为单位)
    int stride;                       /// 跨度, 即每行所占的字节数
    STImageOrientation orientation;   /// 图像方向
    double time_stamp;           /// 时间戳
    double exposure_time;       
} STColorImage;

/// 深度图像定义
typedef struct STDepthImage
{
    unsigned short *data = nullptr;                /// 图像数据指针
    int width;                           /// 宽度(以像素为单位)
    int height;                          /// 高度(以像素为单位)
    STImageOrientation orientation;      /// 图像方向
    double time_stamp;              /// 时间戳
    double exposure_time;
} STDepthImage;

/// Raycasting
typedef struct STRayCastingResult
{
    int width;
    int height;
    float *points;
    float *normals;
    float *colors;
    int *unreliable_pt_index;
    int unreliable_pt_size;
} STRayCastingResult;

/// 相机内参定义
typedef struct STCameraIntrinsic
{
    int width, height;
    float fx, fy, cx, cy;
    float k1, k2, k3, p1, p2;
} STCameraIntrinsic;

/// RGBD相机标定参数
typedef struct STSensorCalib
{
    STCameraIntrinsic intrinsic_rgb;    /// rgb相机标定内参
    STCameraIntrinsic intrinsic_depth;  /// 深度相机标定内参
    float pos_depth2rgb[12];            /// depth相机到rgb相机的变换矩阵，[r00, r01, r02, ... , r20, r21, r22, t0, t1, t2]
} STSensorCalib;

/// 系统跟踪状态定义
typedef enum STTrackingState{
    TRACKING_NORMAL,                    /// 正常跟踪
    RELOCALIZING,                       /// 处于重定位状态
    TRACKING_FAIL                       /// 系统失败，需重启
} STTrackingState;

typedef enum STGlobalRelocState{
    SUCCESS,                    /// 正常跟踪
    TOO_SMALL,                       /// 处于重定位状态
    RELOC_FAIL                       /// 系统失败，需重启
} STGlobalRelocState;

/// 相机位姿定义
typedef struct STReconCamera {
    float rotation[3][3];               /// 旋转矩阵
    float translation[3];               /// 平移向量
} STReconCamera;

/// 单帧跟踪结果
typedef struct STTrackingResult {
    int info_length;                    /// debug info的长度
    STTrackingState tracking_state;     /// 系统跟踪状态
    STReconCamera camera;               /// 当前帧的相机位姿态
    STRayCastingResult raycasting;      /// raycasting结果
    float min_dist;
    float speed;
    int frame_buffer_size;              /// 当前缓存的帧数
    int progress;                       /// 扫描进度
    char reserved[56];                  /// 保留字段
    const char *info;                   /// debug信息
} STTrackingResult;

/// 顶点格式定义
typedef enum {
    ST_VERTEX_3D_POSITION = 0,                 /// 1个顶点3个float (位置)
    ST_VERTEX_3D_POSITION_NORMAL,              /// 1个顶点6个float (位置, 法向)
    ST_VERTEX_3D_POSITION_NORMAL_RGB,          /// 1个顶点9个float (位置, 法向, RGB), RGB颜色范围[0, 1]
    ST_VERTEX_3D_POSITION_NORMAL_RGBA,         /// 1个顶点10个float (位置, 法向, RGBA), RGBA颜色范围[0, 1]
    ST_VERTEX_3D_POSITION_RGB                  /// 1个顶点6个float (位置, RGB), RGB颜色范围[0, 1]
} STVertexFormat;

typedef enum {
    ST_RELEASE,
    ST_DEBUG
} STReconMode;

typedef enum {
    ST_IMU,
    ST_GRAVITY_ONLY
} STIMUOption;

typedef enum {
    ST_NORMAL,
    ST_HIGH_RESOLUTION
} STReconResolution;

typedef enum {
    ST_BACKGROUND,
    ST_FOREGROUND,
    ST_BOUNDARY,
    ST_PLANE
} STMaskType;

/// 点云结构定义
typedef struct STPointCloud {
    STVertexFormat format;         /// 点云顶点结构
    int point_cloud_size;          /// point_cloud长度
    char reserved[64];             /// 保留字段
    float *point_cloud;            /// 点云数据，null如果长度为0
} STPointCloud;

/// mesh结构定义
typedef struct STRawMesh {
    STVertexFormat format;          /// mesh顶点结构
    int mesh_v_size;                /// mesh顶点数组长度
    int mesh_f_size;                /// mesh面片数组长度
    char reserved[64];              /// 保留字段
    float *mesh_v;                  /// mesh顶点数组，null如果数组长度为0
    int *mesh_f;                    /// mesh面片数组，null如果数组长度为0
} STRawMesh;

/// 带纹理的mesh结构
typedef struct STTextureMesh {
    STRawMesh raw_mesh;             /// mesh模型

    STImageFormat texture_format;   /// 纹理图像格式
    int width;                      /// 纹理图像宽
    int height;                     /// 纹理图像长
    int uv_size;                    /// texture_uv数组长度

    char reserved[64];              /// 保留字段

    unsigned char *texture_image;   /// 纹理图像数据
    float *texture_uv;              /// 纹理uv数据
} STTextureMesh;

typedef struct STIMU
{
    double acceleration[6];    /* raw acceleration data in gravity unit. */
    double gyroscope[6];    /* raw angular velocity data in radian */
    double time_stamp;       /* timestamp in second */
} STIMU;

typedef struct STAttitude
{
    double quaternion[4];    /* attitude of the device */
    double gravity[3];        /* gravity direction */
    double time_stamp;       /* timestamp in second */
} STAttitude;


// https://developer.android.com/reference/android/hardware/Sensor.html#TYPE_ACCELEROMETER_UNCALIBRATED
typedef struct STAcceleration
{
    double data[6];             /* index [0-2] is raw data, index [3-5] is bias*/
    double time_stamp;
} STAcceleration;

// https://developer.android.com/reference/android/hardware/Sensor.html#TYPE_GYROSCOPE_UNCALIBRATED
typedef struct STGyroscope
{
    double data[6];             /* index [0-2] is raw data, index [3-5] is bias */
    double time_stamp;
} STGyroscope;

typedef struct STGravity {
    double data[3];
    double time_stamp;
} STGravity;

typedef struct STRotationVector {
    double data[4];
    double time_stamp;
} STRotationVector;

typedef struct STRGBDReconConfig
{
    int config_size;
    char *config;

    int voc_length;
    char *voc_data;

    int license_length;
    char *license;
    
    STReconMode mode;

    STReconResolution resolution;

    STIMUOption imu_option = ST_IMU;
    
    bool batch_test_old = false;
} STRGBDReconConfig;

typedef struct STRGBDReconOption
{
    // Maximum number of faces as target face number for model simplification.
    // If max_face_count <= 0 or larger than the face number of the model, no simplification is done for the model.
    int max_face_count = -1;
} STRGBDReconOption;

#ifdef __cplusplus
}
#endif

#endif

