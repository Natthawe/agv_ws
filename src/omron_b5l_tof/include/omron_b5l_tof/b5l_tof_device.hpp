#include <string>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "TOFApiZ.hpp"
#include "uart.h"
#pragma once

typedef struct
{
    std::string frame_id;
    std::string device;
    double frequency;
    int fov_limit;
    std::string image_path;
    int output_format;
    int operation_mode;
    std::vector<int64_t> exposure_fps;
    std::vector<int64_t> t3d_xyz;
    int led_frequency_id;
    int min_amp_all;
    int min_amp_near;
    int operation_check_led;
    std::vector<int64_t> response_speed;
    int enr_treshold;
} ConfigParam;

class b5l_tof_device
{
private:
public:
    static int Init(ConfigParam &config_param, int baudrate, INT32 &Tof_output_format);
    static int SetParameter(ConfigParam &conf);
    static void Stop();
    static int Run(pcl::PointCloud<pcl::PointXYZI> *cloud_);
    static int ImageOutput(INT32 ToF_Format, unsigned char *puImage, pcl::PointCloud<pcl::PointXYZI> *cloud_);
    
    /** xyz */ 
    static int PCD_xyz_cloud(unsigned char *puImage, pcl::PointCloud<pcl::PointXYZI> *cloud_);

    /** xyzi */
    static int PCD_xyzi_cloud(unsigned char *puImage, pcl::PointCloud<pcl::PointXYZI> *cloud_);
};