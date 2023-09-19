#include <rclcpp/rclcpp.hpp>
#include "b5l_tof_device.hpp"

#include <pthread.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <iostream>
#include <cstdlib>
#include <pcl/filters/voxel_grid.h>

using namespace std::chrono_literals;

/* Topic string macro */
#define PUB_TOPIC_FRMT_001_002 "front/pointcloud2_xyz"
#define PUB_TOPIC_FRMT_257_258 "front/pointcloud2_xyzi"

class omron_b5l_tof : public rclcpp::Node
{
private:
    std::string frame_id = "omron_b5l_tof";
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
	pcl::PointCloud<pcl::PointXYZI> cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_;
    void timer_callback();
    void load_paramiters(ConfigParam &config_param);

public:
    omron_b5l_tof();
    static void terminate_application();
};