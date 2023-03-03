/*---------------------------------------------------------------------------*/
/* Copyright(C)  2020  OMRON Corporation                                     */
/*                                                                           */
/* Licensed under the Apache License, Version 2.0 (the "License");           */
/* you may not use this file except in compliance with the License.          */
/* You may obtain a copy of the License at                                   */
/*                                                                           */
/*     http://www.apache.org/licenses/LICENSE-2.0                            */
/*                                                                           */
/* Unless required by applicable law or agreed to in writing, software       */
/* distributed under the License is distributed on an "AS IS" BASIS,         */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  */
/* See the License for the specific language governing permissions and       */
/* limitations under the License.                                            */
/*---------------------------------------------------------------------------*/

/*******************************************************************************
 * File Name    : publisher_member_function.hpp
 * revision		: 0.1
 * Description  : main publisher module source file.
 ******************************************************************************/
/******************************************************************************
 * History      : DD.MM.YYYY Version Description
 * 				: 15.07.2020	0.1		Initial Version
 *****************************************************************************/

#include <chrono>
#include <memory>
#include <iostream>
#include <cstdlib>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ToF_Sample.hpp"

using namespace std::chrono_literals;

/* Maximum number of ROS message queue */
#define PUB_MSG_QUEUE_DEPTH 100

/* Topic string macro */
#define PUB_TOPIC_FRMT_001_002 "pointcloud2_xyz"
#define PUB_TOPIC_FRMT_257_258 "pointcloud2_xyzi"

/* Config file string */
#define CONFIG_FILE_PATH "/src/config/ToF_Sample.prm"


class MinimalPublisher : public rclcpp::Node
{

private:

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
	pcl::PointCloud<pcl::PointXYZI> cloud_;

	sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_;
	std::string frame_id_;
	
public:

	MinimalPublisher();
	int load_pcd();	
	static void terminate_application();
    void timer_callback();
};
