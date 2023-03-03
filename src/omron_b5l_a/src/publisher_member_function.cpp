/*---------------------------------------------------------------------------*/
/* Copyright(C)  2019-2020  OMRON Corporation                                */
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
 * File Name    : publisher_member_function.cpp
 * revision		: 0.1
 * Description  : main publisher module source file.
 ******************************************************************************/
/******************************************************************************
 * History      : DD.MM.YYYY Version Description
 * 				: 15.07.2020	0.1		Initial Version
 *****************************************************************************/

#include "publisher_member_function.hpp"
#include <unistd.h>
using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

int ShutDownFlag;

/* Topc string */
std::string Topic = "";

/* Output formate */
INT32 Tof_output_format = 0;

/*****************************************************************************
 * Function Name : terminate
 *****************************************************************************/
/**
 * @brief	This function is for stop communication and shutdown application
 *
 * @param[in/out]	None
 *
 * @return	None
 */
void MinimalPublisher::terminate_application()
{
		CTOFSample::Stop();
		rclcpp::shutdown();
}

/*****************************************************************************
 * Class Constructor : MinimalPublisher
 *****************************************************************************/
/**
 * @brief	Init variables of class
 */
MinimalPublisher::MinimalPublisher() : Node("minimal_publisher")
{
	frame_id_ = "omron";
	ShutDownFlag = 0;

	/*Create pulisher with topic as per config file 'OUTPUT_FORMAT' parameter*/
    /* Here 10 represents max number of messages in publisher queue*/
	publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(Topic, PUB_MSG_QUEUE_DEPTH);
	pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

	/* Create timer for 80ms to achive approx 10/12 Fps */
	timer_ = this->create_wall_timer(50ms, std::bind(&MinimalPublisher::timer_callback, this));
}

/*****************************************************************************
 * Function Name : timer_callback
 *****************************************************************************/
/**
 * @brief	This function is Timer callback function which publish ROS2 msg on particuler time interval
 *
 * @param[in/out]	None
 *
 * @return	None
 */
void MinimalPublisher::timer_callback()
{

	pcl::PointCloud<pcl::PointXYZI> cloud_;
	int Ret = CTOFSample::Run(&cloud_);
	//fprintf(stderr, "PUB A \n");
	if (Ret == 0)
	{
		/* Convert cloud data to ROS2 msg type */
		pcl::toROSMsg(cloud_, *pc2_msg_);
		pc2_msg_->header.frame_id = frame_id_;
		pc2_msg_->header.stamp = now();
		/* Publish ROS2 message */
		publisher_->publish(*pc2_msg_);

		//fprintf(stderr, "PUB OK\n");

	}
	else
	{
		ShutDownFlag = 1;
		terminate_application();
	}
}

/*****************************************************************************
 * Function Name : main
 *****************************************************************************/
/**
 * @brief	main function for init and start application
 *
 * @param[in/out]	None
 *
 * @return	0 on successful shutdown
 */
int main(int argc, char *argv[])
{
	std::string config_file = SOURCE_DIR_PREFIX;
	ShutDownFlag = 0;
	/* Load USB serial module */
	// system("sudo modprobe usbserial vendor=0x0590 product=0x00ca");
	rclcpp::init(argc, argv);

	config_file.append(CONFIG_FILE_PATH);

	std::cout<< "omron_b5l_a application version 0.1 started"<< std::endl;
	int Ret = CTOFSample::Init(config_file, Tof_output_format); // Init camera and UART
	/* Here we are supporting output format 257 || 258 || 1 || 2 */
	/* Format validation done in Init function and if fails then it returns non zero value */
	if (Ret == 0)
	{
		if (Tof_output_format == 257 || Tof_output_format == 258)
		{
			Topic = PUB_TOPIC_FRMT_257_258;
		}
		else
		{
			Topic = PUB_TOPIC_FRMT_001_002;
		}
		
		rclcpp::spin(std::make_shared<MinimalPublisher>());

		if (ShutDownFlag == 0)
		{			
			MinimalPublisher::terminate_application();
		}
	}

	return 0;
}
