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

#include <ctype.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/stat.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "TOFApiZ.hpp"
#include "uart.h"
#pragma once

class CTOFSample
{
public:
    CTOFSample(void);
    ~CTOFSample(void);
    static int Run(pcl::PointCloud<pcl::PointXYZI> *cloud_);
    static void Stop();
    static int PCD_xyz_cloud(unsigned char *puImage, pcl::PointCloud<pcl::PointXYZI> *cloud_);
    static int PCD_xyzi_cloud(unsigned char *puImage, pcl::PointCloud<pcl::PointXYZI> *cloud_);
    static int ImageOutput(INT32 ToF_Format, unsigned char *puImage, pcl::PointCloud<pcl::PointXYZI> *cloud_) ;
    static int SetParameter(const char *pcFileName);
    static int Init(std::string config_file, INT32 & Tof_output_format);
};
