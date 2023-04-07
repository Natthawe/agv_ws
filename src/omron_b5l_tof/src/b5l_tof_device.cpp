#include "b5l_tof_device.hpp"

using PointT = pcl::PointXYZI;

#define TOF_TIMEOUT 1000 /* ms (Minimum: 500ms) */


/* Tof Output Format */
INT32 ToF_Format;

/* Flag for deleting the pixels out-of-FOV */
int FOV_Limitation = 0,
    PointNum = 76800,
    InFOV[76800];

char ImagePath[256] = "";
UINT8 outStatus, *outData;

int b5l_tof_device::SetParameter(ConfigParam &conf)
{
    int ret = 1;
    /* Set Image Path */
    strcpy(ImagePath, conf.image_path.c_str());

    /* Set FOV Limitation */
    FOV_Limitation = conf.fov_limit;

    /* Set Output Format */
    if (conf.output_format != ToF_Format)
    {
        if ((ret = CTOFApiZ::setOutputFormat(TOF_TIMEOUT, conf.output_format, &outStatus)) != TOF_NO_ERROR)
        {
            RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::setOutputFormat() has been terminated with error code %d.\n", ret);
        }
        ToF_Format = conf.output_format;
    }

    /* Set Operation Mode */
    if ((ret = CTOFApiZ::setOperationMode(TOF_TIMEOUT, conf.operation_mode, &outStatus)) != TOF_NO_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::setOperationMode() has been terminated with error code %d.\n", ret);
    }
    if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for GetOperationMode(0x87) command.\n", outStatus);
    }
    if (conf.operation_mode != outData[0])
    {
        if ((ret = CTOFApiZ::setOperationMode(TOF_TIMEOUT, conf.operation_mode, &outStatus)) != TOF_NO_ERROR)
        {
            RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::setOperationMode() has been terminated with error code %d.\n", ret);
        }
    }

    /* Set Exposure FPS */
    if ((ret = CTOFApiZ::getExposureTime(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::getExposureTime() has been terminated with error code %d.\n", ret);
    }
    if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for GetExposureTime(0x89) command.\n", outStatus);
    }
    int exposure_time = outData[0] * 256 + outData[1];
    int fps = outData[6];
    if (conf.exposure_fps[0] != exposure_time || conf.exposure_fps[0] != fps)
    {
        if ((ret = CTOFApiZ::setExposureTime(TOF_TIMEOUT, conf.exposure_fps[0], conf.exposure_fps[1], &outStatus)) != TOF_NO_ERROR)
        {
            RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::setExposureTime() has been terminated with error code %d.\n", ret);
        }
    }

    /* Set T3D XYZ */
    if ((ret = CTOFApiZ::getRotationAngle(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::getRotationAngle() has been terminated with error code %d.\n", ret);
    }
    if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for GetRotationAngle(0x8B) command.\n", outStatus);
    }
    if (conf.t3d_xyz[0] != outData[0] * 256 + outData[1] || conf.t3d_xyz[1] != outData[2] * 256 + outData[3] || conf.t3d_xyz[2] != outData[4] * 256 + outData[5])
    {
        if ((ret = CTOFApiZ::setRotationAngle(TOF_TIMEOUT, conf.t3d_xyz[0], conf.t3d_xyz[1], conf.t3d_xyz[2], &outStatus)) != TOF_NO_ERROR)
        {
            RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::setRotationAngle() has been terminated with error code %d.\n", ret);
        }
    }

    /* LED_FREQUENCY_ID */
    if ((ret = CTOFApiZ::getLEDfrequencyID(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::getLEDfrequencyID() has been terminated with error code %d.\n", ret);
    }
    if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for GetLEDfrequencyID(0x8D) command.\n", outStatus);
    }
    if (conf.led_frequency_id != outData[0])
    {
        if ((ret = CTOFApiZ::setLEDfrequencyID(TOF_TIMEOUT, conf.led_frequency_id, &outStatus)) != TOF_NO_ERROR)
        {
            RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::setLEDfrequencyID() has been terminated with error code %d.\n", ret);
        }
    }

    /* MIN_AMP_ALL */
    if ((ret = CTOFApiZ::getMinAmpAll(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::getMinAmpAll() has been terminated with error code %d.\n", ret);
    }
    if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for GetMinAmpAll(0x91) command.\n", outStatus);
    }
    if (conf.min_amp_all != outData[0])
    {
        if ((ret = CTOFApiZ::setMinAmpAll(TOF_TIMEOUT, conf.min_amp_all, &outStatus)) != TOF_NO_ERROR)
        {
            RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::setMinAmpAll() has been terminated with error code %d.\n", ret);
        }
    }

    /* OPERATION_CHECK_LED */
    if ((ret = CTOFApiZ::getOperationCheckLED(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::getOperationCheckLED() has been terminated with error code %d.\n", ret);
    }
    if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for GetOperationCheckLED(0x96) command.\n", outStatus);
    }
    if (conf.operation_check_led != outData[0])
    {
        if ((ret = CTOFApiZ::setOperationCheckLED(TOF_TIMEOUT, conf.operation_check_led, &outStatus)) != TOF_NO_ERROR)
        {
            RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::setOperationCheckLED() has been terminated with error code %d.\n", ret);
        }
    }

    /* RESPONSE_SPEED */
    if ((ret = CTOFApiZ::getResponseSpeed(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::getResponseSpeed() has been terminated with error code %d.\n", ret);
    }
    if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for GetResponseSpeed(0x98) command.\n", outStatus);
    }
    if (conf.response_speed[0] != outData[0] || conf.response_speed[1] * 256 + outData[2])
    {
        if ((ret = CTOFApiZ::setResponseSpeed(TOF_TIMEOUT, conf.response_speed[0], conf.response_speed[1], &outStatus)) != TOF_NO_ERROR)
        {
            RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::setResponseSpeed() has been terminated with error code %d.\n", ret);
        }
    }

    /* ENR_TRESHOLD */
    if ((ret = CTOFApiZ::getENRthreshold(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::getEnrTreshold() has been terminated with error code %d.\n", ret);
    }
    if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for GetENRthreshold(0x9A) command.\n", outStatus);
    }
    if (conf.enr_treshold != outData[0] * 256 + outData[1])
    {
        if ((ret = CTOFApiZ::setENRthreshold(TOF_TIMEOUT, conf.enr_treshold, &outStatus)) != TOF_NO_ERROR)
        {
            RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::setENRthreshold() has been terminated with error code %d.\n", ret);
        }
    }

    if (ret > 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "Can't set configuration parameters");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("b5l_tof_device"), "Configuration parameters set successfully");
    }
    return ret;
}

int b5l_tof_device::Init(ConfigParam &conf, int baudrate, INT32 &Tof_output_format)
{
    int i, ret = 0;

    if (com_init(conf.device.c_str(), baudrate) == 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "Can't open serial port %s", conf.device.c_str());
        return 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("b5l_tof_device"), "Serial port %s opened successfully", conf.device.c_str());

    /* Get Version */
    if ((ret = CTOFApiZ::getVersion(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR)
    {
        RCLCPP_INFO(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::getVersion() has been terminated with error code %d.\n", ret);
        com_close();
        return 2;
    }
    if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for GetVersion(0x00) command.\n", outStatus);
        com_close();
        return 3;
    }
    RCLCPP_INFO(rclcpp::get_logger("b5l_tof_device"), "OMRON ToF Sensor:  %.11s %d.%d.%d Revision:%d Serial:%.11s",
                outData, outData[11], outData[12], outData[13], (outData[14] << 24) + (outData[15] << 16) + (outData[16] << 8) + outData[17], outData + 18);

    /* Get Format setting */
    if ((ret = CTOFApiZ::getOutputFormat(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::getFormat() has been terminated with error code %d.", ret);
        com_close();
        return 4;
    }
    if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for GetOutputFormat(0x85) command.", outStatus);
        com_close();
        return 5;
    }
    ToF_Format = outData[0] * 256 + outData[1];

    /* Parameter Setting */
    if ((ret = SetParameter(conf) != 0))
    {
        return ret;
    }

    if (ToF_Format == 257 || ToF_Format == 258 || ToF_Format == 1 || ToF_Format == 2)
    {
        Tof_output_format = ToF_Format;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor Invalid OUTPUT_FORMAT");
        com_close();
        return 6;
    }

    /* Set Out of FOV flag when FOV_Limitation is active */
    if (FOV_Limitation)
    {
        if ((ret = CTOFApiZ::getThetaPhiTable(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR)
        {
            RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::getThetaPhiTable() has been terminated with error code %d.", ret);
            com_close();
            return 2;
        }
        if (outStatus != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for GetThetaPhiTable(0x94) command.\n", outStatus);
            com_close();
            return 3;
        }

        /* Set In-FOV flags */
        for (i = PointNum = 0; i < 76800; i++)
        {
            if (InFOV[i] = ((outData[i * 2 + 1] & 0xf0) != 0xf0))
                PointNum++;
        }
    }
    else
    {
        /* Or Initialize InFOV */
        memset(InFOV, 1, 76800 * sizeof(int));
    }

    /* Start Measurement */
    if ((ret = CTOFApiZ::startMeasurement(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::startMeasurement() has been terminated with error code %d.", ret);
        com_close();
        return 7;
    }
    if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for StartMeasurement(0x81) command.\n", outStatus);
        com_close();
        return 8;
    }

    return ret;
}

/**
 * @brief this function is for stop measurement and close serial port
 *
 * @param none
 * @return none
 */

void b5l_tof_device::Stop()
{
    INT32 ret;
    if ((ret = CTOFApiZ::stopMeasurement(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::stopMeasurement() has been terminated with error code %d.", ret);
    }
    else if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for StopMeasurement(0x00) command.\n", outStatus);
    }

    com_close();
    RCLCPP_INFO(rclcpp::get_logger("b5l_tof_device"), "Serial port closed successfully");
}

/**
 * @brief This function is get point cloud data from sensor and fill that data into input cloud
 *
 * @param[out] pointcloud pointer
 *
 */

int b5l_tof_device::Run(pcl::PointCloud<pcl::PointXYZI> *cloud_)
{
    char fname[4096];
    INT32 ret = 0, outSize, format;

    /* Get Measurement Data */
    if ((ret = CTOFApiZ::getMesurementResult(TOF_TIMEOUT, 0, &outStatus, &outSize, &outData)) != TOF_NO_ERROR)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "CTOFApiZ::getMesurementResult() has been terminated with error code %d.", ret);
        ret = 8;
        return ret;
    }
    if (outStatus != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "ToF Sensor responded with error code %d for GetMeasurementResult(0x82) command.", outStatus);
        ret = 9;
        return ret;
    }

    /* Get Format */
    format = ToF_Format;
    // RCLCPP_INFO(rclcpp::get_logger("b5l_tof_device"), "Format: %d", format);

    if (ImageOutput(format, outData, cloud_) < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("b5l_tof_device"), "File output error. [%s]\n", fname);
        ret = 11;
        return ret;
    }
    return ret;
}

int b5l_tof_device::ImageOutput(INT32 ToF_Format, unsigned char *puImage, pcl::PointCloud<pcl::PointXYZI> *cloud_)
{
    /* [CODE     - Data Type:                       Data Size (PCD) ] */
    /* 001h,002h - rectangle coordinates:           0x708AA           */
    /* 101h,102h - rectangle coordinates+amplitude: 0x960AA (0x708AA) */

    int ret = 0;

    switch (ToF_Format)
    {
    case 0x001: /* distance data (rectangle coordinates without rotation) */
    case 0x002: /* distance data (rectangle coordinates with rotation) */
        if (PCD_xyz_cloud(puImage, cloud_) < 0)
        {
            ret = -1;
        }
        puImage += 0x708AA;
        break;
    case 0x101: /* distance data (rectangle coordinates without rotation) and amplitude data */
    case 0x102: /* distance data (rectangle coordinates with rotation) and amplitude data */
        if (PCD_xyzi_cloud(puImage, cloud_) < 0)
        {
            ret = -1;
        }
        puImage += 0x708AA;
        break;
    }

    return ret;
}

int b5l_tof_device::PCD_xyz_cloud(unsigned char *puImage, pcl::PointCloud<pcl::PointXYZI> *cloud_)
{
    /* The result format should be 0x001 or 0x002 (PCD) */

    PointT currPoint;
    short *pXYZ = (short *)(puImage + 170);
    const float inv_scale = 1.0 / 1000.0;
    int i;

    /* QVGA Loop (unrolled by 4) */
    for (i = 0; i < 76800; i += 4)
    {
        /* FOV check */
        if (InFOV[i])
        {
            currPoint.x = (float)pXYZ[0] * inv_scale;
            currPoint.y = (float)pXYZ[1] * inv_scale;
            currPoint.z = (float)pXYZ[2] * inv_scale;
            cloud_->points.push_back(currPoint);
        }

        if (InFOV[i + 1])
        {
            currPoint.x = (float)pXYZ[3] * inv_scale;
            currPoint.y = (float)pXYZ[4] * inv_scale;
            currPoint.z = (float)pXYZ[5] * inv_scale;
            cloud_->points.push_back(currPoint);
        }

        if (InFOV[i + 2])
        {
            currPoint.x = (float)pXYZ[6] * inv_scale;
            currPoint.y = (float)pXYZ[7] * inv_scale;
            currPoint.z = (float)pXYZ[8] * inv_scale;
            cloud_->points.push_back(currPoint);
        }

        if (InFOV[i + 3])
        {
            currPoint.x = (float)pXYZ[9] * inv_scale;
            currPoint.y = (float)pXYZ[10] * inv_scale;
            currPoint.z = (float)pXYZ[11] * inv_scale;
            cloud_->points.push_back(currPoint);
        }

        pXYZ += 12;
    }

    return 0;
}


int b5l_tof_device::PCD_xyzi_cloud(unsigned char* puImage, pcl::PointCloud<pcl::PointXYZI>* cloud_)
{
    PointT currPoint;
    float d, m;
    short *pXYZ, *pAmplitude;
    int i, j;
    unsigned int a;

    pXYZ = (short*)(puImage + 170);
    pAmplitude = (short*)(puImage + 170 + 6 * 76800);

    const int kChunkSize = 16; // Read 8 points at a time
    const int kNumChunks = 76800 / kChunkSize;

    for (i = 0; i < kNumChunks; i++)
    {
        for (j = 0; j < kChunkSize; j++)
        {
            int idx = i * kChunkSize + j;

            if (InFOV[idx])
            {
                m = 0.0;

                d = (float)pXYZ[0] / 1000.0;
                m += d * d;
                currPoint.x = d;

                d = (float)pXYZ[1] / 1000.0;
                m += d * d;
                currPoint.y = d;

                d = (float)pXYZ[2] / 1000.0;
                m += d * d;
                currPoint.z = d;

                a = (unsigned int)pAmplitude[0];

                if (a > 255)
                {
                    if (a < TOF_OVERFLOW_AMPLITUDE)
                    {
                        a = 0;
                    }
                    else
                    {
                        a = 255;
                    }
                }
                else
                {
                    if (m * a > 255.0)
                    {
                        a = 255;
                    }
                    else
                    {
                        a = (unsigned int)(m * a);
                    }
                }

                currPoint.intensity = (unsigned char)a;

                cloud_->points.push_back(currPoint);
            }

            pXYZ += 3;
            pAmplitude++;
        }
    }

    return 0;
}