/*---------------------------------------------------------------------------*/
/* Copyright(C)  2019-2021  OMRON Corporation                                */
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

#include "ToF_Sample.hpp"
using PointT = pcl::PointXYZI;

/* [CODE:  Data Type:                                                              ] */
/* 0x001:  distance data (rectangle coordinates without rotation)                    */
/* 0x002:  distance data (rectangle coordinates with rotation)                       */
/* 0x101:  distance data (rectangle coordinates without rotation) and amplitude data */
/* 0x102:  distance data (rectangle coordinates with rotation) and amplitude data    */

#define TOF_TIMEOUT 500 /* ms (Minimum: 500ms) */

const char	*ucParameters[] = {
	"IMAGE_PATH",
	"FOV_LIMITATION",
	"OUTPUT_FORMAT",
	"OPERATION_MODE",
	"EXPOSURE_FPS",
	"T3D_XYZ",
	"LED_FREQUENCY_ID",
	"MIN_AMP_ALL",
	"MIN_AMP_NEAR",
	"OPERATION_CHECK_LED",
	"RESPONSE_SPEED",
	"ENR_TRESHOLD"};

/* Tof Output Format */
INT32		ToF_Format;

/* Flag for deleting the pixels out-of-FOV */
int			FOV_Limitation = 0,
			PointNum = 76800,
			InFOV[76800];

char		ImagePath[256] = "";
UINT8		outStatus,
			*outData;


/*****************************************************************************
 * Function Name : PCD_xyz_cloud
 *****************************************************************************/
/**
 * @brief	This function is for fill PointXYZ data in point cloud
 *
 * @param[in/out] Image char string which has point cloud data /  point cloud pointer
 *
 * @return	On Succefull it returns 0 else returns error code
 */
int CTOFSample::PCD_xyz_cloud(unsigned char *puImage, pcl::PointCloud<pcl::PointXYZI> *cloud_)
{
	/* The result format should be 0x001 or 0x002 (PCD) */

	PointT	currPoint;
	float	d;
	short	*pXYZ;
	int		i;

	pXYZ = (short *)(puImage + 170);

	/* QVGA Loop */
	for (i = 0; i < 76800; i++) {

		/* FOV check */
		if (InFOV[i]) {
			d = (float)*pXYZ++ / 1000.0;

			currPoint.x = d;

			d = (float)*pXYZ++ / 1000.0;

			currPoint.y = d;

			d = (float)*pXYZ++ / 1000.0;

			currPoint.z = d;

			cloud_->points.push_back(currPoint);
		}
	}

	return 0;
}


/*****************************************************************************
 * Function Name : PCD_xyzi_cloud
 *****************************************************************************/
/**
 * @brief	This function is for fill PointXYZI data in point cloud
 *
 * @param[in/out] Image char string which has point cloud data /  point cloud pointer
 *
 * @return	On Succefull it returns 0 else returns error code
 */
int CTOFSample::PCD_xyzi_cloud(unsigned char *puImage, pcl::PointCloud<pcl::PointXYZI> *cloud_)
{
	/* The result format should be 0x101 or 0x102 (PCD + Amplitude) */

	PointT			currPoint;
	float			d,
					m;
	short			*pXYZ,
					*pAmplitude;
	int				i;
	unsigned int	a;

	pXYZ = (short *)(puImage + 170);
	pAmplitude = (short *)(puImage + 170 + 6 * 76800);

	/* QVGA Loop */
	for (i = 0; i < 76800; i++) {

		/* FOV check */
		if (InFOV[i]) {

			/* Output XYZ */
			m = 0.0;

			d = (float)*pXYZ++ / 1000.0;
			m += d * d;
			currPoint.x = d;

			d = (float)*pXYZ++ / 1000.0;
			m += d * d;
			currPoint.y = d;

			d = (float)*pXYZ++ / 1000.0;
			m += d * d;
			currPoint.z = d;

			/* Prepare Amplitude data */
			a = (unsigned int)*pAmplitude++;

			/* Error check */
			if (a > 255) {
				if (a < TOF_OVERFLOW_AMPLITUDE) {
					// Low Amplitude
					a = 0;
				}
				else {
					// Satulation or Overflow
					a = 255;
				}
			}
			else {
				if (m * a > 255.0) {
					a = 255;
				}
				else {
					a = (unsigned int)(m * a);
				}
			}

			currPoint.intensity = (unsigned char)a;

			cloud_->points.push_back(currPoint);
		}
	}

	return 0;
}


/*****************************************************************************
 * Function Name : ImageOutput
 *****************************************************************************/
/**
 * @brief	This function is for fill PointXYZ or PointXYZI data in point cloud according to output format
 *
 * @param[in/out] Tof format integer, Image char string which has point cloud data /  point cloud pointer
 *
 * @return	On Succefull it returns 0 else returns error code
 */
int CTOFSample::ImageOutput(INT32 ToF_Format, unsigned char *puImage, pcl::PointCloud<pcl::PointXYZI> *cloud_)
{
	/* [CODE     - Data Type:                       Data Size (PCD) ] */
	/* 001h,002h - rectangle coordinates:           0x708AA           */
	/* 101h,102h - rectangle coordinates+amplitude: 0x960AA (0x708AA) */

	int	ret = 0;

	switch (ToF_Format)
	{
	case 0x001: /* distance data (rectangle coordinates without rotation) */
	case 0x002: /* distance data (rectangle coordinates with rotation) */
		if (PCD_xyz_cloud(puImage, cloud_) < 0) {
			ret = -1;
		}
		puImage += 0x708AA;
		break;
	case 0x101: /* distance data (rectangle coordinates without rotation) and amplitude data */
	case 0x102: /* distance data (rectangle coordinates with rotation) and amplitude data */
		if (PCD_xyzi_cloud(puImage, cloud_) < 0) {
			ret = -1;
		}
		puImage += 0x708AA;
		break;
	}
	
	return ret;
}


/*****************************************************************************
 * Function Name : SetParameter
 *****************************************************************************/
/**
 * @brief	This function is for set parameter from config file
 *
 * @param[in] config file pointer
 *
 * @return	On Succefull it returns 0 else returns error code
 */
int CTOFSample::SetParameter(const char *pcFileName)
{
	FILE	*fp;
	int		c,
			x,
			y,
			z,
			ret;
	char	buff[256],
			cmd[256],
			value[256],
			*p;

	/* Open Parameter File */
	if ((fp = fopen(pcFileName, "rt")) == NULL) {
		fprintf(stderr, "Cannot read the Parameter File: [%s]\n", pcFileName);
		return -1;
	}

	/* Parse Parameter File */
	while (fgets(buff, 256, fp) != NULL) {

		/* Determine EOL while converting lowercase to uppercase */
		for (p = buff; *p != '\0' && *p != '#' && *p != ';'; p++);
		*p = '\0';

		/* Skip leading whitespace */
		for (p = buff; *p == ' ' || *p == '\t'; p++);

		/* Parse Key and Value */
		c = sscanf(p, "%[^= \t]=%s", cmd, value);
		if (c == EOF || c < 2) {
			continue;
		}

		/* Find Key */
		for (p = cmd; *p != '\0'; p++) {
			*p = toupper(*p);
		}
		for (c = 0; c < 11; c++) {
			if (strcmp(ucParameters[c], cmd) == 0)
				break;
		}
		ret = 0;
		switch (c) {
			case 0: /* IMAGE_PATH */
				strcpy(ImagePath, value);
				break;
			case 1: /* FOV_LIMITATION */
				if (sscanf(value, "%i", &x) != 1) {
					ret = 1;
					break;
				}
				if (x != 0 && x != 1) {
					ret = 1;
					break;
				}
				FOV_Limitation = x;
				break;
			case 2: /* OUTPUT_FORMAT */
				if (sscanf(value, "%i", &x) != 1) {
					ret = 1;
					break;
				}
				if (x != ToF_Format) {
					if ((ret = CTOFApiZ::setOutputFormat(TOF_TIMEOUT, x, &outStatus)) != TOF_NO_ERROR) {
						fprintf(stderr, "CTOFApiZ::setOutputFormat() has been terminated with error code %d.\n", ret);
					}
					ToF_Format = x;
				}
				break;
			case 3: /* OPERATION_MODE */
				if (sscanf(value, "%i", &x) != 1) {
					ret = 1;
					break;
				}
				if ((ret = CTOFApiZ::getOperationMode(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
					fprintf(stderr, "CTOFApiZ::getOperationMode() has been terminated with error code %d.\n", ret);
					break;
				}
				if (outStatus != 0) {
					fprintf(stderr, "ToF Sensor responded with error code %d for GetOperationMode(0x87) command.\n", outStatus);
					break;
				}
				if (x != outData[0]) {
					if ((ret = CTOFApiZ::setOperationMode(TOF_TIMEOUT, x, &outStatus)) != TOF_NO_ERROR) {
						fprintf(stderr, "CTOFApiZ::setOperationMode() has been terminated with error code %d.\n", ret);
					}
				}
				break;
			case 4: /* EXPOSURE_FPS */
				if (sscanf(value, "%i,%i", &x, &y) != 2) {
					ret = 1;
					break;
				}
				if ((ret = CTOFApiZ::getExposureTime(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
					fprintf(stderr, "CTOFApiZ::getExposureTime() has been terminated with error code %d.\n", ret);
					break;
				}
				if (outStatus != 0) {
					fprintf(stderr, "ToF Sensor responded with error code %d for GetExposureTime(0x89) command.\n", outStatus);
					break;
				}
				if (x != outData[0] * 256 + outData[1] || y != outData[6]) {
					if ((ret = CTOFApiZ::setExposureTime(TOF_TIMEOUT, x, y, &outStatus)) != TOF_NO_ERROR) {
						fprintf(stderr, "CTOFApiZ::setExposureTime() has been terminated with error code %d.\n", ret);
					}
				}
				break;
			case 5: /* T3D_XYZ */
				if (sscanf(value, "%i,%i,%i", &x, &y, &z) != 3) {
					ret = 1;
					break;
				}
				if ((ret = CTOFApiZ::getRotationAngle(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
					fprintf(stderr, "CTOFApiZ::getRotationAngle() has been terminated with error code %d.\n", ret);
					break;
				}
				if (outStatus != 0) {
					fprintf(stderr, "ToF Sensor responded with error code %d for GetRotationAngle(0x8B) command.\n", outStatus);
					break;
				}
				if (x != outData[0] * 256 + outData[1] || y != outData[2] * 256 + outData[3] || z != outData[4] * 256 + outData[5]) {
					if ((ret = CTOFApiZ::setRotationAngle(TOF_TIMEOUT, x, y, z, &outStatus)) != TOF_NO_ERROR) {
						fprintf(stderr, "CTOFApiZsetRotationAngle() has been terminated with error code %d.\n", ret);
					}
				}
				break;
			case 6: /* LED_FREQUENCY_ID */
				if (sscanf(value, "%i", &x) != 1) {
					ret = 1;
					break;
				}
				if ((ret = CTOFApiZ::getLEDfrequencyID(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
					fprintf(stderr, "CTOFApiZ::getLEDfrequencyID() has been terminated with error code %d.\n", ret);
					break;
				}
				if (outStatus != 0) {
					fprintf(stderr, "ToF Sensor responded with error code %d for GetLEDfrequencyID(0x8F) command.\n", outStatus);
					break;
				}
				if (x != outData[0]) {
					if ((ret = CTOFApiZ::setLEDfrequencyID(TOF_TIMEOUT, x, &outStatus)) != TOF_NO_ERROR) {
						fprintf(stderr, "CTOFApiZ::setLEDfrequencyID() has been terminated with error code %d.\n", ret);
					}
				}
				break;
			case 7: /* MIN_AMP_ALL */
				if (sscanf(value, "%i", &x) != 1) {
					ret = 1;
					break;
				}
				if ((ret = CTOFApiZ::getMinAmpAll(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
					fprintf(stderr, "CTOFApiZ::getMinAmpAll() has been terminated with error code %d.\n", ret);
					break;
				}
				if (outStatus != 0) {
					fprintf(stderr, "ToF Sensor responded with error code %d for GetMinAmpAll(0x91) command.\n", outStatus);
					break;
				}
				if (x != outData[0]) {
					if ((ret = CTOFApiZ::setMinAmpAll(TOF_TIMEOUT, x, &outStatus)) != TOF_NO_ERROR) {
						fprintf(stderr, "CTOFApiZ::setMinAmpAll() has been terminated with error code %d.\n", ret);
					}
				}
				break;
			case 8: /* MIN_AMP_NEAR */
				if (sscanf(value, "%i", &x) != 1) {
					ret = 1;
					break;
				}
				if ((ret = CTOFApiZ::getMinAmpNear(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
					fprintf(stderr, "CTOFApiZ::getMinAmpNear() has been terminated with error code %d.\n", ret);
					break;
				}
				if (outStatus != 0) {
					fprintf(stderr, "ToF Sensor responded with error code %d for GetMinAmpNear(0x93) command.\n", outStatus);
					break;
				}
				if (x != outData[0]) {
					if ((ret = CTOFApiZ::setMinAmpNear(TOF_TIMEOUT, x, &outStatus)) != TOF_NO_ERROR) {
						fprintf(stderr, "CTOFApiZ::setMinAmpNear() has been terminated with error code %d.\n", ret);
					}
				}
				break;
			case 9: /* OPERATION_CHECK_LED */
				if (sscanf(value, "%i", &x) != 1) {
					ret = 1;
					break;
				}
				if ((ret = CTOFApiZ::getOperationCheckLED(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
					fprintf(stderr, "CTOFApiZ::getOperationCheckLED() has been terminated with error code %d.\n", ret);
					break;
				}
				if (outStatus != 0) {
					fprintf(stderr, "ToF Sensor responded with error code %d for GetOperationCheckLED(0x96) command.\n", outStatus);
					break;
				}
				if (x != outData[0]) {
					if ((ret = CTOFApiZ::setOperationCheckLED(TOF_TIMEOUT, x, &outStatus)) != TOF_NO_ERROR) {
						fprintf(stderr, "CTOFApiZ::setOperationCheckLED() has been terminated with error code %d.\n", ret);
					}
				}
				break;
			case 10: /* RESPONSE_SPEED */
				if (sscanf(value, "%i,%i", &x, &y) != 2) {
					ret = 1;
					break;
				}
				if ((ret = CTOFApiZ::getResponseSpeed(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
					fprintf(stderr, "CTOFApiZ::getResponseSpeed() has been terminated with error code %d.\n", ret);
					break;
				}
				if (outStatus != 0) {
					fprintf(stderr, "ToF Sensor responded with error code %d for GetResponseSpeed(0x98) command.\n", outStatus);
					break;
				}
				if (x != outData[0] || y != outData[1] * 256 + outData[2]) {
					if ((ret = CTOFApiZ::setResponseSpeed(TOF_TIMEOUT, x, y, &outStatus)) != TOF_NO_ERROR) {
						fprintf(stderr, "CTOFApiZ::setResponseSpeed() has been terminated with error code %d.\n", ret);
					}
				}
				break;
			case 11: /* ENR_TRESHOLD */
				if (sscanf(value, "%i", &x) != 1) {
					ret = 1;
					break;
				}
				if ((ret = CTOFApiZ::getENRthreshold(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
					fprintf(stderr, "CTOFApiZ::getENRthreshold() has been terminated with error code %d.\n", ret);
					break;
				}
				if (outStatus != 0) {
					fprintf(stderr, "ToF Sensor responded with error code %d for GetENRthreshold(0x9A) command.\n", outStatus);
					break;
				}
				if (x != outData[0] * 256 + outData[1]) {
					if ((ret = CTOFApiZ::setENRthreshold(TOF_TIMEOUT, x, &outStatus)) != TOF_NO_ERROR) {
						fprintf(stderr, "CTOFApiZ::setENRthreshold() has been terminated with error code %d.\n", ret);
					}
				}
				break;
			default: /* ELSE */
				ret = 1;
		}
	}
	if (ret > 0) {
		fprintf(stderr, "Parameter File Read Error: %s\n", buff);
	}
	fclose(fp);
	return ret;
}


/*****************************************************************************
 * Function Name : Init
 *****************************************************************************/
/**
 * @brief	This function is for initialize application
 *
 * @param[in/out] config file string / output format integer
 *
 * @return	On Succefull it returns 0 else returns error code
 */
int CTOFSample::Init(std::string config_file, INT32 & Tof_output_format)
{
	S_STAT	stat;
	INT32	i,
			ret = 0;

	/* Open Serial Port */
	stat.com_num = 0;
	stat.BaudRate = 921600;
	if (com_init(&stat) == 0) {
		fprintf(stderr, "Can not open Device\n");
		return 1;
	}
	fprintf(stderr, "/dev/ttyUSB%d is opened successfully.\n", stat.com_num);

	/* Get Version */
	if ((ret = CTOFApiZ::getVersion(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
		fprintf(stderr, "CTOFApiZ::getVersion() has been terminated with error code %d.\n", ret);
		com_close();
		return 2;
	}
	if (outStatus != 0) {
		fprintf(stderr, "ToF Sensor responded with error code %d for GetVersion(0x00) command.\n", outStatus);
		com_close();
		return 3;
	}
	fprintf(stderr, "OMRON ToF Sensor:  %.11s %d.%d.%d Revision:%d Serial:%.11s\n",
			outData, outData[11], outData[12], outData[13], (outData[14] << 24) + (outData[15] << 16) + (outData[16] << 8) + outData[17], outData + 18);

	/* Get Format setting */
	if ((ret = CTOFApiZ::getOutputFormat(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
		fprintf(stderr, "CTOFApiZ::getOutputFormat() has been terminated with error code %d.\n", ret);
		com_close();
		return 4;
	}
	if (outStatus != 0) {
		fprintf(stderr, "ToF Sensor responded with error code %d for GetOutputFormat(0x85) command.\n", outStatus);
		com_close();
		return 5;
	}
	ToF_Format = outData[0] * 256 + outData[1];
		 
	/* Parameter Setting */
	if ((ret = SetParameter(config_file.c_str()) != 0)) {
		return ret;
	}

	if (ToF_Format == 257 || ToF_Format == 258 || ToF_Format == 1 || ToF_Format == 2) {
		Tof_output_format = ToF_Format;
	}
	else {
		fprintf(stderr, "Invalid OUTPUT_FORMAT in config file\n");
		com_close();
		return 6;
	}

	/* Set Out of FOV flag when FOV_Limitation is active */
	if (FOV_Limitation) {
		if ((ret = CTOFApiZ::getThetaPhiTable(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
			fprintf(stderr, "CTOFApiZ::getThetaPhiTable() has been terminated with error code %d.\n", ret);
			com_close();
			return 2;
		}
		if (outStatus != 0) {
			fprintf(stderr, "ToF Sensor responded with error code %d for GetThetaPhiTable(0x94) command.\n", outStatus);
			com_close();
			return 3;
		}

		/* Set In-FOV flags */
		for (i = PointNum = 0; i < 76800; i++) {
			if (InFOV[i] = ((outData[i * 2 + 1] & 0xf0) != 0xf0)) PointNum++;
		}
	}

	/* Or Initialize InFOV */
	else {
		for (i = 0; i < 76800; InFOV[i++] = 1);
	}

	/* Start Measurement */
	if ((ret = CTOFApiZ::startMeasurement(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
		fprintf(stderr, "CTOFApiZ::startMeasurement() has been terminated with error code %d.\n", ret);
		com_close();
		return 7;
	}
	if (outStatus != 0) {
		fprintf(stderr, "ToF Sensor responded with error code %d for StartMeasurement(0x80) command.\n", outStatus);
		com_close();
		return 8;
	}

	return ret;
}


/*****************************************************************************
 * Function Name : Stop
 *****************************************************************************/
/**
 * @brief	This function is for stop measurement and close com port
 *
 * @param[in/out] None
 *
 * @return	None
 */
void CTOFSample::Stop()
{
	INT32	ret;

	/* Termination */
	if ((ret = CTOFApiZ::stopMeasurement(TOF_TIMEOUT, &outStatus, &outData)) != TOF_NO_ERROR) {
		fprintf(stderr, "CTOFApiZ::stopMeasurement() has been terminated with error code %d.\n", ret);
	}
	else if (outStatus != 0) {
		fprintf(stderr, "ToF Sensor responded with error code %d for StopMeasurement(0x00) command.\n", outStatus);
	}

	/* Close Com. Port */
	com_close();
	printf("\rTermination Complete!\n");
}


/*****************************************************************************
 * Function Name : Run
 *****************************************************************************/
/**
 * @brief	This function is get point cloud data from sensor and fill that data into input cloud
 *
 * @param[out] point cloud pointer
 *
 * @return	On Succefull it returns 0 else returns error code
 */
int CTOFSample::Run(pcl::PointCloud<pcl::PointXYZI> *cloud_)
{
	char	fname[1024];
	INT32	ret = 0,
			outSize,
			format;

	/* Get Measurement Result */
	if ((ret = CTOFApiZ::getMesurementResult(TOF_TIMEOUT, 0, &outStatus, &outSize, &outData)) != TOF_NO_ERROR) {
		fprintf(stderr, "CTOFApiZ::getMesurementResult() has been terminated with error code %d.\n", ret);
		ret = 8;
		return ret;
	}
	if (outStatus != 0) {
		fprintf(stderr, "ToF Sensor responded with error code %d for GetMesurementResult(0x82) command.\n", outStatus);
		ret = 9;
		return ret;
	}

	/* Output Data File */
	format = ToF_Format;

	if (ImageOutput(format, outData, cloud_) < 0) {
		fprintf(stderr, "File output error. [%s]\n", fname);
		ret = 11;
		return ret;
	}

	return ret;
}
