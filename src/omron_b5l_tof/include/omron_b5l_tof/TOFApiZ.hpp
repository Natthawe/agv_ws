/*---------------------------------------------------------------------------*/
/* Copyright(C)  2020-2021  OMRON Corporation                                */
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


/*----------------------------------------------------------------------------*/
/* Type Definition                                                            */
/*----------------------------------------------------------------------------*/
#ifndef UINT8
typedef     unsigned char       UINT8;      /*  8 bit Unsigned Integer  */
#endif /* UINT8 */
#ifndef INT32
typedef     int                 INT32;      /* 32 bit Signed   Integer  */
#endif /* INT32 */
#ifndef UINT32
typedef     unsigned int        UINT32;     /* 32 bit Unsigned Integer  */
#endif /* UINT32 */
#ifndef     NULL
#define     NULL						0
#endif


/* Error code */
#define	TOF_NO_ERROR					0

/* Parameter error */
#define	TOF_ERROR_PARAMETER				-1

/* Send signal timeout error */
#define	TOF_ERROR_SEND_DATA				-10

/* Receive header signal timeout error */
#define	TOF_ERROR_HEADER_TIMEOUT		-20

/* Invalid header error */
#define	TOF_ERROR_HEADER_INVALID		-21

/* Receive data signal timeout error */
#define	TOF_ERROR_DATA_TIMEOUT			-22

/* Error Pixel Values */
#define	TOF_SATULATION_DISTANCE		31000
#define	TOF_OVERFLOW_DISTANCE		32000
#define	TOF_LOWAMP_DISTANCE			30000
#define	TOF_SATULATION_AMPLITUDE	511
#define	TOF_OVERFLOW_AMPLITUDE		510
#define	TOF_LOWAMP_AMPLITUDE		256		/* Amplitude + 0x100 */


#pragma once
class CTOFApiZ
{
public:
    CTOFApiZ(void);
    ~CTOFApiZ(void);

	static INT32 receiveLength(int inTime, int inDataLen, unsigned char *outData);
	static INT32 TOF_UartSendCommand(int inCommandNo, int inDataSize, unsigned char *inData);
	static INT32 TOF_UartReceiveSync(int inTimeOutTime, int inDataSize, unsigned char *outResult);
	static INT32 TOF_SendCommand(UINT8 inCommandNo, INT32 inDataSize, UINT8 *inData);
    static INT32 TOF_ReceiveData(INT32 inTimeOutTime, UINT8 *outStatus, INT32 *outDataSize, UINT8 **outResult);

	static int getVersion(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int startMeasurement(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int stopMeasurement(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int getMesurementResult(INT32 inTimeOutTime, INT32 inMode, UINT8 *outStatus, INT32 *outSize, UINT8 **outData);
	static int setOutputFormat(INT32 inTimeOutTime, INT32 inFormat, UINT8 *outStatus);
	static int getOutputFormat(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int setOperationMode(INT32 inTimeOutTime, INT32 inFormat, UINT8 *outStatus);
	static int getOperationMode(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int setExposureTime(INT32 inTimeOutTime, INT32 inExTime, INT32 inFps, UINT8 *outStatus);
	static int getExposureTime(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int setRotationAngle(INT32 inTimeOutTime, INT32 inAngX, INT32 inAngY, INT32 inAngZ, UINT8 *outStatus);
	static int getRotationAngle(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int setLEDfrequencyID(INT32 inTimeOutTime, INT32 inID, UINT8 *outStatus);
	static int getLEDfrequencyID(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int setMinAmpAll(INT32 inTimeOutTime, INT32 inMinAmp, UINT8 *outStatus);
	static int getMinAmpAll(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int setMinAmpNear(INT32 inTimeOutTime, INT32 inMinAmp, UINT8 *outStatus);
	static int getMinAmpNear(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int getThetaPhiTable(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int setOperationCheckLED(INT32 inTimeOutTime, INT32 inLED, UINT8 *outStatus);
	static int getOperationCheckLED(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int setResponseSpeed(INT32 inTimeOutTime, INT32 inTransmissionSize, INT32 inTransmissionInterval, UINT8 *outStatus);
	static int getResponseSpeed(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int setENRthreshold(INT32 inTimeOutTime, INT32 inThreshold, UINT8 *outStatus);
	static int getENRthreshold(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int getImagerTemparature(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int getLEDTemparature(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int initializeParameters(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
	static int softwareReset(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData);
};

