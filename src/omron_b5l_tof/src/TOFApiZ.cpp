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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "TOFApiZ.hpp"
#include "uart.h"

#include <time.h>
#include <sys/time.h>

extern void com_clear();


/*----------------------------------------------------------------------------*/
/* Command number                                                             */
/*----------------------------------------------------------------------------*/

#define	GET_VERSION							0x00
#define	START_MEASUREMENT					0x80
#define	STOP_MEASUREMENT					0x81
#define	GET_MEASUREMENT_RESULT				0x82
#define	SET_OUTPUT_FORMAT					0x84
#define	GET_OUTPUT_FORMAT					0x85
#define	SET_OPERATION_MODE					0x86
#define	GET_OPERATION_MODE					0x87
#define	SET_EXPOSURE_TIME					0x88
#define	GET_EXPOSURE_TIME					0x89
#define	SET_ROTATION_ANGLE					0x8A
#define	GET_ROTATION_ANGLE					0x8B
#define	SET_LED_FREQUENCY_ID				0x8E
#define	GET_LED_FREQUENCY_ID				0x8F
#define	SET_MIN_AMP_ALL						0x90
#define	GET_MIN_AMP_ALL						0x91
#define	SET_MIN_AMP_NEAR					0x92
#define	GET_MIN_AMP_NEAR					0x93
#define	GET_THETA_PHI_TABLE					0x94
#define	SET_OPERATION_CHECK_LED				0x95
#define	GET_OPERATION_CHECK_LED				0x96
#define	SET_RESPONSE_SPEED					0x97
#define	GET_RESPONSE_SPEED					0x98
#define	SET_ENR_THRESHOLD					0x99
#define	GET_ENR_THRESHOLD					0x9A
#define	GET_IMAGER_TEMPERATURE				0x9B
#define	GET_LED_TEMPERATURE					0x9C
#define	INITIALIZE_PARAMETERS				0x9E
#define	SOFTWARE_RESET						0x9F

#define ERROR_WAIT							500		// ms


#define	TOF_SYNC_BYTE_LEN		1
#define	TOF_STATUS_LEN			1
#define	TOF_RCV_DATASIZE_LEN	4
#define	TOF_RCV_HEADER_LEN		(TOF_SYNC_BYTE_LEN + TOF_STATUS_LEN + TOF_RCV_DATASIZE_LEN)


/*----------------------------------------------------------------------------*/
/* Header for send signal data */
typedef enum {
    SEND_HEAD_SYNCBYTE = 0,
    SEND_HEAD_COMMANDNO,
    SEND_HEAD_DATALENGTHLSB,
    SEND_HEAD_DATALENGTHMSB,
    SEND_HEAD_NUM
}SEND_HEADER;
/*----------------------------------------------------------------------------*/
/* Header for receive signal data */
typedef enum {
    RECEIVE_HEAD_SYNCBYTE = 0,
    RECEIVE_HEAD_STATUS,
    RECEIVE_HEAD_DATALENMM,
    RECEIVE_HEAD_DATALENML,
    RECEIVE_HEAD_DATALENLM,
    RECEIVE_HEAD_DATALENLL,
    RECEIVE_HEAD_NUM
}RECEIVE_HEADER;

static UINT8 acReceiveBuffer[4*1024*1024];          /* Store received data */



CTOFApiZ::CTOFApiZ(void)
{
}


CTOFApiZ::~CTOFApiZ(void)
{
}


INT32 CTOFApiZ::receiveLength(int inTime, int inDataLen, unsigned char *outData)
{

	int nTotal = 0;
	double finishTime = 0.0;
	struct timeval sStartTime, sEndTime;

	gettimeofday(&sStartTime, NULL);
	do {
		int len = com_recv(10, &outData[nTotal], inDataLen - nTotal);

		if (len <= 0) {
			gettimeofday(&sEndTime, NULL);
			finishTime = difftime(sEndTime.tv_sec, sStartTime.tv_sec) * 1000.0 + (sEndTime.tv_usec - sStartTime.tv_usec) / 1000.0;
			if (finishTime >= (double)inTime)
				return -nTotal;
			continue;
		}
		nTotal += len;
		if (nTotal >= inDataLen) break;
	} while (1);

	return nTotal;
}


INT32 CTOFApiZ::TOF_UartSendCommand(int inCommandNo, int inDataSize, unsigned char *inData)
{
	int ret = 0;
	int nSize = 0;
	unsigned char *pSendData = NULL;

	pSendData = (UINT8*)malloc(4 + inDataSize);
	if (pSendData == NULL) return -1;

	com_clear();


	if (inCommandNo >= 0) {
		/* Create header */
		pSendData[0] = (UINT8)0xFE;
		pSendData[1] = inCommandNo;
		pSendData[2] = (UINT8)(((UINT32)inDataSize >> 8) & 0xff);
		pSendData[3] = (UINT8)(((UINT32)inDataSize) & 0xff);
		nSize += 4;
	}

	if ((inData != NULL) && (inDataSize > 0)) {
		memcpy(&pSendData[nSize], inData, inDataSize);
		nSize += inDataSize;
	}

	ret = com_write(pSendData, nSize);

	free(pSendData);

	if (ret != nSize) {
		return -1;
	}
	return ret;
}


INT32 CTOFApiZ::TOF_UartReceiveSync(int inTimeOutTime, int inDataSize, unsigned char *outResult)
{
	int len;
	int rxlen = 0;
	int rxTotal = 0;
	int rxReal = 0;
	unsigned char rxbuf[1024];

	double finishTime = 0.0;
	struct timeval sStartTime, sEndTime;

	gettimeofday(&sStartTime, NULL);

	do {
		gettimeofday(&sEndTime, NULL);
		finishTime = difftime(sEndTime.tv_sec, sStartTime.tv_sec) * 1000.0 + (sEndTime.tv_usec - sStartTime.tv_usec) / 1000.0;
		if (finishTime >= (double)inTimeOutTime)
			return -rxReal;

		len = com_length();
		if (len <= 0) continue;

		len = com_recv(10, &rxbuf[rxlen], 1);
		if (len <= 0) continue;

		rxReal += len;
		rxlen += len;
		rxbuf[rxlen] = 0x00;

		if (rxbuf[0] != 0xFE) {
			rxlen = 0;
			rxbuf[rxlen] = 0;
			continue;
		}

		gettimeofday(&sStartTime, NULL);
		if (rxlen >= TOF_RCV_HEADER_LEN) {
			int nDataLen = (rxbuf[TOF_SYNC_BYTE_LEN + TOF_STATUS_LEN] << 24) + (rxbuf[TOF_SYNC_BYTE_LEN + TOF_STATUS_LEN + 1] << 16) + (rxbuf[TOF_SYNC_BYTE_LEN + TOF_STATUS_LEN + 2] << 8) + (rxbuf[TOF_SYNC_BYTE_LEN + TOF_STATUS_LEN + 3]);

			if (nDataLen < 0 || (nDataLen + TOF_RCV_HEADER_LEN) > inDataSize) {
				return -1;
			}
			if (rxTotal == 0) {
				if (rxlen > nDataLen + TOF_RCV_HEADER_LEN) {
					rxlen = nDataLen + TOF_RCV_HEADER_LEN;
				}
				memcpy(&outResult[0], &rxbuf[0], rxlen);
				rxTotal += rxlen;

				do {
					if ((nDataLen + TOF_RCV_HEADER_LEN) <= rxTotal) {
						break;
					}
					len = receiveLength(inTimeOutTime, nDataLen - (rxTotal - TOF_RCV_HEADER_LEN), &outResult[rxTotal]);
					if (len <= 0) return -rxReal;
					rxReal += len;
					rxTotal += len;
				} while (1);
			}
			else {
				if (rxTotal + (rxlen - TOF_RCV_HEADER_LEN) > (nDataLen + TOF_RCV_HEADER_LEN)) {
					rxlen = (nDataLen + TOF_RCV_HEADER_LEN) - rxTotal + TOF_RCV_HEADER_LEN;
				}
				memcpy(&outResult[rxTotal], &rxbuf[TOF_RCV_HEADER_LEN], rxlen - TOF_RCV_HEADER_LEN);
				rxTotal += (rxlen - TOF_RCV_HEADER_LEN);
			}
			rxlen = TOF_RCV_HEADER_LEN;
			if ((nDataLen + TOF_RCV_HEADER_LEN) <= rxTotal) {
				break;
			}
		}
	} while (1);

	return rxTotal;
}



/*----------------------------------------------------------------------------*/
/* Send command signal                                                        */
/* param    : UINT8         inCommandNo     command number                    */
/*          : INT32         inDataSize      sending signal data size          */
/*          : UINT8         *inData         sending signal data               */
/* return   : INT32                         execution result error code       */
/*          :                               0...normal                        */
/*          :                               -10...timeout error               */
/*----------------------------------------------------------------------------*/
INT32 CTOFApiZ::TOF_SendCommand(UINT8 inCommandNo, INT32 inDataSize, UINT8 *inData)
{
    INT32 ret = 0;

	com_clear();

	/* Send command signal */
    ret = TOF_UartSendCommand(inCommandNo, inDataSize, inData);

    if(ret != SEND_HEAD_NUM+inDataSize){
        return TOF_ERROR_SEND_DATA;
    }
    return 0;
}

/*----------------------------------------------------------------------------*/
/* Receive data                                                               */
/* param    : INT32         inTimeOutTime   timeout time                      */
/*          : UINT8         *outStatus      status                            */
/*          : INT32         *outDataSize    receive signal data length        */
/*          : UINT8         *outResult      receive signal data               */
/* return   : INT32                         execution result error code       */
/*          :                               0...normal                        */
/*          :                               -20...timeout error               */
/*          :                               -21...invalid header error        */
/*----------------------------------------------------------------------------*/
INT32 CTOFApiZ::TOF_ReceiveData(INT32 inTimeOutTime, UINT8 *outStatus, INT32 *outDataSize, UINT8 **outResult)
{
    INT32 ret      = 0;
    INT32 data_len = 0;

    /* Get header part */
    data_len = TOF_UartReceiveSync(inTimeOutTime, sizeof(acReceiveBuffer), acReceiveBuffer);

    if(data_len < RECEIVE_HEAD_NUM){
        ret = TOF_ERROR_HEADER_TIMEOUT;
    }
    else if((UINT8)0xFE != acReceiveBuffer[RECEIVE_HEAD_SYNCBYTE]){
        /* Different value indicates an invalid result */
        ret = TOF_ERROR_HEADER_INVALID;
    }
	else{
		/* Get command execution result */
		*outStatus  = acReceiveBuffer[RECEIVE_HEAD_STATUS];

		/* Get data length */
		*outDataSize =	(acReceiveBuffer[RECEIVE_HEAD_DATALENLL]    ) +
						(acReceiveBuffer[RECEIVE_HEAD_DATALENLM]<< 8) +
						(acReceiveBuffer[RECEIVE_HEAD_DATALENML]<<16) +
						(acReceiveBuffer[RECEIVE_HEAD_DATALENMM]<<24);

		if ( outResult != NULL ) {
			*outResult = &acReceiveBuffer[RECEIVE_HEAD_NUM];
		}

		if(data_len != RECEIVE_HEAD_NUM + *outDataSize){
			ret = TOF_ERROR_DATA_TIMEOUT;
		}
	}
	if( ret != 0 ){
		usleep(ERROR_WAIT * 1000);
	}
    return ret;
}

/********************************************************************/
/* Get Version                                                      */
/********************************************************************/
int CTOFApiZ::getVersion(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(GET_VERSION, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Start Measurement                                                */
/********************************************************************/
int CTOFApiZ::startMeasurement(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(START_MEASUREMENT, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Stop Measurement                                                 */
/********************************************************************/
int CTOFApiZ::stopMeasurement(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(STOP_MEASUREMENT, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Get Mesurement Result                                            */
/********************************************************************/
int CTOFApiZ::getMesurementResult(INT32 inTimeOutTime, INT32 inMode, UINT8 *outStatus, INT32 *outSize, UINT8 **outData)
{
    INT32 ret	= 0;
    UINT8 sendData[16];

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	sendData[0] = inMode & 0xFF;

    ret = TOF_SendCommand(GET_MEASUREMENT_RESULT, 1, sendData);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, outSize, outData);

    return ret;
}

/********************************************************************/
/* Set Output Format                                                */
/********************************************************************/
int CTOFApiZ::setOutputFormat(INT32 inTimeOutTime, INT32 inFormat, UINT8 *outStatus)
{
    INT32 ret	= 0;
    INT32 size	= 0;
    UINT8 sendData[16];

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	sendData[0] = (inFormat >> 8) & 0xFF;
	sendData[1] = inFormat & 0xFF;

    ret = TOF_SendCommand(SET_OUTPUT_FORMAT, 2, sendData);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, NULL);

    return ret;
}

/********************************************************************/
/* Get Output Format                                                */
/********************************************************************/
int CTOFApiZ::getOutputFormat(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(GET_OUTPUT_FORMAT, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Set Operation Mode                                               */
/********************************************************************/
int CTOFApiZ::setOperationMode(INT32 inTimeOutTime, INT32 inMode, UINT8 *outStatus)
{
    INT32 ret	= 0;
    INT32 size	= 0;
    UINT8 sendData[16];

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	sendData[0] = inMode & 0xFF;

    ret = TOF_SendCommand(SET_OPERATION_MODE, 1, sendData);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, NULL);

    return ret;
}

/********************************************************************/
/* Get Operation Mode                                               */
/********************************************************************/
int CTOFApiZ::getOperationMode(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	ret = TOF_SendCommand(GET_OPERATION_MODE, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Set Exposure Time                                                */
/********************************************************************/
int CTOFApiZ::setExposureTime(INT32 inTimeOutTime, INT32 inExTime, INT32 inFps, UINT8 *outStatus)
{
    INT32 ret	= 0;
    INT32 size	= 0;
    UINT8 sendData[16];

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	sendData[0] = (inExTime >> 8) & 0xFF;
	sendData[1] = inExTime & 0xFF;
	sendData[2] = 0x00;
	sendData[3] = 0x00;
	sendData[4] = 0x00;
	sendData[5] = 0x00;
	sendData[6] = inFps & 0xFF;

    ret = TOF_SendCommand(SET_EXPOSURE_TIME, 7, sendData);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, NULL);

    return ret;
}

/********************************************************************/
/* Get Exposure Time                                                */
/********************************************************************/
int CTOFApiZ::getExposureTime(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(GET_EXPOSURE_TIME, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Set Rotation Angle                                               */
/********************************************************************/
int CTOFApiZ::setRotationAngle(INT32 inTimeOutTime, INT32 inAngX, INT32 inAngY, INT32 inAngZ, UINT8 *outStatus)
{
    INT32 ret	= 0;
    INT32 size	= 0;
    UINT8 sendData[16];

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	sendData[0] = (inAngX >> 8) & 0xFF;
	sendData[1] = inAngX & 0xFF;
	sendData[2] = (inAngY >> 8) & 0xFF;
	sendData[3] = inAngY & 0xFF;
	sendData[4] = (inAngZ >> 8) & 0xFF;
	sendData[5] = inAngZ & 0xFF;

    ret = TOF_SendCommand(SET_ROTATION_ANGLE, 6, sendData);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, NULL);

    return ret;
}

/********************************************************************/
/* Get Rotation Angle                                               */
/********************************************************************/
int CTOFApiZ::getRotationAngle(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(GET_ROTATION_ANGLE, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Set LED Frequency ID                                             */
/********************************************************************/
int CTOFApiZ::setLEDfrequencyID(INT32 inTimeOutTime, INT32 inID, UINT8 *outStatus)
{
    INT32 ret	= 0;
    INT32 size	= 0;
    UINT8 sendData[16];

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	sendData[0] = inID & 0xFF;

    ret = TOF_SendCommand(SET_LED_FREQUENCY_ID, 1, sendData);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, NULL);

    return ret;
}

/********************************************************************/
/* Get LED Frequency ID                                             */
/********************************************************************/
int CTOFApiZ::getLEDfrequencyID(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(GET_LED_FREQUENCY_ID, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Set MIN_AMP (All range)                                         */
/********************************************************************/
int CTOFApiZ::setMinAmpAll(INT32 inTimeOutTime, INT32 inMinAmp, UINT8 *outStatus)
{
    INT32 ret	= 0;
    INT32 size	= 0;
    UINT8 sendData[16];

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	sendData[0] = inMinAmp & 0xFF;

    ret = TOF_SendCommand(SET_MIN_AMP_ALL, 1, sendData);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, NULL);

    return ret;
}

/********************************************************************/
/* Get MIN_AMP (All range)                                         */
/********************************************************************/
int CTOFApiZ::getMinAmpAll(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(GET_MIN_AMP_ALL, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Set MIN_AMP (Near range)                                         */
/********************************************************************/
int CTOFApiZ::setMinAmpNear(INT32 inTimeOutTime, INT32 inMinAmp, UINT8 *outStatus)
{
    INT32 ret	= 0;
    INT32 size	= 0;
    UINT8 sendData[16];

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	sendData[0] = inMinAmp & 0xFF;

    ret = TOF_SendCommand(SET_MIN_AMP_NEAR, 1, sendData);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, NULL);

    return ret;
}

/********************************************************************/
/* Get MIN_AMP (Near range)                                         */
/********************************************************************/
int CTOFApiZ::getMinAmpNear(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(GET_MIN_AMP_NEAR, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Get Theta Phi Table                                              */
/********************************************************************/
int CTOFApiZ::getThetaPhiTable(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(GET_THETA_PHI_TABLE, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Set Operation Check LED                                          */
/********************************************************************/
int CTOFApiZ::setOperationCheckLED(INT32 inTimeOutTime, INT32 inLED, UINT8 *outStatus)
{
    INT32 ret	= 0;
    INT32 size	= 0;
    UINT8 sendData[16];

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	sendData[0] = inLED & 0xFF;

    ret = TOF_SendCommand(SET_OPERATION_CHECK_LED, 1, sendData);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, NULL);

    return ret;
}

/********************************************************************/
/* Get Operation Check LED                                          */
/********************************************************************/
int CTOFApiZ::getOperationCheckLED(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(GET_OPERATION_CHECK_LED, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Set Response Speed                                               */
/********************************************************************/
int CTOFApiZ::setResponseSpeed(INT32 inTimeOutTime, INT32 inTransmissionSize, INT32 inTransmissionInterval, UINT8 *outStatus)
{
    INT32 ret	= 0;
    INT32 size	= 0;
    UINT8 sendData[16];

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	sendData[0] = inTransmissionSize & 0xFF;
	sendData[1] = (inTransmissionInterval >> 8) & 0xFF;
	sendData[2] = inTransmissionInterval & 0xFF;

    ret = TOF_SendCommand(SET_RESPONSE_SPEED, 3, sendData);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, NULL);

    return ret;
}

/********************************************************************/
/* Get Response Speed                                               */
/********************************************************************/
int CTOFApiZ::getResponseSpeed(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(GET_RESPONSE_SPEED, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Set ENR Threshold                                                */
/********************************************************************/
int CTOFApiZ::setENRthreshold(INT32 inTimeOutTime, INT32 inThreshold, UINT8 *outStatus)
{
    INT32 ret	= 0;
    INT32 size	= 0;
    UINT8 sendData[16];

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	sendData[0] = (inThreshold >> 8) & 0xFF;
	sendData[1] = inThreshold & 0xFF;

	ret = TOF_SendCommand(SET_ENR_THRESHOLD, 2, sendData);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, NULL);

    return ret;
}

/********************************************************************/
/* Get ENR Threshold                                                */
/********************************************************************/
int CTOFApiZ::getENRthreshold(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

	ret = TOF_SendCommand(GET_ENR_THRESHOLD, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Get Imager Temparature                                           */
/********************************************************************/
int CTOFApiZ::getImagerTemparature(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(GET_IMAGER_TEMPERATURE, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Get LED Temparature                                              */
/********************************************************************/
int CTOFApiZ::getLEDTemparature(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(GET_LED_TEMPERATURE, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Initialize Parameters                                            */
/********************************************************************/
int CTOFApiZ::initializeParameters(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(INITIALIZE_PARAMETERS, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}

/********************************************************************/
/* Software Reset                                                   */
/********************************************************************/
int CTOFApiZ::softwareReset(INT32 inTimeOutTime, UINT8 *outStatus, UINT8 **outData)
{
    INT32 ret	= 0;
    INT32 size	= 0;

    if(NULL == outStatus){
        return TOF_ERROR_PARAMETER;
    }

    ret = TOF_SendCommand(SOFTWARE_RESET, 0, NULL);
    if ( ret != 0 ) return ret;
    ret = TOF_ReceiveData(inTimeOutTime, outStatus, &size, outData);

    return ret;
}
