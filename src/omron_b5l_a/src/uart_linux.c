/*---------------------------------------------------------------------------*/
/* Copyright(C)  2017-2020  OMRON Corporation                                */
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
#include <stdarg.h>
#include <stdlib.h>
#include <fcntl.h>
#include <memory.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include "uart.h"

typedef pthread_mutex_t CRITICAL_SECTION;

#define	SIZE_RCVBUF	(1 * 1024 * 1024)

static void *ExecThread(void *pParam);

static CRITICAL_SECTION cs;

static int hCom = -1;
static struct termios save_options;

static int m_bThread = 0;	    		// Continue Flag
static pthread_t m_hThread = (pthread_t)NULL;		// Recieving Thread

static int m_ptrRead = 0;
static int m_ptrReceive = 0;
static unsigned char m_acRcvBuf[SIZE_RCVBUF];
static int m_nReceiveCount = 0;

void InitializeCriticalSection(CRITICAL_SECTION *section)
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr); /* 2012/07/26 added */
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init(section, &attr);
    pthread_mutexattr_destroy(&attr);
}
void EnterCriticalSection(CRITICAL_SECTION *section)
{
    pthread_mutex_lock(section);
}
void LeaveCriticalSection(CRITICAL_SECTION *section)
{
    pthread_mutex_unlock(section);
}

void com_sleep(int nSleep)
{
    usleep(nSleep*100);
}

/* UART */
void com_close(void)
{
    // Stop Recieving Process
    if (m_hThread != 0)
    {
        (void)pthread_cancel(m_hThread);
    }
    if ( hCom != -1 ) {
		m_bThread = 0;

        tcsetattr(hCom, TCSANOW, &save_options);    // Restore the existing configuration
        close(hCom);
        hCom = -1;
    }
}

int com_init(S_STAT *stat)
{
    int rtn;
    int nBaud;
    char device[16];
    struct termios tio;

    pthread_attr_t tattr;
    struct sched_param spp;

    com_close();

    sprintf(device, "/dev/ttyUSB%d", stat->com_num);
    hCom = open(device,O_RDWR | O_NOCTTY);          // Open a Device File (Serial Port)
    if ( hCom == -1 ) {
        fprintf(stderr, "Can't open (%s) hCom = %d\n", device, hCom);
        return 0;
    }

    tcgetattr(hCom, &save_options);

    memset(&tio,0,sizeof(tio));
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_lflag &= ~(ECHO | ECHOE | ISIG);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    // Set Baud Rate
    switch(stat->BaudRate) {
    case 9600:
        nBaud = B9600;
        break;
    case 38400:
        nBaud = B38400;
        break;
    case 115200:
        nBaud = B115200;
        break;
    case 230400:
        nBaud = B230400;
        break;
    case 460800:
        nBaud = B460800;
        break;
    case 921600:
        nBaud = B921600;
        break;
    default:
        nBaud = B9600;
        break;
    }
    cfsetispeed(&tio,nBaud);
    cfsetospeed(&tio,nBaud);

    // Set the Device
    tcsetattr(hCom,TCSANOW,&tio);

    // Make a Critical Section
    InitializeCriticalSection(&cs);
	m_bThread = 1;

    /* Initialize with the Default Setting */
    rtn = pthread_attr_init(&tattr);
    /* Get Existing Scheduling Parameter */
    rtn = pthread_attr_getschedparam(&tattr, &spp);
    /* Set Priority, No Change for Others */
    //spp.sched_priority = 50;
    spp.sched_priority = 10;

    /* Set new Scheduling Parameter */
    rtn = pthread_attr_setschedparam(&tattr, &spp);
    /* Use Specified New Priority */
    rtn = pthread_create(&m_hThread, &tattr, ExecThread, NULL);

    return (rtn == 0);
}

void com_clear();
int com_send(unsigned char *buf, int len)
{
    int nSize = 0;

    if ( hCom != -1 ) {
        if ( buf[0] == 0xFE ) {
            com_clear();
        }
        nSize = write(hCom, buf, len);
    }
    return nSize;
}

int _com_recv(int inTimeOutTimer, unsigned char *buf, int len)
{
    int nSize = 0;
    int nTotalSize = 0;

    time_t start,end;

    if ( hCom != -1 ) {
        //Start timer
        time(&start);
        while ( nTotalSize < len ) {
            nSize = read(hCom, &buf[nTotalSize], len-nTotalSize);
            nTotalSize += nSize;

            //Time Out Check
            time(&end);
            if( (end - start)*1000 >= inTimeOutTimer){
                break;
            }
        }
    }
    return nTotalSize;
}

/********************************************************************/
/* Execute a Thread                                                 */
/********************************************************************/
void *ExecThread(void *pParam)
{
	int i;
	int rlen;
	unsigned char rxbuf[1024];

	m_ptrRead = 0;
	m_ptrReceive = 0;
	m_nReceiveCount = 0;
    i = sizeof(pParam); 

    // Execute Recieving
	while ( hCom != -1 ) {
		rlen = _com_recv(0, rxbuf, sizeof(rxbuf));

		if ( rlen > 0 ) {
		    EnterCriticalSection(&cs);
			// Recieved Data
			for ( i = 0 ; i < (int)rlen ; i++ ) {
				m_acRcvBuf[m_ptrReceive++] = rxbuf[i];
				if ( m_ptrReceive >= SIZE_RCVBUF ) {
					m_ptrReceive = 0;
				}
				m_nReceiveCount++;
				if ( m_nReceiveCount > SIZE_RCVBUF ) {
					m_nReceiveCount--;

					m_ptrRead++;
					if ( m_ptrRead >= SIZE_RCVBUF ) {
						m_ptrRead = 0;
					}
				}
			}
		    LeaveCriticalSection(&cs);
		} 
        else 
        {
			usleep(10);//1000
		}

		if ( !m_bThread ) 
        {
			break;
		}
	}

	return NULL;
}

/********************************************************************/
/* Recieving API functions                                          */
/********************************************************************/
// Reset Recieving
void com_clear()
{
	EnterCriticalSection(&cs);

	m_ptrRead = 0;
	m_ptrReceive = 0;
	m_nReceiveCount = 0;

	LeaveCriticalSection(&cs);
}

// Confirm Recieving
int com_length()
{
	int nCount = 0;

	EnterCriticalSection(&cs);

	nCount = m_nReceiveCount;

	LeaveCriticalSection(&cs);
	return nCount;
}

// Recieve Data
int com_recv(int inTimeOutTimer, unsigned char *buf, int len)
{
	int nSize = 0;

    time_t start,end;

    //Start timer
    time(&start);
    do{
        usleep(100);//10000
        if ( com_length() >= len ) break;

        //Time Out Check
        time(&end);
        if( (end - start)*1000 >= inTimeOutTimer){
            break;
        }
    }while(1);

	EnterCriticalSection(&cs);

	if ( len > m_nReceiveCount ) {
		len = m_nReceiveCount;
	}

	if ( m_ptrRead + len > SIZE_RCVBUF ) {
		memcpy(&buf[nSize], &m_acRcvBuf[m_ptrRead], SIZE_RCVBUF - m_ptrRead);
		nSize += (SIZE_RCVBUF - m_ptrRead);
		m_ptrRead = 0;
	}
	memcpy(&buf[nSize], &m_acRcvBuf[m_ptrRead], len - nSize);
	m_ptrRead += (len - nSize);
	nSize += (len - nSize);
	m_nReceiveCount -= nSize;

	LeaveCriticalSection(&cs);
	return nSize;
}

// Send Data with Critical Section
int com_write(unsigned char *buf, int len)
{
	int nRet;

	EnterCriticalSection(&cs);
	nRet = com_send(buf, len);
	LeaveCriticalSection(&cs);
	return nRet;
}
