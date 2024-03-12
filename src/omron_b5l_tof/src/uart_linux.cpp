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
#include <iostream>

#include <rclcpp/rclcpp.hpp>

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

/* original code */
int com_init(const char *device, unsigned long baudrate)
{
    int rtn;
    int nBaud;
    // char device[16];
    // struct termios tio;

    pthread_attr_t tattr;
    struct sched_param spp;

    com_close();

    // sprintf(device, "/dev/ttyUSB%d", stat->com_num);
    hCom = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);       // Open a Device File (Serial Port)
    if ( hCom == -1 ) {
        fprintf(stderr, "Can't open (%s) hCom = %d\n", device, hCom);
        return 0;
    }

    tcgetattr(hCom, &save_options);

    memset(&save_options,0,sizeof(save_options));

    // Set Baud Rate
    switch(baudrate) {
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

    save_options.c_cflag |= CS8 | CLOCAL | CREAD;
    save_options.c_lflag &= ~(ECHO | ECHOE | ISIG);
    save_options.c_cc[VMIN] = 0;
    save_options.c_cc[VTIME] = 0;
    
    cfsetispeed(&save_options,nBaud);
    cfsetospeed(&save_options,nBaud);

    speed_t input_speed = cfgetispeed(&save_options);
    speed_t output_speed = cfgetospeed(&save_options);

    RCLCPP_INFO(rclcpp::get_logger("b5l_tof_device"), "input_speed = %d, output_speed = %d", input_speed, output_speed);

    // Set the Device
    tcsetattr(hCom,TCSANOW,&save_options);

    // Make a Critical Section
    InitializeCriticalSection(&cs);
	m_bThread = 1;

    /* Initialize with the Default Setting */
    rtn = pthread_attr_init(&tattr);
    /* Get Existing Scheduling Parameter */
    rtn = pthread_attr_getschedparam(&tattr, &spp);
    /* Set Priority, No Change for Others */
    spp.sched_priority = 50;
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
    int nTotalSize = 0;
    int nSize;

    // Set the file descriptor to non-blocking mode
    int flags = fcntl(hCom, F_GETFL, 0);
    fcntl(hCom, F_SETFL, flags | O_NONBLOCK);

    // Use select() to wait for data to become available
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(hCom, &read_fds);
    struct timeval timeout = { inTimeOutTimer / 1000, (inTimeOutTimer % 1000) * 1000 };
    time_t start, end;
    time(&start);
    while (nTotalSize < len) {
        int sel = select(hCom + 1, &read_fds, NULL, NULL, &timeout);
        if (sel == -1) {
            if (errno == EINTR)
                continue;  // Interrupted by signal
            else
                break;  // Error occurred
        } else if (sel == 0) {
            break;  // Timeout
        }
        // Data is available to be read
        nSize = read(hCom, &buf[nTotalSize], len - nTotalSize);
        if (nSize == -1) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;  // No data available yet
            else
                break;  // Error occurred
        }
        nTotalSize += nSize;

        // Timeout check
        time(&end);
        if ((end - start) * 1000 >= inTimeOutTimer) {
            break;
        }
    }

    // Set the file descriptor back to blocking mode
    fcntl(hCom, F_SETFL, flags);

    return nTotalSize;
}

// /**original code */
// int _com_recv(int inTimeOutTimer, unsigned char *buf, int len)
// {
//     int nSize = 0;
//     int nTotalSize = 0;

//     time_t start,end;

//     if ( hCom != -1 ) {
//         //Start timer
//         time(&start);
//         while ( nTotalSize < len ) {
//             nSize = read(hCom, &buf[nTotalSize], len-nTotalSize);
//             nTotalSize += nSize;

//             //Time Out Check
//             time(&end);
//             if( (end - start)*1000 >= inTimeOutTimer){
//                 break;
//             }
//         }
//     }
//     return nTotalSize;
// }

void *ExecThread(void *pParam)
{
    int rlen;
    unsigned char rxbuf[256]; // increase buffer size

    while (hCom != -1) {
        rlen = _com_recv(0, rxbuf, sizeof(rxbuf));

        if (rlen > 0) {
            EnterCriticalSection(&cs);
            // Received Data
            for (int i = 0; i < rlen; i++) {
                m_acRcvBuf[m_ptrReceive++] = rxbuf[i];
                if (m_ptrReceive >= SIZE_RCVBUF) {
                    m_ptrReceive = 0;
                }
                m_nReceiveCount++;
                if (m_nReceiveCount > SIZE_RCVBUF) {
                    m_nReceiveCount--;
                    m_ptrRead++;
                    if (m_ptrRead >= SIZE_RCVBUF) {
                        m_ptrRead = 0;
                    }
                }
            }
            LeaveCriticalSection(&cs);
        } else {
            // use a non-blocking read operation instead of usleep()
            fd_set fds;
            struct timeval tv;
            FD_ZERO(&fds);
            FD_SET(hCom, &fds);
            tv.tv_sec = 0;
            tv.tv_usec = 100;
            int ret = select(hCom + 1, &fds, NULL, NULL, &tv);
            if (ret < 0) {
                // handle error
            }
        }

        if (!m_bThread) {
            break;
        }
    }

    return NULL;
}

// /********************************************************************/
// /* Execute a Thread original                                        */
// /********************************************************************/
// void *ExecThread(void *pParam)
// {
// 	int i;
// 	int rlen;
// 	unsigned char rxbuf[1024];

// 	m_ptrRead = 0;
// 	m_ptrReceive = 0;
// 	m_nReceiveCount = 0;
//     i = sizeof(pParam); 

//     // Execute Recieving
// 	while ( hCom != -1 ) {
// 		rlen = _com_recv(0, rxbuf, sizeof(rxbuf));

// 		if ( rlen > 0 ) {
// 		    EnterCriticalSection(&cs);
// 			// Recieved Data
// 			for ( i = 0 ; i < (int)rlen ; i++ ) {
// 				m_acRcvBuf[m_ptrReceive++] = rxbuf[i];
// 				if ( m_ptrReceive >= SIZE_RCVBUF ) {
// 					m_ptrReceive = 0;
// 				}
// 				m_nReceiveCount++;
// 				if ( m_nReceiveCount > SIZE_RCVBUF ) {
// 					m_nReceiveCount--;

// 					m_ptrRead++;
// 					if ( m_ptrRead >= SIZE_RCVBUF ) {
// 						m_ptrRead = 0;
// 					}
// 				}
// 			}
// 		    LeaveCriticalSection(&cs);
// 		} else {
// 			usleep(1000);
// 		}

// 		if ( !m_bThread ) {
// 			break;
// 		}
// 	}

// 	return NULL;
// }

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
        usleep(100);
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
