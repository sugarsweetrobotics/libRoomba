/********************************************************
 * SerialPort.h
 *
 * Portable Serial Port Class Library for Windows and Unix.
 * @author ysuga@ysuga.net
 * @date 2010/11/02
 ********************************************************/

#ifndef SERIAL_PORT_HEADER_INCLUDED
#define SERIAL_PORT_HEADER_INCLUDED

#include <exception>
#include "ComAccessException.h"
#include "ComOpenException.h"
#include "ComStateException.h"


#ifdef WIN32
#include <windows.h>
#endif

namespace net {
	namespace ysuga {

		/***************************************************
		 * SerialPort
		 *
		 * @brief Portable Serial Port Class
		 ***************************************************/
		class SerialPort 
		{
		private:
#ifdef WIN32

			HANDLE m_hComm;
#else

			/**
			 * @brief file descriptor
			 */
			int m_Fd;
#endif



		public:
			/**
			 * @brief Constructor
			 * 
			 * @param filename Filename of Serial Port (eg., "COM0", "/dev/tty0")
			 * @baudrate baudrate. (eg., 9600, 115200)
			 */
			SerialPort(const char* filename, int baudrate);

			/**
			 * @brief Destructor
			 */
			~SerialPort();

		public:
			/**
			 * @brief flush receive buffer.
			 * @return zero if success.
			 */
			void FlushRxBuffer();

			/**
			 * @brief flush transmit buffer.
			 * @return zero if success.
			 */
			void FlushTxBuffer();

		public:
			/**
			 * @brief Get stored datasize of in Rx Buffer
			 * @return Stored Data Size of Rx Buffer;
			 */
			int GetSizeInRxBuffer();

			/**
			 * @brief write data to Tx Buffer of Serial Port.
			 *
			 */
			int Write(const void* src, const unsigned int size);

			/**
			 * @brief read data from RxBuffer of Serial Port 
			 			 */
			int Read(void *dst, const unsigned int size);

		};

	};//namespace ysuga
};//namespace org

#endif

/*******************************************************
 * Copyright  2010, ysuga.net all rights reserved.
 *******************************************************/
