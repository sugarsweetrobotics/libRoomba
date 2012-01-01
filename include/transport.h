#pragma once



#include "SerialPort.h"

namespace net {
	namespace ysuga {
		namespace roomba {
			class Transport
			{
			private:
				SerialPort* m_pSerialPort;

			public:
				Transport(const char* portName, const int baudrate);

				~Transport(void);

				int SendPacket(unsigned char opCode, const unsigned char *dataBytes = NULL, const unsigned int dataSize = 0);

				int ReceiveData(unsigned char *buffer, unsigned int maxBufferSize, unsigned int* readByte);
			};
		}
	}
}