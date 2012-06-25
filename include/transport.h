#pragma once

#include "type.h"

#include "SerialPort.h"

namespace net {
	namespace ysuga {
		namespace roomba {


			class Transport
			{
			private:
				SerialPort* m_pSerialPort;

			public:
				Transport(const char* portName, const uint16_t baudrate);

				~Transport(void);

				int32_t SendPacket(uint8_t opCode, const uint8_t *dataBytes = NULL, const uint32_t dataSize = 0);

				int32_t ReceiveData(uint8_t *buffer, uint32_t maxBufferSize, uint32_t* readByte);
			};
		}
	}
}