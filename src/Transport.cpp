#include "Transport.h"

using namespace net::ysuga;
using namespace net::ysuga::roomba;

Transport::Transport(const char* portName, const uint16_t baudrate)
{
	m_pSerialPort = new SerialPort(portName, baudrate);
}


Transport::~Transport(void)
{
	delete m_pSerialPort;
}

int32_t Transport::SendPacket(uint8_t opCode, 
						  const uint8_t *dataBytes /*= NULL*/,
						  const uint32_t dataSize /*= 0*/)
{
	uint8_t* buffer = new uint8_t[dataSize + 1];
	buffer[0] = opCode;
	for(unsigned int i = 1;i < dataSize + 1;i++) {
		buffer[i] = dataBytes[i-1];
	}
	m_pSerialPort->Write(buffer, dataSize + 1);
	delete buffer;
	return 0;
}


int32_t Transport::ReceiveData(uint8_t *buffer, uint32_t requestSize, uint32_t* readBytes)
{
	uint8_t* data = new uint8_t[requestSize];
	while ((uint32_t)m_pSerialPort->GetSizeInRxBuffer() < requestSize) {
		Sleep(100);
	}

	*readBytes = m_pSerialPort->Read(buffer, requestSize);
	return 0;
}
