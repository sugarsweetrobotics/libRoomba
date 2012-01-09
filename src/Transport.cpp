#include "Transport.h"

using namespace net::ysuga;
using namespace net::ysuga::roomba;

Transport::Transport(const char* portName, const int baudrate)
{
	m_pSerialPort = new SerialPort(portName, baudrate);
}


Transport::~Transport(void)
{
	delete m_pSerialPort;
}

int Transport::SendPacket(unsigned char opCode, 
						  const unsigned char *dataBytes /*= NULL*/,
						  const unsigned int dataSize /*= 0*/)
{
	unsigned char* buffer = new unsigned char[dataSize + 1];
	buffer[0] = opCode;
	for(unsigned int i = 1;i < dataSize + 1;i++) {
		buffer[i] = dataBytes[i-1];
	}
	m_pSerialPort->Write(buffer, dataSize + 1);
	delete buffer;
	return 0;
}


int Transport::ReceiveData(unsigned char *buffer, unsigned int requestSize, unsigned int* readBytes)
{
	unsigned char* data = new unsigned char[requestSize];
	while ((unsigned int)m_pSerialPort->GetSizeInRxBuffer() <= requestSize) {
		
	}

	*readBytes = m_pSerialPort->Read(buffer, requestSize);
	return 0;
}
