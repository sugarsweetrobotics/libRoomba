#include "Thread.h"
#include "Transport.h"

#include "Timer.h"

using namespace ssr;

Transport::Transport(const char* portName, const uint16_t baudrate) {
  m_pSerialPort = new SerialPort(portName, baudrate);
  m_pBuffer = new uint8_t[256];
}


Transport::~Transport(void) {
  delete m_pBuffer;
  delete m_pSerialPort;
}


Packet Transport::ReceivePacket(const uint32_t requestSize, const uint32_t timeout) {
  ReceiveData(m_pBuffer, requestSize, timeout);
  return Packet(m_pBuffer[0], m_pBuffer+1, requestSize);
}

void Transport::ReceiveData(uint8_t *pData, const uint32_t requestSize, const uint32_t timeout) {
  m_Timer.tick();
  TimeSpec current;
  while ((uint32_t)m_pSerialPort->GetSizeInRxBuffer() < requestSize) {
    Thread::Sleep(10);
    m_Timer.tack(&current);
    if (current.sec * 1000*1000 + current.usec> timeout) {
      throw TimeoutException();
    }
  }
  uint32_t readBytes = m_pSerialPort->Read(pData, requestSize);
}
