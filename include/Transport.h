/**
 * @filename Transport.h
 * @author Yuki Suga (ysuga.net)
 * @copyright Sugar Sweet Robotics Co., Ltd.
 */


#pragma once

#include "type.h"
#include "SerialPort.h"
#include "Packet.h"
#include "Timer.h"

#define ROOMBA_INFINITE 0xFFFFFFFF
namespace ssr {

  class Transport {
  private:
    SerialPort* m_pSerialPort;
    uint8_t *m_pBuffer;
    Timer m_Timer;
  public:
    Transport(const char* portName, const uint16_t baudrate);
    
    ~Transport(void);
	
    void SendPacket(const Packet& packet) {SendPacket(packet.getOpCode(), packet.getData(), packet.getSize());}

    void SendPacket(const uint8_t opCode, const uint8_t* pData=NULL, const uint32_t Size=0) {
      m_pSerialPort->Write(&opCode, 1);
      m_pSerialPort->Write(pData, Size);
    }
    
    Packet ReceivePacket(const uint32_t requestSize, const uint32_t timeout = ROOMBA_INFINITE);
    void ReceiveData(uint8_t *pData, const uint32_t requestSize, const uint32_t timeout = ROOMBA_INFINITE) ;
  };
}

