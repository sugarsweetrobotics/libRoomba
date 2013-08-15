#pragma once
#include "type.h"
#include "Roomba.h"

namespace ssr {
  class ROOMBA_API Packet {
  private:
    uint8_t m_OpCode;
    uint8_t* m_pData;
    uint32_t m_Size;
  public:
    uint8_t getOpCode() const {return m_OpCode;}
    uint8_t *getData() const {return m_pData;}
    uint32_t getSize() const {return m_Size;}
  public:
    Packet(uint8_t op_code, uint8_t *dataBytes=NULL, uint32_t dataSize=0) {
      initialize(op_code, dataBytes, dataSize);
    }

    Packet(const Packet& packet) {
      initialize(packet.m_OpCode, packet.m_pData, packet.m_Size);
    }

    virtual ~Packet() {
      delete m_pData;
    }

    void operator=(const Packet& packet) {
      initialize(packet.m_OpCode, packet.m_pData, packet.m_Size);
    }

    void initialize(uint8_t op_code, uint8_t *dataBytes=NULL, uint32_t dataSize=0) {
      m_OpCode = op_code;
      if (dataSize > 0) {
	m_pData = new uint8_t[dataSize];
	memcpy(m_pData, dataBytes, dataSize);
	m_Size = dataSize;
      } else {
	m_pData= NULL;
	m_Size = 0;
      }
    }


  };
}
