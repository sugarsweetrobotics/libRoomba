#pragma once

#include "Packet.h"
#include "RoombaImpl.h"
#include "Transport.h"
#include "Thread.h"

namespace ssr {

  class Protocol : public Thread {
  private:
    RoombaImpl* m_pRoomba;
    Transport* m_pTransport;

    Mutex m_AsyncThreadMutex;
    
  public:
    Protocol(RoombaImpl* pRoomba, Transport* pTransport);
    virtual ~Protocol();


  public:
    void translateMessage(const Packet& packet);

    template<typename T>
      T getSensorValue(const uint8_t sensorId, const uint32_t timeout=ROOBMA_INFINITE) {
      T data;
      m_pTransport->SensPacket(OP_SENSORS, &sensorId, 1);
      m_pTransport->ReceiveData(&data, sizeof(T), timeout);
      return data;
    }

    void handleStreamData();
    

  public:
    void Run();
  };

}
