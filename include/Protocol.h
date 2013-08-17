#pragma once

#include "Packet.h"
#include "Transport.h"
#include "Thread.h"
#include "op_code.h"
#include "Odometry.h"
#include <map>

namespace ssr {
  class RoombaImpl;


  class Protocol : public Thread {
  private:
    RoombaImpl* m_pRoomba;
    Transport* m_pTransport;
    Odometry* m_pOdometry;

    Mutex m_AsyncThreadMutex;

  private:
    uint8_t *m_pBuffer;

  private:
    std::map<SensorID, int32_t> m_SensorDataMap;
    bool m_streamMode;
    uint32_t m_SleepTime;
    Version m_Version;

  public:
    Protocol(RoombaImpl* pRoomba, Transport* pTransport, Odometry* pOdometry, Version version);
    virtual ~Protocol();

  public:
    template<typename T>
      T getSensorValue(const uint8_t sensorId, const uint32_t timeout=ROOMBA_INFINITE) {
      T data;
      m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
      m_pTransport->ReceiveData((uint8_t*)&data, sizeof(T), timeout);
      return data;
    }

  public:
    void resumeSensorStream();
    void suspendSensorStream();

  private:
    void handleStreamData();
    void handleBasicData();
    void processOdometry();
    void getSensorGroup2(uint8_t *remoteOpcode, uint8_t *buttons, int16_t *distance, int16_t *angle, uint32_t timeout);
  public:
    void Run();
  };

}
