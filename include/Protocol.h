#pragma once

#include "Packet.h"
#include "Transport.h"
#include "Thread.h"
#include "Timer.h"
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
	Timer m_Timer;

    Mutex m_AsyncThreadMutex;

  private:
    uint8_t *m_pBuffer;

  private:
    std::map<SensorID, int32_t> m_SensorDataMap;
    bool m_streamMode;
    uint32_t m_SleepTime;
    Version m_Version;
	uint8_t m_StreamingSensorIDs[64];
	uint8_t m_numStreamingSensors;

  public:
	  void clearStreamingSensorID() {
		  m_numStreamingSensors = 0;
	  }

	  void addStreamingSensorID(const uint8_t id) {
		  m_StreamingSensorIDs[m_numStreamingSensors] = id;
		  m_numStreamingSensors++;
	  }

	  void startStreaming(void);

  public:
    Protocol(RoombaImpl* pRoomba, Transport* pTransport, Odometry* pOdometry, Version version);
    virtual ~Protocol();

  public:
    template<typename T>
      T getSensorValue(const uint8_t sensorId, const uint32_t timeout=ROOMBA_INFINITE) {
      T data;

	  if (m_streamMode) {

		  if (m_SensorDataMap.find((SensorID)sensorId) != m_SensorDataMap.end()) {
			  //int32_t buf = m_SensorDataMap[sensorId];
			  //uint32_t mask = 0;
			  //for (int i = 0;i < sizeof(T);i++) {
			  //  mask |= 0x0000000F << i*8;
			  //}
			  data = m_SensorDataMap[(SensorID)sensorId];
			  return data;
		  } else {
			  addStreamingSensorID((SensorID)sensorId);
			  startStreaming();
			  Thread::Sleep(10);
			  while(m_SensorDataMap.find((SensorID)sensorId) == m_SensorDataMap.end()) {
				  Thread::Sleep(10);
			  }
			  data = m_SensorDataMap[(SensorID)sensorId];
			  return data;
		  }
	  } else {
		  m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
		  m_pTransport->ReceiveData((uint8_t*)&data, sizeof(T), timeout);
	  }
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
