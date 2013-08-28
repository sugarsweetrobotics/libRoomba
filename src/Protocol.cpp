#include <iostream>
#include "Protocol.h"

using namespace ssr;

Protocol::Protocol(RoombaImpl* pRoomba, Transport* pTransport, 
		   Odometry* pOdometry,
		   Version version) :
  m_pRoomba(pRoomba), m_pTransport(pTransport), m_pOdometry(pOdometry),
  m_pBuffer(NULL), m_streamMode(false), m_SleepTime(10), m_Version(version)
{
  m_pRoomba = pRoomba;
  m_pTransport = pTransport;
  m_pOdometry = pOdometry;
}


Protocol::~Protocol()
{
  if (m_streamMode) {
    m_streamMode = false;
    Join();
  }
  delete m_pBuffer;
}

void Protocol::startStreaming(void)
{
	uint8_t buffer[64];
	buffer[0] = m_numStreamingSensors;
	for (uint32_t i = 0;i < m_numStreamingSensors;i++) {
		buffer[i+1] = m_StreamingSensorIDs[i];
	}
    m_pTransport->SendPacket(OP_STREAM, buffer , buffer[0]+1);
}


void Protocol::Run()
{
  m_SensorDataMap.clear();
 /// std::cout << "Protocol::Run" << std::endl;
  if (m_Version == VERSION_500_SERIES) {
    const uint8_t numSensors = 3;
    SensorID defaultSensorId[numSensors] = {RIGHT_ENCODER_COUNTS,
				   LEFT_ENCODER_COUNTS, 
				   BUMPS_AND_WHEEL_DROPS};
	this->clearStreamingSensorID();
	for (int i = 0;i < numSensors;i++) {
		addStreamingSensorID(defaultSensorId[i]);
	}
	startStreaming();
  }

  m_streamMode = true;    
  while(m_streamMode) {
    Thread::Sleep(m_SleepTime);
	try {
		if(m_Version != VERSION_500_SERIES) {
		  handleBasicData();
		}else {
		  handleStreamData();
		}
    processOdometry();
	} catch (ChecksumException &ex) {
//		std::cout << "Protocol::Run ->" << ex.what() << std::endl;
	}
  }

  m_streamMode = false;
}

void Protocol::resumeSensorStream()
{
  if(m_Version == VERSION_500_SERIES) {
    uint8_t buf = 1;
    m_pTransport->SendPacket(OP_PAUSE_RESUME_STREAM, &buf, 1);
  }
}

void Protocol::suspendSensorStream()
{
  if(m_Version == VERSION_500_SERIES) {
    uint8_t buf = 0;
    m_pTransport->SendPacket(OP_PAUSE_RESUME_STREAM, &buf, 1);
  }
}


void Protocol::handleStreamData() {
  uint32_t readBytes;
  uint8_t header[2];
  uint8_t bufSize = 0;
  uint32_t sum;
  uint32_t timeout = 1 * 1000*1000;

  //std::cout << "handling StreamData" << std::endl;
  while(1) {
    m_pTransport->ReceiveData(header, 1, timeout);
    if(header[0] == 19) break;
  }

  m_pTransport->ReceiveData(header+1, 1, timeout);
  if(header[1] > bufSize) {
    delete m_pBuffer;
    bufSize = header[1];
    m_pBuffer = new uint8_t[bufSize];
  }
  sum = 19 + header[1];
  m_pTransport->ReceiveData(m_pBuffer, header[1], timeout);
  for(int i = 0;i < header[1];i++) {
    sum += m_pBuffer[i];
  }

  uint8_t check_sum;
  m_pTransport->ReceiveData(&check_sum, 1, timeout);
  sum += check_sum;
  
  if((sum & 0xFF) != 0) {
    throw ChecksumException();
  }
  
  //m_AsyncThreadReceiveCounter++;

  int counter = 0;
  do {
    uint8_t sensorId = m_pBuffer[counter];
    uint16_t dataBuf = 0;

    counter++;
    switch(sensorId) {
    case BUMPS_AND_WHEEL_DROPS:
    case WALL:
    case CLIFF_LEFT:
    case CLIFF_FRONT_LEFT:
    case CLIFF_FRONT_RIGHT:
    case CLIFF_RIGHT:
    case VIRTUAL_WALL:
    case WHEEL_OVERCURRENTS:
    case DIRT_DETECT:
    case UNUSED_BYTE:
    case INFRARED_CHARACTER_OMNI:
    case INFRARED_CHARACTER_LEFT:
    case INFRARED_CHARACTER_RIGHT:
    case BUTTONS:
    case CHARGING_STATE:
    case TEMPERATURE:
    case CHARGING_SOURCE_AVAILABLE:
    case OI_MODE:
    case SONG_NUMBER:
    case SONG_PLAYING:
    case NUMBER_OF_STREAM_PACKETS:
    case LIGHT_BUMPER:
    case STASIS:
      dataBuf |= m_pBuffer[counter];
      counter++;
      m_AsyncThreadMutex.Lock();
      m_SensorDataMap[(SensorID)sensorId] = dataBuf;
   //   std::cout <<"sensor:" << (int)sensorId << ", data:" << dataBuf << std::endl;
      m_AsyncThreadMutex.Unlock();
      break;
      
    case DISTANCE:
    case ANGLE:
    case VOLTAGE:
    case CURRENT:
    case BATTERY_CHARGE:
    case BATTERY_CAPACITY:
    case WALL_SIGNAL:
    case CLIFF_LEFT_SIGNAL:
    case CLIFF_FRONT_LEFT_SIGNAL:
    case CLIFF_FRONT_RIGHT_SIGNAL:
    case CLIFF_RIGHT_SIGNAL:
      //	UNUSED1,
      // UNUSED2,
    case REQUESTED_VELOCITY:
    case REQUESTED_RADIUS:
    case REQUESTED_RIGHT_VELOCITY:
    case REQUESTED_LEFT_VELOCITY:
    case RIGHT_ENCODER_COUNTS:
    case LEFT_ENCODER_COUNTS:
    case LIGHT_BUMP_LEFT_SIGNAL:
    case LIGHT_BUMP_FRONT_LEFT_SIGNAL:
    case LIGHT_BUMP_CENTER_LEFT_SIGNAL:
    case LIGHT_BUMP_CENTER_RIGHT_SIGNAL: 
    case LIGHT_BUMP_FRONT_RIGHT_SIGNAL:
    case LEFT_MOTOR_CURRENT:
    case RIGHT_MOTOR_CURRENT:
    case MAIN_BRUSH_MOTOR_CURRENT:
    case SIDE_BRUSH_MOTOR_CURRENT:
#ifdef __BIG_ENDIAN__
      dataBuf |= m_pBuffer[counter];
      counter++;
      dataBuf <<= 8;
      dataBuf |= m_pBuffer[counter];
      counter++;
#else
      dataBuf |= ((uint16_t)m_pBuffer[counter] << 8);
      counter++;
      dataBuf |= m_pBuffer[counter];
      counter++;
#endif
      m_AsyncThreadMutex.Lock();
      m_SensorDataMap[(SensorID)sensorId] = dataBuf;
 // std::cout <<"sensor:" << (int)sensorId << ", data:" << dataBuf << std::endl;
      m_AsyncThreadMutex.Unlock();
      break;
    default:
      break;
    }
  } while(counter < header[1]-1);
}

void Protocol::handleBasicData()
{
  uint32_t timeout = 1 * 1000 * 1000;
  uint8_t opcode, buttons;
  uint16_t distance, angle;
  getSensorGroup2(&opcode, &buttons, (int16_t*)&distance, (int16_t*)&angle, timeout);
  m_AsyncThreadMutex.Lock();
  m_SensorDataMap[BUTTONS] = buttons;
  m_SensorDataMap[ANGLE] = angle;
  m_SensorDataMap[DISTANCE] = distance;
  m_AsyncThreadMutex.Unlock();
}

void Protocol::getSensorGroup2(uint8_t *remoteOpcode, uint8_t *buttons, int16_t *distance, int16_t *angle, uint32_t timeout)
{
  uint8_t data[6];
  uint32_t readBytes;
  uint8_t sensorId = 2;

  m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
  m_pTransport->ReceiveData(data, 6, timeout);

  *remoteOpcode = data[0];
  *buttons = data[1];
  *distance = *angle = 0;
#ifdef __BIG_ENDIAN__
  *distance = ((uint16_t)data[2] << 8) | (data[3] & 0xFF);
  *angle    = ((uint16_t)data[4] << 8) | (data[5] & 0xFF);
#else
  *distance |= ((uint16_t)data[2] << 8) | ((uint16_t)data[3] & 0xFF);
  *angle    |= ((uint16_t)data[4] << 8) | ((uint16_t)data[5] & 0xFF);
#endif
  // std::cout << "D=" << *distance << " A=" << *angle << std::endl;
}

void Protocol::processOdometry(void)
{
  uint32_t timeout_us = 1*1000*1000;
  if(m_Version == VERSION_500_SERIES) {
    m_pOdometry->updatePositionEncoder(getSensorValue<uint16_t>(RIGHT_ENCODER_COUNTS, timeout_us),
				     getSensorValue<uint16_t>(LEFT_ENCODER_COUNTS, timeout_us),
					 m_Timer.getTimeOfDay().getUsec());
  } else {
    m_pOdometry->updatePositionAngleDistance(getSensorValue<uint16_t>(DISTANCE, timeout_us),
					   getSensorValue<uint16_t>(ANGLE, timeout_us)*3.141592 / 180.0,
					   m_Timer.getTimeOfDay().getUsec());
  }
}

