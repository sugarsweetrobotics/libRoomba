#include "Protocol.h"



using namespace ssr;


Protocol::Protocol(RoombaImpl* pRoomba, Transport* pTransport) {
  m_pRoomba = pRoomba;
  m_pTransport = pTransport;
}


Protocol::~Protocol()
{

}

void Protocol::Run()
{
  while(m_streamMode) {
    Thread::Sleep(m_SleepTime);
    if(m_Version != Roomba::VERSION_500_SERIES) {
      handleBasicData();
    }else {
      handleStreamData();
    }
    processOdometry();
  }
}

void Protocol::handleStreamData() {
  uint8_t *buffer = NULL;
  uint32_t readBytes;
  uint8_t header[2];
  uint8_t bufSize = 0;
  uint32_t sum;
  uint32_t timeout = 1 * 1000*1000;

  while(1) {
    m_pTransport->ReceiveData(header, 1, timeout);
    if(header[0] == 19) break;
  }

  m_pTransport->ReceiveData(header+1, 1, timeout);
  if(header[1] > bufSize) {
    delete buffer;
    bufSize = header[1];
    buffer = new uint8_t[bufSize];
  }
  sum = 19 + header[1];
  m_pTransport->ReceiveData(buffer, header[1], timeout);
  for(int i = 0;i < header[1];i++) {
    sum += buffer[i];
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
    uint8_t sensorId = buffer[counter];
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
      dataBuf |= buffer[counter];
      counter++;
      m_AsyncThreadMutex.Lock();
      m_SensorDataMap[(SensorID)sensorId] = dataBuf;
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
      dataBuf |= buffer[counter];
      counter++;
      dataBuf <<= 8;
      dataBuf |= buffer[counter];
      counter++;
#else
      dataBuf |= ((uint16_t)buffer[counter] << 8);
      counter++;
      dataBuf |= buffer[counter];
      counter++;
#endif
      m_AsyncThreadMutex.Lock();
      m_SensorDataMap[(SensorID)sensorId] = dataBuf;
      m_AsyncThreadMutex.Unlock();
      break;
    default:
      break;
    }
  } while(counter < header[1]-1);
}

void Protocol::handleBasicData()
{
  uint8_t opcode, buttons;
  uint16_t distance, angle;
  getSensorGroup2(&opcode, &buttons, (int16_t*)&distance, (int16_t*)&angle);
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


qqvoid Protocol::processOdometry(void)
{
  double lengthOfShaft = 0.235;
  double distance;
  double angle;
  if(m_Version == Roomba::MODEL_500SERIES) {
    
    if(!m_EncoderInitFlag) {
      m_EncoderInitFlag = true;
      m_EncoderRightOld = getRightEncoderCounts();
      m_EncoderLeftOld  = getLeftEncoderCounts();
      return;
    }
    
    int32_t encoderRight = getRightEncoderCounts();
    int32_t encoderLeft  = getLeftEncoderCounts();

    int32_t dR = encoderRight - m_EncoderRightOld;
    int32_t dL = encoderLeft  - m_EncoderLeftOld;
    
#define PULSES_TO_METER 0.000445558279992234
    
    if(dR > 32767) {
      dR -= 65535;
    } else if (dR < -32768) {
      dR += 65535;
    }
    
    if(dL > 32767) {
      dL -= 65535;
    } else if (dL < -32768) {
      dL += 65535;
    }
    
    distance = (dR + dL) * PULSES_TO_METER / 2 ;
    angle = (dR - dL) * PULSES_TO_METER / lengthOfShaft;
    
    m_EncoderRightOld = encoderRight;
    m_EncoderLeftOld = encoderLeft;
  } else {
    distance = getDistance();
    angle = getAngle() * 2 / lengthOfShaft;
    ///angle = getAngle() / 180.0 * 3.1415926;
  }
  double dX = distance * cos( m_Th + angle/2 );
  double dY = distance * sin( m_Th + angle/2 );
  m_X += dX;
  m_Y += dY;
  m_Th += angle;
  if(m_Th < -3.1415926536) {
    m_Th += 3.1415926536 * 2;
  } else if(m_Th > 3.1415896536) {
    m_Th -= 3.1415926536 * 2;
  }
}



void translateMessage(const Packet& packet)
{

}

