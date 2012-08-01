

#include "Roomba.h"
#include <iostream>


#include "op_code.h"

using namespace net::ysuga::roomba;

Roomba::Roomba(const uint32_t model, const char *portName, const uint32_t baudrate) :
m_isStreamMode(0), 
m_X(0), m_Y(0), m_Th(0), m_EncoderInitFlag(0),
m_MainBrushFlag(MOTOR_OFF), m_SideBrushFlag(MOTOR_OFF), m_VacuumFlag(MOTOR_OFF)
{
  if(model == MODEL_CREATE) {
	  m_Version = VERSION_ROI;
  } else {
	  m_Version = VERSION_500_SERIES;
  }

  m_ledFlag = m_intensity = m_color = 0;
  
  m_pTransport = new Transport(portName, baudrate);
  start();

  buffer = NULL;
}


Roomba::~Roomba(void)
{
  if(m_isStreamMode) {
    suspendSensorStream();
    m_isStreamMode = false;
    Join();
  }
  safeControl();
  start();
  delete m_pTransport;
}

void Roomba::setMode(Mode mode)
{
	switch(mode) {
	case MODE_START:
		m_pTransport->SendPacket(OP_START);
		m_CurrentMode = MODE_PASSIVE;
		break;	

	case MODE_SAFE:
		m_pTransport->SendPacket(OP_SAFE);
		m_CurrentMode = MODE_SAFE;
		break;

	case MODE_FULL:
		m_pTransport->SendPacket(OP_FULL);
		m_CurrentMode = MODE_FULL;
		break;
		
	case MODE_SPOT_CLEAN:
		m_pTransport->SendPacket(OP_SPOT);
		m_CurrentMode = MODE_PASSIVE;
		break;

	case MODE_NORMAL_CLEAN:
		m_pTransport->SendPacket(OP_CLEAN);
		m_CurrentMode = MODE_PASSIVE;
		break;

	case MODE_MAX_TIME_CLEAN:
		m_pTransport->SendPacket(OP_MAX);
		m_CurrentMode = MODE_PASSIVE;
		break;

	case MODE_DOCK:
		m_pTransport->SendPacket(OP_DOCK);
		m_CurrentMode = MODE_PASSIVE;
		break;

	case MODE_POWER_DOWN:
		m_pTransport->SendPacket(OP_POWER);
		m_CurrentMode = MODE_PASSIVE;
		break;

	default:
		break;
	}

	Thread::Sleep(100);
}

void Roomba::drive(uint16_t translation, uint16_t turnRadius) {
	if(getMode() != MODE_SAFE && getMode() != MODE_FULL) {
		throw PreconditionNotMetError();
	}

	uint8_t data[4];
#ifdef __BIG_ENDIAN__
	data[0] = translation & 0xFF;
	data[1] = (translation >> 8) & 0xFF;
	data[2] = turnRadius & 0xFF;
	data[3] = (turnRadius >> 8) & 0xFF;
#else
	data[0] = (translation >> 8) & 0xFF;
	data[1] = translation & 0xFF;
	data[2] = (turnRadius >> 8) & 0xFF;
	data[3] = turnRadius & 0xFF;
#endif
	m_pTransport->SendPacket(OP_DRIVE, data, 4);
}

void Roomba::driveDirect(int16_t rightWheel, int16_t leftWheel) {
	if(getMode() != MODE_SAFE && getMode() !=MODE_FULL) {
		throw PreconditionNotMetError();
	}

	uint8_t data[4];
#ifdef __BIG_ENDIAN__
	data[1] = (rightWheel >> 8) & 0xFF;
	data[0] = rightWheel & 0xFF;
	data[3] = (leftWheel >> 8) & 0xFF;
	data[2] = leftWheel & 0xFF;
#else
	data[1] = rightWheel & 0xFF;
	data[0] = (rightWheel >> 8) & 0xFF;
	data[3] = leftWheel & 0xFF;
	data[2] = (leftWheel >> 8) & 0xFF;

#endif
	m_pTransport->SendPacket(OP_DRIVE_DIRECT, data, 4);
}

void Roomba::drivePWM(int16_t rightWheel, int16_t leftWheel) {
	if(getMode() != MODE_SAFE && getMode() != MODE_FULL) {
		throw PreconditionNotMetError();
	}

	uint8_t data[4];
#ifdef __BIG_ENDIAN__
	data[0] = rightWheel & 0xFF;
	data[1] = (rightWheel >> 8) & 0xFF;
	data[2] = leftWheel & 0xFF;
	data[3] = (leftWheel >> 8) & 0xFF;
#else
	data[0] = (rightWheel >> 8) & 0xFF;
	data[1] = rightWheel & 0xFF;
	data[2] = (leftWheel >> 8) & 0xFF;
	data[3] = leftWheel & 0xFF;

#endif
	m_pTransport->SendPacket(OP_DRIVE_PWM, data, 4);
}

LIBROOMBA_API void Roomba::driveMotors(Motors mainBrush, Motors sideBrush, Motors vacuum)
{
	if(getMode() != MODE_SAFE && getMode() !=MODE_FULL) {
		throw PreconditionNotMetError();
	}

	uint8_t data = 0;
	switch(mainBrush) {
		case MOTOR_CW:
			data = MainBrush;
			break;
		case MOTOR_CCW:
			data = MainBrush | MainBrushOpposite;
			break;
		default:
			break;
	}

	switch(sideBrush) {
		case MOTOR_CW:
			data |= SideBrush;
			break;
		case MOTOR_CCW:
			data |= SideBrush | SideBrushOpposite;
			break;
		default:
			break;
	}

	if(vacuum == MOTOR_ON) {
		data |= Vacuum;
	}
	
	
	m_pTransport->SendPacket(OP_MOTORS, &data, 1);
}



void Roomba::setLED(uint8_t leds, uint8_t intensity, uint8_t color /* = 127*/) 
{
	uint8_t buf[3] = {leds, color, intensity};
	m_pTransport->SendPacket(OP_LEDS, buf, 3);
}


				
void Roomba::startSensorStream(uint8_t* requestingSensors, uint32_t numSensors)
{
	m_SensorDataMap.clear();

	if(m_Version == Roomba::VERSION_500_SERIES) {
		uint8_t* buffer = new uint8_t[numSensors + 1];
		buffer[0] = numSensors;
		for(unsigned int i = 0;i < numSensors;i++) {
			m_SensorDataMap[(SensorID)requestingSensors[i]] = 0;
			buffer[i+1] = requestingSensors[i];
		}
		m_pTransport->SendPacket(OP_STREAM, buffer, numSensors+1);
		
		m_isStreamMode = true;
		resumeSensorStream();
		Start();

		getRightEncoderCounts();
		getLeftEncoderCounts();

		delete buffer;
	} else {
		m_isStreamMode = true;
		Start();
	}
}

void Roomba::resumeSensorStream()
{
	if(m_Version == Roomba::VERSION_500_SERIES) {
		uint8_t buf = 1;
		m_pTransport->SendPacket(OP_PAUSE_RESUME_STREAM, &buf, 1);
	}
}

void Roomba::suspendSensorStream()
{
	if(m_Version == Roomba::VERSION_500_SERIES) {
		uint8_t buf = 0;
		m_pTransport->SendPacket(OP_PAUSE_RESUME_STREAM, &buf, 1);
	}
}


void Roomba::getSensorGroup2(uint8_t *remoteOpcode, uint8_t *buttons, int16_t *distance, int16_t *angle)
{
	uint8_t data[6];
	uint32_t readBytes;
	uint8_t sensorId = 2;
	m_AsyncThreadMutex.Lock();
	m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
	m_pTransport->ReceiveData(data, 6, &readBytes);
	m_AsyncThreadMutex.Unlock();
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
	std::cout << "D=" << *distance << " A=" << *angle << std::endl;
}



void Roomba::getSensorValue(uint8_t sensorId, uint16_t *value) {
	uint8_t data[2];
	uint32_t readBytes;
	m_AsyncThreadMutex.Lock();
	m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
	m_pTransport->ReceiveData(data, 2, &readBytes);
	m_AsyncThreadMutex.Unlock();
#ifdef __BIG_ENDIAN__
	*value = ((uint16_t)data[0] << 8) | (data[1] & 0xFF);
#else
	*value = ((uint16_t)data[1] << 8) | (data[0] & 0xFF);
#endif
}

void Roomba::getSensorValue(uint8_t sensorId, int16_t *value) {
	uint8_t data[2];
	uint32_t readBytes;
	m_AsyncThreadMutex.Lock();
	m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
	m_pTransport->ReceiveData(data, 2, &readBytes);
	m_AsyncThreadMutex.Unlock();
#ifdef __BIG_ENDIAN__
	*value = ((int16_t)data[0] << 8) | (data[1] & 0xFF);
#else
	*value = ((int16_t)data[1] << 8) | (data[0] & 0xFF);
#endif
}


void Roomba::getSensorValue(uint8_t sensorId, uint8_t *value) {
	uint8_t data[2];
	uint32_t readBytes;
	m_AsyncThreadMutex.Lock();
	m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
	m_pTransport->ReceiveData(data, 1, &readBytes);
	m_AsyncThreadMutex.Unlock();
	*value = data[0];
}

void Roomba::getSensorValue(uint8_t sensorId, int8_t *value) {
	uint8_t data[2];
	uint32_t readBytes;
	m_AsyncThreadMutex.Lock();
	m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
	m_pTransport->ReceiveData(data, 1, &readBytes);
	m_AsyncThreadMutex.Unlock();
	*value = data[0];
}

void Roomba::handleStreamData() {

	uint32_t readBytes;
	uint8_t header[2];
	uint8_t bufSize = 0;
	uint32_t sum;

	while(1) {
		m_pTransport->ReceiveData(header, 1, &readBytes);
		if(header[0] == 19) break;
	}
	m_pTransport->ReceiveData(header+1, 1, &readBytes);
	if(header[1] > bufSize) {
		delete buffer;
		bufSize = header[1];
		buffer = new uint8_t[bufSize];
	}
	sum = 19 + header[1];
	m_pTransport->ReceiveData(buffer, header[1], &readBytes);
	if(readBytes != header[1]) {
		std::cout << "Received Packet is wrong." << std::endl;
		delete buffer;
		buffer = NULL;
		m_isStreamMode = 0;
		Exit(-1);
	}
	uint8_t check_sum;
	m_pTransport->ReceiveData(&check_sum, 1, &readBytes);

	for(int i = 0;i < header[1];i++) {
		sum += buffer[i];
	}
	sum += check_sum;

	if((sum & 0xFF) != 0) {
		// CheckSumError;
		return;
	}

	m_AsyncThreadReceiveCounter++;

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


void Roomba::handleBasicData()
{
	uint8_t opcode, buttons;
	uint16_t distance, angle;
	getSensorGroup2(&opcode, &buttons, (int16_t*)&distance, (int16_t*)&angle);
	m_SensorDataMap[ANGLE] = angle;
	m_SensorDataMap[DISTANCE] = distance;

	m_AsyncThreadReceiveCounter++;
}


void Roomba::Run()
{

	m_AsyncThreadReceiveCounter = 0;

	while(m_isStreamMode) {
		if(m_Version != Roomba::VERSION_500_SERIES) {
			handleBasicData();
		} else {
			handleStreamData();
		}

		processOdometry();
		//Sleep(10);
	}

	std::cout << "Exiting Sensor Stream" << std::endl;
	delete buffer;
}

void Roomba::processOdometry(void)
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

void Roomba::move(const double trans, const double rotate) 
{
	double lengthOfShaft = 0.235;
	double distance;
	double angle;
	if(m_Version == Roomba::MODEL_500SERIES) {
		
#define PULSES_TO_METER 0.000445558279992234

		double dR = trans + rotate * lengthOfShaft;
		double dL = trans - rotate * lengthOfShaft;
		if(dR < -1.0) dR = -1.0;
		else if(dR > 1.0) dR = 1.0;
		if(dL < -1.0) dL = -1.0;
		else if(dL > 1.0) dL = 1.0;
		Roomba::driveDirect(dR * 10000, dL * 10000);
	} else {
		double dR = trans + rotate * lengthOfShaft;
		double dL = trans - rotate * lengthOfShaft;
		if(dR < -1.0) dR = -1.0;
		else if(dR > 1.0) dR = 1.0;
		if(dL < -1.0) dL = -1.0;
		else if(dL > 1.0) dL = 1.0;
		Roomba::driveDirect(dR * 10000, dL * 10000);
	}
}


void Roomba::waitPacketReceived() {
	uint32_t buf = m_AsyncThreadReceiveCounter;
	while(buf == m_AsyncThreadReceiveCounter) {
		Thread::Sleep(1);
	}
}


void Roomba::runAsync()
{
	if(m_Version == Roomba::VERSION_500_SERIES) {
	uint8_t defaultSensorId[3] = {RIGHT_ENCODER_COUNTS,
		LEFT_ENCODER_COUNTS, BUMPS_AND_WHEEL_DROPS};
	//uint8_t defaultSensorId[3] = {DISTANCE, ANGLE, BUMPS_AND_WHEEL_DROPS};

	uint8_t numSensor = 3;
	this->startSensorStream(defaultSensorId, numSensor);
	} else if(m_Version == Roomba::VERSION_ROI) {
		this->startSensorStream(NULL, 0);
	}
}

void Roomba::RequestSensor(uint8_t sensorId, uint16_t *value) 
{

	if(m_isStreamMode) {
		m_AsyncThreadMutex.Lock();
		std::map<SensorID, uint16_t>::const_iterator it = m_SensorDataMap.find((SensorID)sensorId);
		if(it != m_SensorDataMap.end()) {
			*value = 0;
			*value |= (*it).second;
			m_AsyncThreadMutex.Unlock();
			return;
		} else {
			if(m_Version == Roomba::VERSION_500_SERIES) {
				m_SensorDataMap[(SensorID)sensorId] = 0;
				m_AsyncThreadMutex.Unlock();
				waitPacketReceived();
				*value = 0;
				m_AsyncThreadMutex.Lock();
				*value |= m_SensorDataMap[(SensorID)sensorId];
				m_AsyncThreadMutex.Unlock();
			} else {
				*value = 0;
			}
			return;
		}
	}

	/*
	if(m_Version != Roomba::VERSION_500_SERIES) {
		*value = 0;
		return;
	}
	*/

	getSensorValue(sensorId, value);
}

void Roomba::RequestSensor(uint8_t sensorId, int16_t *value)
{
	if(m_isStreamMode) {
		m_AsyncThreadMutex.Lock();
		std::map<SensorID, uint16_t>::const_iterator it = m_SensorDataMap.find((SensorID)sensorId);
		if(it != m_SensorDataMap.end()) {
			*value = 0;
			*value |= (*it).second;
			m_AsyncThreadMutex.Unlock();
			return;
		} else {
			if(m_Version == Roomba::MODEL_500SERIES) {
				m_SensorDataMap[(SensorID)sensorId] = 0;
				m_AsyncThreadMutex.Unlock();
				waitPacketReceived();
				*value = 0;
				m_AsyncThreadMutex.Lock();
				*value |= m_SensorDataMap[(SensorID)sensorId];
				m_AsyncThreadMutex.Unlock();
			} else {
				*value = 0;
			}
			return;
		}
	}

	/*
	if(m_Version != Roomba::VERSION_500_SERIES) {
		*value = 0;
		return;
	}
	*/

	getSensorValue(sensorId, value);
}

void Roomba::RequestSensor(uint8_t sensorId, char *value)
{


	if(m_isStreamMode) {
		m_AsyncThreadMutex.Lock();
		std::map<SensorID, uint16_t>::const_iterator it = m_SensorDataMap.find((SensorID)sensorId);
		if(it != m_SensorDataMap.end()) {
			*value = 0;
			*value |= (*it).second;
			m_AsyncThreadMutex.Unlock();
			return;
		} else {
			if(m_Version == Roomba::MODEL_500SERIES) {
				m_SensorDataMap[(SensorID)sensorId] = 0;
				m_AsyncThreadMutex.Unlock();
				waitPacketReceived();
				*value = 0;
				m_AsyncThreadMutex.Lock();
				*value |= m_SensorDataMap[(SensorID)sensorId];
				m_AsyncThreadMutex.Unlock();
			} else {
				*value = 0;
			}
			return;
		}
	}

	/*
	if(m_Version != Roomba::VERSION_500_SERIES) {
		*value = 0;
		return;
	}
	*/

	getSensorValue(sensorId, value);
}

void Roomba::RequestSensor(uint8_t sensorId, uint8_t *value)
{

	if(m_isStreamMode) {
		m_AsyncThreadMutex.Lock();
		std::map<SensorID, uint16_t>::const_iterator it = m_SensorDataMap.find((SensorID)sensorId);
		if(it != m_SensorDataMap.end()) {
			*value = 0;
			*value |= (*it).second;
			m_AsyncThreadMutex.Unlock();
			return;
		} else {
			if(m_Version == Roomba::MODEL_500SERIES) {
				m_SensorDataMap[(SensorID)sensorId] = 0;
				m_AsyncThreadMutex.Unlock();
				waitPacketReceived();
				*value = 0;
				m_AsyncThreadMutex.Lock();
				*value |= m_SensorDataMap[(SensorID)sensorId];
				m_AsyncThreadMutex.Unlock();
			} else {
				*value = 0;
			}
			return;
		}
	}

	/*
	if(m_Version != Roomba::VERSION_500_SERIES) {
		*value = 0;
		return;
	}
	*/

	getSensorValue(sensorId, value);
}


bool Roomba::isRightWheelDropped() {
	uint8_t buf;
	RequestSensor(BUMPS_AND_WHEEL_DROPS, &buf);
	return buf & 0x04 ? true : false;
}

bool Roomba::isLeftWheelDropped() {
	uint8_t buf;
	RequestSensor(BUMPS_AND_WHEEL_DROPS, &buf);
	return buf & 0x08 ? true : false;
}

bool Roomba::isRightBump() {
	uint8_t buf;
	RequestSensor(BUMPS_AND_WHEEL_DROPS, &buf);
	return buf & 0x01 ? true : false;
}

bool Roomba::isLeftBump() 
{
	uint8_t buf;
	RequestSensor(BUMPS_AND_WHEEL_DROPS, &buf);
	return buf & 0x02 ? true : false;
}

bool Roomba::isCliffLeft()
{
	uint8_t buf;
	RequestSensor(CLIFF_LEFT, &buf);
	return buf > 0 ? true : false;
}

bool Roomba::isCliffFrontLeft() {
	uint8_t buf;
	RequestSensor(CLIFF_FRONT_LEFT, &buf);
	return buf > 0 ? true : false;
}

bool Roomba::isCliffFrontRight() {
	uint8_t buf;
	RequestSensor(CLIFF_FRONT_RIGHT, &buf);
	return buf > 0 ? true : false;
}

bool Roomba::isCliffRight() {
	uint8_t buf;
	RequestSensor(CLIFF_RIGHT, &buf);
	return buf > 0 ? true : false;
}

bool Roomba::isVirtualWall()
{
	uint8_t buf;
	RequestSensor(VIRTUAL_WALL, &buf);
	return buf > 0 ? true : false;
}

Roomba::MotorFlag Roomba::isWheelOvercurrents()
{
	uint8_t buf;
	RequestSensor(WHEEL_OVERCURRENTS, &buf);
	return (MotorFlag)buf;
}

bool Roomba::isRightWheelOvercurrent() 
{
	return isWheelOvercurrents() & RightWheel ? true : false;
}

bool Roomba::isLeftWheelOvercurrent() 
{
	return isWheelOvercurrents() & LeftWheel ? true : false;
}

bool Roomba::isMainBrushOvercurrent() 
{
	return isWheelOvercurrents() & MainBrush ? true : false;
}

bool Roomba::isSideBrushOvercurrent() 
{
	return isWheelOvercurrents() & SideBrush ? true : false;
}

bool Roomba::dirtDetect()
{
	uint8_t buf;
	RequestSensor(DIRT_DETECT, &buf);
	return buf ? true : false;
}

int8_t Roomba::getInfraredCharacterOmni()
{
	int8_t buf;
	RequestSensor(INFRARED_CHARACTER_OMNI, &buf);
	return buf;
}

int8_t Roomba::getInfraredCharacterRight()
{
	int8_t buf;
	RequestSensor(INFRARED_CHARACTER_RIGHT, &buf);
	return buf;
}

int8_t Roomba::getInfraredCharacterLeft()
{
	int8_t buf;
	RequestSensor(INFRARED_CHARACTER_LEFT, &buf);
	return buf;
}


Roomba::ButtonFlag Roomba::getButtons()
{
	uint8_t buf;
	RequestSensor(BUTTONS, &buf);
	return (ButtonFlag)buf;
}

int16_t Roomba::getDistance()
{
	int16_t buf;
	RequestSensor(DISTANCE, &buf);
	return buf;
}

int16_t Roomba::getAngle()
{
	int16_t buf;
	RequestSensor(ANGLE, &buf);
	return buf;
}

Roomba::ChargingState Roomba::getChargingState()
{
	uint8_t buf;
	RequestSensor(CHARGING_STATE, &buf);
	return (ChargingState)buf;
}

uint16_t Roomba::getVoltage() {
	uint16_t buf;
	RequestSensor(VOLTAGE, &buf);
	return buf;
}

uint16_t Roomba::getCurrent() {
	uint16_t buf;
	RequestSensor(VOLTAGE, &buf);
	return buf;
}

int8_t Roomba::getTemperature() {
	int8_t buf;
	RequestSensor(TEMPERATURE, &buf);
	return buf;
}

Roomba::Mode Roomba::getOIMode()
{
	uint8_t buf;
	RequestSensor(OI_MODE, &buf);
	if(buf== 0) 
		return MODE_OFF;
	else if(buf == 1)
		return MODE_PASSIVE;
	else if(buf == 2)
		return MODE_SAFE;
	else if(buf == 3)
		return MODE_FULL;

	return MODE_OFF;
}


int16_t Roomba::getRequestedVelocity() {
	int16_t buf;
	RequestSensor(REQUESTED_VELOCITY, &buf);
	return buf;
}


int16_t Roomba::getRequestedRadius() {
	int16_t buf;
	RequestSensor(REQUESTED_RADIUS, &buf);
	return buf;
}


uint16_t Roomba::getRightEncoderCounts()
{
	uint16_t buf;
	RequestSensor(LEFT_ENCODER_COUNTS, &buf);
	return buf;
}

uint16_t Roomba::getLeftEncoderCounts()
{
	uint16_t buf;
	RequestSensor(RIGHT_ENCODER_COUNTS, &buf);
	return buf;
}
