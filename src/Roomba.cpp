
#include "Roomba.h"
#include <iostream>


#include "op_code.h"

using namespace net::ysuga::roomba;

Roomba::Roomba(const char *portName, const int baudrate) :
m_isStreamMode(0)
{
	m_pTransport = new Transport(portName, baudrate);
	m_pTransport->SendPacket(OP_START);
	m_CurrentMode = MODE_PASSIVE;

	setMode(MODE_SAFE);
}


Roomba::~Roomba(void)
{
	setMode(POWER_DOWN);
	if(m_isStreamMode) {
		suspendSensorStream();
		m_isStreamMode = false;
		Join();
	}
	delete m_pTransport;
}

void Roomba::setMode(Mode mode)
{
	switch(mode) {
	case MODE_SAFE:
		m_pTransport->SendPacket(OP_SAFE);
		m_CurrentMode = MODE_SAFE;
		break;

	case MODE_FULL:
		m_pTransport->SendPacket(OP_FULL);
		m_CurrentMode = MODE_FULL;
		break;
		
	case SPOT_CLEAN:
		m_pTransport->SendPacket(OP_SPOT);
		m_CurrentMode = MODE_PASSIVE;
		break;

	case NORMAL_CLEAN:
		m_pTransport->SendPacket(OP_CLEAN);
		m_CurrentMode = MODE_PASSIVE;

		break;

	case MAX_TIME_CLEAN:
		m_pTransport->SendPacket(OP_MAX);
		m_CurrentMode = MODE_PASSIVE;
		break;

	case DOCK:
		m_pTransport->SendPacket(OP_DOCK);
		m_CurrentMode = MODE_PASSIVE;
		break;

	case POWER_DOWN:
		m_pTransport->SendPacket(OP_POWER);
		m_CurrentMode = MODE_PASSIVE;
		break;

	default:
		break;
	}
}

void Roomba::drive(unsigned short translation, unsigned short turnRadius) {
	if(getMode() != MODE_SAFE && getMode() != MODE_FULL) {
		throw PreconditionNotMetError();
	}

	unsigned char data[4];
#ifdef __BIG_ENDIAN__
	data[0] = (translation >> 8) & 0xFF;
	data[1] = translation & 0xFF;
	data[2] = (turnRadius >> 8) & 0xFF;
	data[3] = turnRadius & 0xFF;
#else
	data[0] = translation & 0xFF;
	data[1] = (translation >> 8) & 0xFF;
	data[2] = turnRadius & 0xFF;
	data[3] = (turnRadius >> 8) & 0xFF;
	

#endif
	m_pTransport->SendPacket(OP_DRIVE, data, 4);
}

void Roomba::driveDirect(unsigned short rightWheel, unsigned short leftWheel) {
	if(getMode() != MODE_SAFE && getMode() !=MODE_FULL) {
		throw PreconditionNotMetError();
	}

	unsigned char data[4];
#ifdef __BIG_ENDIAN__
	data[0] = (rightWheel >> 8) & 0xFF;
	data[1] = rightWheel & 0xFF;
	data[2] = (leftWheel >> 8) & 0xFF;
	data[3] = leftWheel & 0xFF;
#else
	data[0] = rightWheel & 0xFF;
	data[1] = (rightWheel >> 8) & 0xFF;
	data[2] = leftWheel & 0xFF;
	data[3] = (leftWheel >> 8) & 0xFF;

#endif
	m_pTransport->SendPacket(OP_DRIVE, data, 4);
}

void Roomba::drivePWM(unsigned short rightWheel, unsigned short leftWheel) {
	if(getMode() != MODE_SAFE && getMode() != MODE_FULL) {
		throw PreconditionNotMetError();
	}

	unsigned char data[4];
#ifdef __BIG_ENDIAN__
	data[0] = (rightWheel >> 8) & 0xFF;
	data[1] = rightWheel & 0xFF;
	data[2] = (leftWheel >> 8) & 0xFF;
	data[3] = leftWheel & 0xFF;
#else
	data[0] = rightWheel & 0xFF;
	data[1] = (rightWheel >> 8) & 0xFF;
	data[2] = leftWheel & 0xFF;
	data[3] = (leftWheel >> 8) & 0xFF;

#endif
	m_pTransport->SendPacket(OP_DRIVE_PWM, data, 4);
}

LIBROOMBA_API void Roomba::driveMotors(Motors mainBrush, Motors sideBrush, Motors vacuum)
{
	if(getMode() != MODE_SAFE && getMode() !=MODE_FULL) {
		throw PreconditionNotMetError();
	}

	unsigned char data = 0;
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



void Roomba::setLED(unsigned char leds, unsigned char intensity, unsigned char color /* = 127*/) 
{
	unsigned char buf[3] = {leds, color, intensity};
	m_pTransport->SendPacket(OP_LEDS, buf, 3);
}


				
void Roomba::startSensorStream(unsigned char* requestingSensors, int numSensors)
{
	m_SensorDataMap.clear();
	unsigned char* buffer = new unsigned char[numSensors + 1];
	buffer[0] = numSensors;
	for(int i = 0;i < numSensors;i++) {
		m_SensorDataMap[(SensorID)requestingSensors[i]] = 0;
		buffer[i+1] = requestingSensors[i];
	}
	m_pTransport->SendPacket(OP_STREAM, buffer, numSensors+1);
	
	m_isStreamMode = true;
	resumeSensorStream();
	Start();
	delete buffer;
}

void Roomba::resumeSensorStream()
{
	unsigned char buf = 1;
	m_pTransport->SendPacket(OP_PAUSE_RESUME_STREAM, &buf, 1);
}

void Roomba::suspendSensorStream()
{
	unsigned char buf = 0;
	m_pTransport->SendPacket(OP_PAUSE_RESUME_STREAM, &buf, 1);
}


void Roomba::Run()
{
	unsigned int readBytes;
	unsigned char header[2];
	unsigned char* buffer = NULL;
	unsigned char bufSize = 0;
	unsigned long sum;

	while(m_isStreamMode) {
		//Thread::Sleep(10);
		while(1) {
			m_pTransport->ReceiveData(header, 1, &readBytes);
			if(header[0] == 19) break;
		}
		m_pTransport->ReceiveData(header+1, 1, &readBytes);
		if(header[1] > bufSize) {
			delete buffer;
			bufSize = header[1];
			buffer = new unsigned char[bufSize];
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
		unsigned char check_sum;
		m_pTransport->ReceiveData(&check_sum, 1, &readBytes);

		for(int i = 0;i < header[1];i++) {
			sum += buffer[i];
		}
		sum += check_sum;

		if((sum & 0xFF) != 0) {
			// CheckSumError;
			continue;
		}

		int counter = 0;
		do {
			unsigned char sensorId = buffer[counter];
			unsigned short dataBuf = 0;
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
				m_SensorDataMap[(SensorID)sensorId] = dataBuf;
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
#ifdef BIG_ENDIAN
				dataBuf |= buffer[counter];
				counter++;
				dataBuf <<= 8;
				dataBuf |= buffer[counter];
				counter++;
#else 
				dataBuf |= buffer[counter];
				counter++;
				dataBuf |= ((unsigned short)buffer[counter] << 8);
				counter++;
#endif
				m_SensorDataMap[(SensorID)sensorId] = dataBuf;
				break;
			default:
				break;
			}
		} while(counter < header[1]-1);
	}
	std::cout << "Exiting Sensor Stream" << std::endl;
	delete buffer;
}

void Roomba::runAsync()
{
	unsigned char defaultSensorId[2] = {RIGHT_ENCODER_COUNTS,
		LEFT_ENCODER_COUNTS,};
	unsigned char numSensor = 2;
	this->startSensorStream(defaultSensorId, numSensor);
}

void Roomba::RequestSensor(unsigned char sensorId, unsigned short *value) const
{
	std::map<SensorID, unsigned short>::const_iterator it = m_SensorDataMap.find((SensorID)sensorId);
	if(it != m_SensorDataMap.end()) {
		unsigned short buf = (*it).second;
		*value = 0;
		*value |= buf;
		return;
	}

	unsigned char data[2];
	unsigned int readBytes;
	m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
	m_pTransport->ReceiveData(data, 2, &readBytes);
#ifdef __BIG_ENDIAN__
	*value = ((signed short)data[0] << 8) | (data[1] & 0xFF);
#else
	*value = ((signed short)data[1] << 8) | (data[0] & 0xFF);
#endif
}

void Roomba::RequestSensor(unsigned char sensorId, short *value) const
{

	std::map<SensorID, unsigned short>::const_iterator it = this->m_SensorDataMap.find((SensorID)sensorId);
	if(it != m_SensorDataMap.end()) {
		unsigned short buf = (*it).second;
		*value = 0;
		*value |= buf;
		return;
	}

	unsigned char data[2];
	unsigned int readBytes;
	m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
	m_pTransport->ReceiveData(data, 2, &readBytes);
#ifdef __BIG_ENDIAN__
	*value = ((signed short)data[0] << 8) | (data[1] & 0xFF);
#else
	*value = ((signed short)data[1] << 8) | (data[0] & 0xFF);
#endif
}

void Roomba::RequestSensor(unsigned char sensorId, char *value) const
{

	std::map<SensorID, unsigned short>::const_iterator it = this->m_SensorDataMap.find((SensorID)sensorId);
	if(it != m_SensorDataMap.end()) {
		unsigned short buf = (*it).second;
		*value = 0;
		*value |= (char)buf;
		return;
	}

	char data[1];
	unsigned int readBytes;
	m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
	m_pTransport->ReceiveData((unsigned char*)data, 1, &readBytes);
	memcpy(value, data, 1);
}

void Roomba::RequestSensor(unsigned char sensorId, unsigned char *value) const
{

	std::map<SensorID, unsigned short>::const_iterator it = this->m_SensorDataMap.find((SensorID)sensorId);
	if(it != m_SensorDataMap.end()) {
		unsigned short buf = (*it).second;
		*value = 0;
		*value |= (unsigned char)buf;
		return;
	}

	unsigned char data[1];
	unsigned int readBytes;
	m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
	m_pTransport->ReceiveData((unsigned char*)data, 1, &readBytes);
	memcpy(value, data, 1);
}


int Roomba::isRightWheelDropped() const {
	unsigned char buf;
	RequestSensor(BUMPS_AND_WHEEL_DROPS, &buf);
	return buf & 0x04 ? true : false;
}

int Roomba::isLeftWheelDropped() const {
	unsigned char buf;
	RequestSensor(BUMPS_AND_WHEEL_DROPS, &buf);
	return buf & 0x08 ? true : false;
}

int Roomba::isRightBump() const {
	unsigned char buf;
	RequestSensor(BUMPS_AND_WHEEL_DROPS, &buf);
	return buf & 0x01 ? true : false;
}

int Roomba::isLeftBump() const 
{
	unsigned char buf;
	RequestSensor(BUMPS_AND_WHEEL_DROPS, &buf);
	return buf & 0x02 ? true : false;
}

int Roomba::isCliffLeft() const
{
	unsigned char buf;
	RequestSensor(CLIFF_LEFT, &buf);
	return buf;
}

int Roomba::isCliffFrontLeft() const {
	unsigned char buf;
	RequestSensor(CLIFF_FRONT_LEFT, &buf);
	return buf;
}

int Roomba::isCliffFrontRight() const {
	unsigned char buf;
	RequestSensor(CLIFF_FRONT_RIGHT, &buf);
	return buf;
}

int Roomba::isCliffRight() const {
	unsigned char buf;
	RequestSensor(CLIFF_RIGHT, &buf);
	return buf;
}



int Roomba::isVirtualWall() const
{
	unsigned char buf;
	RequestSensor(VIRTUAL_WALL, &buf);
	return buf;
}

MotorFlag Roomba::isWheelOvercurrents() const
{
	unsigned char buf;
	RequestSensor(WHEEL_OVERCURRENTS, &buf);
	return (MotorFlag)buf;
}

int Roomba::isRightWheelOvercurrent() const 
{
	return isWheelOvercurrents() & RightWheel ? true : false;
}

int Roomba::isLeftWheelOvercurrent() const 
{
	return isWheelOvercurrents() & LeftWheel ? true : false;
}

int Roomba::isMainBrushOvercurrent() const 
{
	return isWheelOvercurrents() & MainBrush ? true : false;
}

int Roomba::isSideBrushOvercurrent() const 
{
	return isWheelOvercurrents() & SideBrush ? true : false;
}

int Roomba::dirtDetect() const
{
	unsigned char buf;
	RequestSensor(DIRT_DETECT, &buf);
	return buf;
}

unsigned char Roomba::getInfraredCharacterOmni() const
{
	unsigned char buf;
	RequestSensor(INFRARED_CHARACTER_OMNI, &buf);
	return buf;
}

unsigned char Roomba::getInfraredCharacterRight() const
{
	unsigned char buf;
	RequestSensor(INFRARED_CHARACTER_RIGHT, &buf);
	return buf;
}

unsigned char Roomba::getInfraredCharacterLeft() const
{
	unsigned char buf;
	RequestSensor(INFRARED_CHARACTER_LEFT, &buf);
	return buf;
}


ButtonFlag Roomba::getButtons() const
{
	unsigned char buf;
	RequestSensor(BUTTONS, &buf);
	return (ButtonFlag)buf;
}

int Roomba::getDistance() const
{
	signed short buf;
	RequestSensor(DISTANCE, &buf);
	return buf;
}

int Roomba::getAngle() const
{
	signed short buf;
	RequestSensor(ANGLE, &buf);
	return buf;
}

ChargingState Roomba::getChargingState() const
{
	unsigned char buf;
	RequestSensor(CHARGING_STATE, &buf);
	return (ChargingState)buf;
}

int Roomba::getVoltage() const {
	unsigned short buf;
	RequestSensor(VOLTAGE, &buf);
	return buf;
}

int Roomba::getCurrent() const {
	unsigned short buf;
	RequestSensor(VOLTAGE, &buf);
	return buf;
}

int Roomba::getTemperature()  const {
	char buf;
	RequestSensor(TEMPERATURE, &buf);
	return buf;
}

Mode Roomba::getOIMode() const
{
	unsigned char buf;
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


int Roomba::getRequestedVelocity() const {
	short buf;
	RequestSensor(REQUESTED_VELOCITY, &buf);
	return buf;
}


int Roomba::getRequestedRadius() const {
	short buf;
	RequestSensor(REQUESTED_RADIUS, &buf);
	return buf;
}


unsigned short Roomba::getRightEncoderCounts() const
{
	unsigned short buf;
	RequestSensor(LEFT_ENCODER_COUNTS, &buf);
	return buf;
}

unsigned short Roomba::getLeftEncoderCounts() const
{
	unsigned short buf;
	RequestSensor(RIGHT_ENCODER_COUNTS, &buf);
	return buf;
}