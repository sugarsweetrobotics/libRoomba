
#include "Roomba.h"
#include <iostream>


using namespace net::ysuga::roomba;

Roomba::Roomba(const char *portName, const int baudrate)
{
	m_pTransport = new Transport(portName, baudrate);
	m_pTransport->SendPacket(OP_START);
	m_CurrentMode = PASSIVE;

	SetMode(SAFE);
}

Roomba::~Roomba(void)
{
	delete m_pTransport;
}

void Roomba::SetMode(Mode mode)
{
	switch(mode) {
	case SAFE:
		m_pTransport->SendPacket(OP_SAFE);
		m_CurrentMode = SAFE;
		break;

	case FULL:
		m_pTransport->SendPacket(OP_FULL);
		m_CurrentMode = FULL;
		break;
		
	case SPOT_CLEAN:
		m_pTransport->SendPacket(OP_SPOT);
		m_CurrentMode = PASSIVE;
		break;

	case NORMAL_CLEAN:
		m_pTransport->SendPacket(OP_CLEAN);
		m_CurrentMode = PASSIVE;

		break;

	case MAX_TIME_CLEAN:
		m_pTransport->SendPacket(OP_MAX);
		m_CurrentMode = PASSIVE;
		break;

	case DOCK:
		m_pTransport->SendPacket(OP_DOCK);
		m_CurrentMode = PASSIVE;
		break;


	case POWER_DOWN:
		m_pTransport->SendPacket(OP_POWER);
		m_CurrentMode = PASSIVE;
		break;

	default:
		break;
	}
}

void Roomba::Drive(unsigned short translation, unsigned short turnRadius) {
	if(GetMode() != SAFE && GetMode() != FULL) {
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

void Roomba::DriveDirect(unsigned short rightWheel, unsigned short leftWheel) {
	if(GetMode() != SAFE && GetMode() != FULL) {
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


void Roomba::SetLED(unsigned char leds, unsigned char intensity, unsigned char color /* = 127*/) 
{
	unsigned char buf[3] = {leds, color, intensity};
	m_pTransport->SendPacket(OP_LEDS, buf, 3);
}

void Roomba::RequestSensor(unsigned char sensorId, unsigned short *value)
{
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

void Roomba::RequestSensor(unsigned char sensorId, short *value)
{
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

void Roomba::RequestSensor(unsigned char sensorId, char *value)
{
	char data[1];
	unsigned int readBytes;
	m_pTransport->SendPacket(OP_SENSORS, &sensorId, 1);
	m_pTransport->ReceiveData((unsigned char*)data, 1, &readBytes);
	memcpy(value, data, 1);
}
