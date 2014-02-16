#include "RoombaImpl.h"
#include <iostream>
#include "op_code.h"

using namespace ssr;

RoombaImpl::RoombaImpl(const uint32_t model, const char *portName, const uint32_t baudrate) :
  m_TargetVelocityX(0), m_TargetVelocityTh(0), 
  m_Transport(portName, baudrate), 
  m_Protocol(this, &m_Transport, &m_Odometry, model==MODEL_CREATE ? VERSION_ROI : VERSION_500_SERIES),
  m_AsyncThread(false)
{
  //m_Protocol.Start();
  //start();
	m_Protocol.suspendSensorStream();
}


RoombaImpl::~RoombaImpl(void)
{
  if(m_AsyncThread) {
    m_AsyncThread = false;
    Join();
  }
  safeControl();
  //start();
}

void RoombaImpl::setMode(Mode mode)
{
  switch(mode) {
  case MODE_START:
    m_Transport.SendPacket(OP_START);
    m_CurrentMode = MODE_PASSIVE;
    break;	
    
  case MODE_SAFE:
    m_Transport.SendPacket(OP_SAFE);
    m_CurrentMode = MODE_SAFE;
    break;
    
  case MODE_FULL:
    m_Transport.SendPacket(OP_FULL);
    m_CurrentMode = MODE_FULL;
    break;
    
  case MODE_SPOT_CLEAN:
    m_Transport.SendPacket(OP_SPOT);
    m_CurrentMode = MODE_PASSIVE;
    break;
    
  case MODE_NORMAL_CLEAN:
    m_Transport.SendPacket(OP_CLEAN);
    m_CurrentMode = MODE_PASSIVE;
    break;
    
  case MODE_MAX_TIME_CLEAN:
    m_Transport.SendPacket(OP_MAX);
    m_CurrentMode = MODE_PASSIVE;
    break;
    
  case MODE_DOCK:
    m_Transport.SendPacket(OP_DOCK);
    m_CurrentMode = MODE_PASSIVE;
    break;
    
  case MODE_POWER_DOWN:
    m_Transport.SendPacket(OP_POWER);
    m_CurrentMode = MODE_PASSIVE;
    break;
    
  default:
    break;
  }
  
  //Thread::Sleep(100);
}

void RoombaImpl::drive(int16_t translation, int16_t turnRadius) {
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
  m_Transport.SendPacket(OP_DRIVE, data, 4);
}

void RoombaImpl::driveDirect(int16_t rightWheel, int16_t leftWheel) {
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
  m_Transport.SendPacket(OP_DRIVE_DIRECT, data, 4);
}

void RoombaImpl::drivePWM(int16_t rightWheel, int16_t leftWheel) {
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
  m_Transport.SendPacket(OP_DRIVE_PWM, data, 4);
}

void RoombaImpl::driveMotors(Motors mainBrush, Motors sideBrush, Motors vacuum)
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
  
  m_Transport.SendPacket(OP_MOTORS, &data, 1);
}



void RoombaImpl::setLED(uint8_t leds, uint8_t intensity, uint8_t color /* = 127*/) 
{
  uint8_t buf[3] = {leds, color, intensity};
  m_Transport.SendPacket(OP_LEDS, buf, 3);
}


void RoombaImpl::Run()
{
  m_AsyncThread = true;
  //m_rightPWM = m_leftPWM = 0;
  //double epsilon = 0.0001;
  //double m_accelPWM = 1;
  while(m_AsyncThread) {
		Thread::Sleep(10);
		double lengthOfShaft = 0.235;

		//Velocity v = Roomba::getCurrentVelocity();
		//double cR = v.x + v.th * lengthOfShaft;
		//double cL = v.x - v.th * lengthOfShaft;
		double dR = m_TargetVelocityX + m_TargetVelocityTh * lengthOfShaft;
		double dL = m_TargetVelocityX - m_TargetVelocityTh * lengthOfShaft;

		if (dR > 0.5) dR = 0.5;
		else if (dR < -0.5) dR = -0.5;
		if (dL > 0.5) dL = 0.5;
		else if (dL < -0.5) dL = -0.5;

		try {
			this->driveDirect(dR*1000, dL * 1000);
		} catch (PreconditionNotMetError &e) {
		}
   // std::cout << "Async Thread" << std::endl;
  }


  /*
	m_AsyncThreadReceiveCounter = 0;

	while(m_isStreamMode) {
	  Thread::Sleep(100);
		if(m_Version != RoombaImpl::VERSION_500_SERIES) {
			handleBasicData();
		} else {
			handleStreamData();
		}

		processOdometry();
	}

	std::cout << "Exiting Sensor Stream" << std::endl;
	delete buffer;
  */
}


void RoombaImpl::setTargetVelocity(const double trans, const double rotate) 
{
	double lengthOfShaft = 0.235;
	double distance;
	double angle;
	m_TargetVelocityX = trans;
	m_TargetVelocityTh = rotate;
	/*
	if(m_Version == RoombaImpl::MODEL_500SERIES) {
		
#define PULSES_TO_METER 0.000445558279992234

		double dR = trans + rotate * lengthOfShaft;
		double dL = trans - rotate * lengthOfShaft;
		if(dR < -1.0) dR = -1.0;
		else if(dR > 1.0) dR = 1.0;
		if(dL < -1.0) dL = -1.0;
		else if(dL > 1.0) dL = 1.0;
		RoombaImpl::driveDirect(dR * 1000, dL * 1000);
	} else {
		double dR = trans + rotate * lengthOfShaft;
		double dL = trans - rotate * lengthOfShaft;
		if(dR < -1.0) dR = -1.0;
		else if(dR > 1.0) dR = 1.0;
		if(dL < -1.0) dL = -1.0;
		else if(dL > 1.0) dL = 1.0;
		RoombaImpl::driveDirect(dR * 1000, dL * 1000);
	}
	*/
}


/*
void RoombaImpl::waitPacketReceived() {
	uint32_t buf = m_AsyncThreadReceiveCounter;
	while(buf == m_AsyncThreadReceiveCounter) {
		Thread::Sleep(1);
	}
}
*/

/*
void RoombaImpl::runAsync()
{
	if(m_Version == RoombaImpl::VERSION_500_SERIES) {
	uint8_t defaultSensorId[3] = {RIGHT_ENCODER_COUNTS,
		LEFT_ENCODER_COUNTS, BUMPS_AND_WHEEL_DROPS};
	//uint8_t defaultSensorId[3] = {DISTANCE, ANGLE, BUMPS_AND_WHEEL_DROPS};

	uint8_t numSensor = 3;
	this->startSensorStream(defaultSensorId, numSensor);
	} else if(m_Version == RoombaImpl::VERSION_ROI) {
		this->startSensorStream(NULL, 0);
	}
}
*/
