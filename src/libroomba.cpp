#include "libroomba.h"
#include "Roomba.h"

#include <iostream>


using namespace net::ysuga::roomba;


static int g_RoombaCounter;
static Roomba* g_pRoomba[MAX_ROOMBA] = {NULL, };


LIBROOMBA_API int Roomba_create(const uint32_t model, const char* portname, const uint32_t baudrate)
{
	g_pRoomba[g_RoombaCounter] = new Roomba(model, portname, baudrate);
	g_RoombaCounter++;
	return g_RoombaCounter-1;
}

LIBROOMBA_API int Roomba_destroy(const int hRoomba)
{
	delete g_pRoomba[hRoomba];
	g_pRoomba[hRoomba] = NULL;
	return 0;
}


LIBROOMBA_API int Roomba_runAsync(const int hRoomba)
{
	g_pRoomba[hRoomba]->runAsync();
	return 0;
}


LIBROOMBA_API int Roomba_setMode(const int hRoomba, const int mode)
{
	g_pRoomba[hRoomba]->setMode((Roomba::Mode)mode);
	return 0;
}

LIBROOMBA_API int Roomba_getMode(const int hRoomba, int *mode)
{
	*mode = g_pRoomba[hRoomba]->getMode();
	return 0;
}

LIBROOMBA_API void Roomba_start(const int hRoomba)
{
	g_pRoomba[hRoomba]->start();
}

LIBROOMBA_API void Roomba_clean(const int hRoomba)
{
	g_pRoomba[hRoomba]->clean();
}

LIBROOMBA_API void Roomba_spotClean(const int hRoomba)
{
	g_pRoomba[hRoomba]->spotClean();
}

LIBROOMBA_API void Roomba_maxClean(const int hRoomba)
{
	g_pRoomba[hRoomba]->maxClean();
}

LIBROOMBA_API void Roomba_dock(const int hRoomba)
{
	g_pRoomba[hRoomba]->dock();
}

LIBROOMBA_API void Roomba_powerDown(const int hRoomba)
{
	g_pRoomba[hRoomba]->powerDown();
}

LIBROOMBA_API void Roomba_safeControl(const int hRoomba)
{
	g_pRoomba[hRoomba]->safeControl();
}

LIBROOMBA_API void Roomba_fullControl(const int hRoomba)
{
	g_pRoomba[hRoomba]->fullControl();
}

LIBROOMBA_API int Roomba_drive(const int hRoomba, const short translationVelocity, const short turnRadius)
{
	try {
		g_pRoomba[hRoomba]->drive(translationVelocity, turnRadius);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << " " << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_driveDirect(const int hRoomba, const short rightWheelVelocity, const short leftWheelVelocity)
{
	try {
		g_pRoomba[hRoomba]->driveDirect(rightWheelVelocity, leftWheelVelocity);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << " " << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_drivePWM(const int hRoomba, const short rightWheel, const short leftWheel)
{
	try {
		g_pRoomba[hRoomba]->drivePWM(rightWheel, leftWheel);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << " " << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_driveMotors(const int hRoomba, const int mainBrush, const int sideBrush, const int vacuum)
{
	try {
		g_pRoomba[hRoomba]->driveMotors((Roomba::Motors)mainBrush, (Roomba::Motors)sideBrush, (Roomba::Motors)vacuum);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << " " << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_driveMainBrsuh(const int hRoomba, const int flag)
{
	try {
		g_pRoomba[hRoomba]->driveMainBrush((Roomba::Motors)flag);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << " " << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_driveSideBrsuh(const int hRoomba, const int flag)
{
	try {
		g_pRoomba[hRoomba]->driveSideBrush((Roomba::Motors)flag);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << " " << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_driveVacuum(const int hRoomba, const int flag)
{
	try {
		g_pRoomba[hRoomba]->driveVacuum((Roomba::Motors)flag);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << " " << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}


LIBROOMBA_API int Roomba_setLED(const int hRoomba, unsigned char leds, unsigned char intensity, unsigned char color)
{
	try {
		g_pRoomba[hRoomba]->setLED(leds, intensity, color);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_drive():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_setDockLED(const int hRoomba, const int flag)
{	
	try {
		g_pRoomba[hRoomba]->setDockLED(flag);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_" << __FUNCTION__ << ":" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;	
}
LIBROOMBA_API int Roomba_setRobotLED(const int hRoomba, const int flag)
{	
	try {
		g_pRoomba[hRoomba]->setRobotLED(flag);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_" << __FUNCTION__ << ":" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;	
}

LIBROOMBA_API int Roomba_setDebrisLED(const int hRoomba, const int flag)
{	
	try {
		g_pRoomba[hRoomba]->setDebrisLED(flag);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_" << __FUNCTION__ << ":" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;	
}

LIBROOMBA_API int Roomba_setSpotLED(const int hRoomba, const int flag)
{	
	try {
		g_pRoomba[hRoomba]->setSpotLED(flag);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_" << __FUNCTION__ << ":" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;	
}

LIBROOMBA_API int Roomba_setCleanLEDIntensity(const int hRoomba, const unsigned char intensity)
{	
	try {
		g_pRoomba[hRoomba]->setCleanLEDIntensity(intensity);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_" << __FUNCTION__ << ":" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;	
}

LIBROOMBA_API int Roomba_setCleanLEDColor(const int hRoomba, const unsigned char color)
{	
	try {
		g_pRoomba[hRoomba]->setCleanLEDColor(color);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_" << __FUNCTION__ << ":" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;	
}


LIBROOMBA_API int Roomba_isRightWheelDropped(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isRightWheelDropped();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_isRightWheelDropped():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}


LIBROOMBA_API int Roomba_isLeftWheelDropped(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isLeftWheelDropped();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_isLeftWheelDropped():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}


LIBROOMBA_API int Roomba_isRightBump(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isRightBump();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_isRightBump():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}


LIBROOMBA_API int Roomba_isLeftBump(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isLeftBump();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_isLeftBump():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}


LIBROOMBA_API int Roomba_isCliffLeft(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isCliffLeft();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_isCliffLeft():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_isCliffFrontLeft(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isCliffFrontLeft();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_isCliffFrontLeft():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_isCliffFrontRight(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isCliffFrontRight();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_isCliffFrontRight():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_isCliffRight(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isCliffRight();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_isCliffRight():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_isVirtualWall(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isVirtualWall();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_isVirtualWall():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_isWheelOvercurrents(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isWheelOvercurrents();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_isWheelOvercurrents():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_isRightWheelOvercurrent(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isRightWheelOvercurrent();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_isRightWheelOvercurrent():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_isLeftWheelOvercurrent(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isLeftWheelOvercurrent();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_isLeftWheelOvercurrent():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_isMainBrushOvercurrent(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isMainBrushOvercurrent();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}


LIBROOMBA_API int Roomba_isSideBrushOvercurrent(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->isSideBrushOvercurrent();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}


LIBROOMBA_API int Roomba_dirtDetect(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->dirtDetect();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}


LIBROOMBA_API int Roomba_getInfraredCharacterOmni(const int hRoomba, char* ret)
{
	try {
		*ret = g_pRoomba[hRoomba]->getInfraredCharacterOmni();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_getInfraredCharacterRight(const int hRoomba, char* ret)
{
	try {
		*ret = g_pRoomba[hRoomba]->getInfraredCharacterRight();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_getInfraredCharacterLeft(const int hRoomba, char* ret)
{
	try {
		*ret = g_pRoomba[hRoomba]->getInfraredCharacterLeft();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_getButtons(const int hRoomba, int *flag)
{
	try {
		*flag = g_pRoomba[hRoomba]->dirtDetect();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_getDistance(const int hRoomba, int *distance)
{
	try {
		*distance = g_pRoomba[hRoomba]->getDistance();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_getAngle(const int hRoomba, int *angle)
{
	try {
		*angle = g_pRoomba[hRoomba]->getAngle();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}


LIBROOMBA_API int Roomba_getChargingState(const int hRoomba, int *state)
{
	try {
		*state = g_pRoomba[hRoomba]->getChargingState();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_getVoltage(const int hRoomba, int *voltage)
{
	try {
		*voltage = g_pRoomba[hRoomba]->getVoltage();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_getCurrent(const int hRoomba, int *current)
{
	try {
		*current = g_pRoomba[hRoomba]->getCurrent();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_getTemperature(const int hRoomba, int* temperature)
{
	try {
		*temperature = g_pRoomba[hRoomba]->getTemperature();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}



LIBROOMBA_API int Roomba_getOIMode(const int hRoomba, int *mode)
{
	try {
		*mode = g_pRoomba[hRoomba]->getOIMode();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_getRequestedVelocity(const int hRoomba, int *velocity)
{
	try {
		*velocity = g_pRoomba[hRoomba]->getRequestedVelocity();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_getRequestedRadius(const int hRoomba, int* radius)
{
	try {
		*radius = g_pRoomba[hRoomba]->getRequestedRadius();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}


LIBROOMBA_API unsigned short Roomba_getRightEncoderCounts(const int hRoomba, unsigned short *count)
{
	try {
		*count = g_pRoomba[hRoomba]->getRightEncoderCounts();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}


LIBROOMBA_API unsigned short Roomba_getLeftEncoderCounts(const int hRoomba, unsigned short* count)
{
	try {
		*count = g_pRoomba[hRoomba]->getLeftEncoderCounts();
	} catch( PreconditionNotMetError &e) {
		std::cerr << "Error in " << __FUNCTION__ << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}
