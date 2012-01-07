#include "libroomba.h"
#include "Roomba.h"

#include <iostream>


using namespace net::ysuga::roomba;


static int g_RoombaCounter;
static Roomba* g_pRoomba[MAX_ROOMBA] = {NULL, };


LIBROOMBA_API int Roomba_create(const char* portname, const int baudrate)
{
	g_pRoomba[g_RoombaCounter] = new Roomba(portname, baudrate);
	g_RoombaCounter++;
	return g_RoombaCounter-1;
}

LIBROOMBA_API int Roomba_destroy(const int hRoomba)
{
	delete g_pRoomba[g_RoombaCounter];
	g_pRoomba[g_RoombaCounter] = NULL;
	return 0;
}

LIBROOMBA_API int Roomba_setMode(const int hRoomba, const int mode)
{
	g_pRoomba[hRoomba]->setMode((Mode)mode);
	return 0;
}

LIBROOMBA_API int Roomba_getMode(const int hRoomba, int *mode)
{
	*mode = g_pRoomba[hRoomba]->getMode();
	return 0;
}

LIBROOMBA_API int Roomba_drive(const int hRoomba, const short translationVelocity, const short turnRadius)
{
	try {
		g_pRoomba[hRoomba]->drive(translationVelocity, turnRadius);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_drive():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

LIBROOMBA_API int Roomba_driveDirect(const int hRoomba, const short rightWheelVelocity, const short leftWheelVelocity)
{
	try {
		g_pRoomba[hRoomba]->driveDirect(rightWheelVelocity, leftWheelVelocity);
	} catch (PreconditionNotMetError &e) {
		std::cerr << "Error in Roomba_drive():" << e.what() << std::endl;
		return PRECONDITION_NOT_MET;
	}
	return 0;
}

