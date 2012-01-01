

#ifndef LIB_ROOMBA_HEADER_INCLUDED
#define LIB_ROOMBA_HEADER_INCLUDED

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif


	LIBROOMBA_API int Roomba_create(const char* portname, const int baudrate);

	LIBROOMBA_API int Roomba_destroy(const int hRoomba);

	LIBROOMBA_API int Roomba_setMode(const int hRoomba, const int mode);
	
	LIBROOMBA_API int Roomba_getMode(const int hRoomba, int *mode);

	LIBROOMBA_API int Roomba_drive(const int hRoomba, const short translationVelocity, const short turnRadius);

	LIBROOMBA_API int Roomba_driveDirect(const int hRoomba, const short rightWheelVelocity, const short leftWheelVelocity);



#ifdef __cplusplus
}
#endif



#endif // #ifndef LIB_ROOMBA_HEADER_INCLUDED