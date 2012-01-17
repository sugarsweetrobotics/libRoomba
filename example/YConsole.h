#ifndef YCONSOLE_HEADER_INCLUDED
#define YCONSOLE_HEADER_INCLUDED

#ifdef __cplusplus
namespace net {
	namespace ysuga {
#endif

#ifdef WIN32
#include <conio.h>
#endif


static void init_scr() {
#ifdef WIN32
	system("cls");
#endif
}

static void clear_scr() {
#ifdef WIN32
	system("cls");
#endif
}

static void exit_scr() {
#ifdef WIN32
	system("cls");
#endif
}

static int myKbhit() {
#ifdef WIN32
	return _kbhit();
#endif
}


static int myGetch() {
#ifdef WIN32
	return _getch();
#endif
}


#ifdef __cplusplus
	} // namespace ysuga
} // namespace net
#endif


#endif // #ifndef YCONSOLE_HEADER_INCLUDED