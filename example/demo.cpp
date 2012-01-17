#include <stdio.h>
#include <iostream>

#include "YConsole.h"
#include "Roomba.h"

using namespace net::ysuga;
using namespace net::ysuga::roomba;



int main(void) {
	try {
		bool endflag = false;
		init_scr();
		Roomba roomba("\\\\.\\COM16");
		
		roomba.fullControl();
		roomba.runAsync();
		
		while(!endflag) {
			clear_scr();
			std::cout << "---------------------------------------------------------------\n";
			std::cout << "sideBrush mainBrush vacuum               manuvour\n";
			std::cout << "w: cw     e: cw     r: on                u: forward\n";
			std::cout << "s: off    d: off    f: off       h: left j: stop    k: right\n";
			std::cout << "x: ccw    c: ccw    v: on                m: backward\n";
			std::cout << "---------------------------------------------------------------\n";

			std::cout << "Right:" << std::right << roomba.getRightEncoderCounts() << std::endl;
			std::cout << "Left :" << std::right << roomba.getLeftEncoderCounts() << std::endl;
			std::cout << "---------------------------------------------------------------\n";
			std::cout << "q: quit\n";
			std::cout << "p: clean    o: spot_clean   i: max_clean  l: stop   ,: dock\n";

			short r_vel = 0;
			short l_vel = 0;

			if(myKbhit()) {
				int c = myGetch();
				c = tolower(c);
				switch(c) {
				case 'u':
					r_vel = 100;
					l_vel = 100;
					break;
				case 'm':
					r_vel = -100;
					l_vel = -100;
					break;
				case 'h':
					r_vel = 100;
					l_vel = -100;
					break;
				case 'k':
					r_vel = -100;
					l_vel = 100;
					break;
				case 'j':
					r_vel = l_vel = 0;
					break;

				case 'w':
					roomba.driveMainBrush(roomba.MOTOR_CW);
					break;
				case 's':
					roomba.driveMainBrush(roomba.MOTOR_OFF);
					break;
				case 'x':
					roomba.driveMainBrush(roomba.MOTOR_CCW);
					break;

				case 'e':
					roomba.driveSideBrush(roomba.MOTOR_CW);
					break;
				case 'd':
					roomba.driveSideBrush(roomba.MOTOR_OFF);
					break;
				case 'c':
					roomba.driveSideBrush(roomba.MOTOR_CCW);
					break;

				case 'r':
					roomba.driveVacuum(roomba.MOTOR_ON);
					break;				
				case 'f':
					roomba.driveVacuum(roomba.MOTOR_OFF);
					break;
				case 'v':
					roomba.driveVacuum(roomba.MOTOR_ON);
					break;

				case 'q':
					r_vel = l_vel = 0;
					roomba.driveMotors(roomba.MOTOR_OFF, roomba.MOTOR_OFF, roomba.MOTOR_OFF);
					endflag = true;
					break;

				case 'p':
					roomba.clean();
					break;

				case 'o':
					roomba.spotClean();
					break;

				case 'i':
					roomba.maxClean();
					break;

				case 'l':
					roomba.safeControl();
					break;

				case ',':
					roomba.dock();
					break;

				default:
					r_vel = l_vel = 0;
					break;
				}
				try {
					if(roomba.getMode() == roomba.MODE_SAFE || roomba.getMode() == roomba.MODE_SAFE) {
						roomba.driveDirect(r_vel, l_vel);
					}
				} catch (net::ysuga::roomba::PreconditionNotMetError &e) {
					std::cout << "Precondition Not Met" << std::endl;
				}

			} else {
				r_vel = l_vel = 0;
			}

		}

		exit_scr();
		roomba.safeControl();

	} catch (net::ysuga::ComOpenException &e) {
		std::cerr << "Exception Occured:" << e.what()  << std::endl;
	}
	std::cout << "Hit Enter Key to Exit." << std::endl;
	getchar();
	return 0;
}