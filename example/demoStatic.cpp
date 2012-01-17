#include <stdio.h>
#include <iostream>
#include "YConsole.h"


#define LIBROOMBA_STATIC_EXPORTS
#include "Roomba.h"

using namespace net::ysuga;
using namespace net::ysuga::roomba;


int main(void) {
	try {
		init_scr();
		Roomba roomba("\\\\.\\COM16");

		roomba.setMode(roomba.MODE_SAFE);
		roomba.setMode(roomba.MODE_FULL);

		roomba.runAsync();
		
		while(true) {
			if(myKbhit()) {
				int c = myGetch();
				c = tolower(c);
				switch(c) {
				case 'u':
					roomba.driveDirect(1.0, 1.0);
					break;
				case 'j':
					roomba.driveDirect(0.0, 0.0);
					break;
				case 'm':
					roomba.driveDirect(-1.0, -1.0);
					break;
				case 'h':
					roomba.driveDirect(-1.0, 1.0);
					break;
				case 'k':
					roomba.driveDirect(1.0, -1.0);
					break;

				default:
					break;
				}
						
			}

		}

		exit_scr();

		for(int i = 0;i < 4;i++) {
			std::cout << "ON!" << std::endl;
			roomba.setLED(LED_DOCK, 250);
			std::cout << "Right Encoder = " << (int)roomba.getRightEncoderCounts() << std::endl;

			Thread::Sleep(1000);
			std::cout << "OFF!" << std::endl;

			roomba.setLED(LED_DOCK, 0);
			std::cout << "Right Encoder = " << (int)roomba.getRightEncoderCounts() << std::endl;
			Thread::Sleep(1000);
		}

	} catch (net::ysuga::ComOpenException &e) {
		std::cerr << "Exception Occured:" << e.what()  << std::endl;
	}
	getchar();
	return 0;
}