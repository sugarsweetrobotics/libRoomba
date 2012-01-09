#include <stdio.h>
#include <iostream>


#include "Roomba.h"

using namespace net::ysuga;
using namespace net::ysuga::roomba;


int main(void) {
	try {
		Roomba roomba("\\\\.\\COM16");
		roomba.start();
		roomba.safeControl();		
		std::cout << "MODE: "<< roomba.getMode();
		roomba.runAsync();

		Thread::Sleep(1000);


		roomba.clean();
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
		roomba.safeControl();
		roomba.dock();
		Thread::Sleep(1000);
		std::cout << "Input" << std::endl;
		getchar();

	} catch (net::ysuga::ComOpenException &e) {
		std::cerr << "Exception Occured:" << e.what()  << std::endl;
	}
	return 0;
}