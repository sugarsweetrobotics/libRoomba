#include <stdio.h>
#include <iostream>


#include "Roomba.h"

using namespace net::ysuga;
using namespace net::ysuga::roomba;


int main(void) {
	try {
		Roomba roomba("\\\\.\\COM16");

		roomba.setMode(MODE_SAFE);
		roomba.setMode(MODE_FULL);

		roomba.runAsync();

		Thread::Sleep(1000);

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