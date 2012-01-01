#include <stdio.h>
#include <iostream>


#include "Roomba.h"

using namespace net::ysuga;
using namespace net::ysuga::roomba;


int main(void) {
	Roomba roomba("\\\\.\\COM16");

	roomba.SetMode(MODE_SAFE);
	roomba.SetMode(MODE_FULL);

	roomba.RunAsync();

	Thread::Sleep(1000);

	for(int i = 0;i < 4;i++) {
		std::cout << "ON!" << std::endl;
		roomba.SetLED(LED_DOCK, 250);
		std::cout << "Right Encoder = " << (int)roomba.GetRightEncoderCounts() << std::endl;

		Thread::Sleep(1000);
		std::cout << "OFF!" << std::endl;

		roomba.SetLED(LED_DOCK, 0);
		std::cout << "Right Encoder = " << (int)roomba.GetRightEncoderCounts() << std::endl;
		Thread::Sleep(1000);
	}
	getchar();

	return 0;
}