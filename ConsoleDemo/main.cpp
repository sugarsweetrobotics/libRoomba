#include <stdio.h>
#include <iostream>


#include "Roomba.h"

using namespace net::ysuga::roomba;


int main(void) {
	Roomba roomba("\\\\.\\COM16");

	roomba.SetMode(Roomba::SAFE);
	roomba.SetMode(Roomba::FULL);

	Sleep(1000);

	for(int i = 0;i < 4;i++) {
		std::cout << "ON!" << std::endl;
		roomba.SetLED(Roomba::LED_DOCK, 250);
		//roomba.Drive(30, 0);
		std::cout << "Temperature = " << roomba.GetTemperature() << std::endl;
		std::cout << "Voltage     = " << roomba.GetVoltage() << std::endl;

		Sleep(1000);
		std::cout << "OFF!" << std::endl;

		roomba.SetLED(Roomba::LED_DOCK, 0);
		//roomba.Drive(0, 0);
		Sleep(1000);
	}
	getchar();

	return 0;
}