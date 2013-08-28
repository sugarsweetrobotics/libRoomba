#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "libroomba.h"

using namespace ssr;

int main(const int argc, const char* argv[]) {
  int baudrate = 115200;
  if(argc == 1) {
    std::cout << "USAGE: demo /dev/ttyUSB0" << std::endl;
    std::cout << "Press enter key to exit" << std::endl;
    getchar();
    return 0;
  } else if (argc == 3) {
    baudrate = atoi(argv[2]);
  }

  try {
    Roomba* pRoomba = createRoomba(MODEL_500SERIES, argv[1], baudrate);
    pRoomba->safeControl();
    pRoomba->runAsync();
    int c;
    std::cin >> c;
    pRoomba->clean();
    std::cin >> c;
    delete pRoomba;
  } catch (std::exception &e) {
    std::cout << "Excepiton:" << e.what() << std::endl;
  }

  int c;
  std::cin >> c;
  return 0;
}
