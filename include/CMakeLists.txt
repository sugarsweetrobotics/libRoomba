set(hdrs ComAccessException.h ComOpenException.h ComException.h ComStateException.h Odometry.h Roomba.h RoombaException.h SerialPort.h Thread.h TimeSpec.h Timer.h Transport.h common.h libroomba.h op_code.h type.h)
set(install_hdrs Roomba.h RoombaException.h)

install(FILES ${install_hdrs} DESTINATION ${INC_INSTALL_DIR}
    COMPONENT library)

