set(hdrs Odometry.h Roomba.h RoombaException.h Transport.h common.h libroomba.h op_code.h )
set(install_hdrs Roomba.h RoombaException.h)

install(FILES ${install_hdrs} DESTINATION ${INC_INSTALL_DIR}
    COMPONENT library)

