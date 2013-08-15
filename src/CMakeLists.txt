set(_srcs RoombaImpl.cpp Transport.cpp libroomba.cpp )
set(_demo_srcs demo.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/libaqua/include)

find_package(Threads)

add_library(${PROJECT_NAME} SHARED ${_srcs})
add_library(${PROJECT_NAME}Static STATIC  ${_srcs})
add_executable(${PROJECT_NAME}_demo ${_demo_srcs})
target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_demo
    EXPORT ${PROJECT_NAME}
    RUNTIME DESTINATION ${BIN_INSTALL_DIR} 
    LIBRARY DESTINATION ${LIB_INSTALL_DIR} 
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR} )
install(EXPORT ${PROJECT_NAME}
    DESTINATION ${LIB_INSTALL_DIR}/${PROJECT_NAME}
    FILE ${PROJECT_NAME}Depends.cmake)

