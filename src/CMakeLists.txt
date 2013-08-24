set(_srcs RoombaImpl.cpp Transport.cpp libroomba.cpp Protocol.cpp)
#set(_srcs Transport.cpp Protocol.cpp)
set(_demo_srcs demo.cpp)
set(_example_srcs example.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/libaqua/include)

find_package(Threads)

add_library(${PROJECT_NAME} SHARED ${_srcs})
add_library(${PROJECT_NAME}Static STATIC  ${_srcs})



set_target_properties(${PROJECT_NAME} PROPERTIES 
  COMPILE_FLAGS "-DaquaStatic_EXPORTS") 
set_target_properties(${PROJECT_NAME}Static PROPERTIES 
  COMPILE_FLAGS "-DaquaStatic_EXPORTS -DroombaStatic_EXPORTS") 

add_executable(${PROJECT_NAME}_demo ${_demo_srcs})
add_executable(${PROJECT_NAME}_example ${_example_srcs})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}Static ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_demo
    EXPORT ${PROJECT_NAME} 
    RUNTIME DESTINATION ${BIN_INSTALL_DIR} 
    LIBRARY DESTINATION ${LIB_INSTALL_DIR} 
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR} )
#install(EXPORT ${PROJECT_NAME}
#    DESTINATION ${LIB_INSTALL_DIR}/${PROJECT_NAME}
#    FILE ${PROJECT_NAME}Depends.cmake)

