cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(RoombaRTC)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
include("${PROJECT_SOURCE_DIR}/cmake/utils.cmake")
set(PROJECT_VERSION 1.0.0 CACHE STRING "RoombaRTC version")
DISSECT_VERSION()
set(PROJECT_DESCRIPTION "Roomba RTC")
set(PROJECT_VENDOR "ysuga_net")
set(PROJECT_AUTHOR "ysuga_net")
set(PROJECT_AUTHOR_SHORT "ysuga_net")

# Add an "uninstall" target
CONFIGURE_FILE ("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
    "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET (uninstall "${CMAKE_COMMAND}" -P
    "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

#option(BUILD_EXAMPLES "Build and install examples" OFF)
option(BUILD_DOCUMENTATION "Build the documentation" ON)
#option(BUILD_TESTS "Build the tests" OFF)
#option(BUILD_TOOLS "Build the tools" OFF)
option(BUILD_IDL "Build and install idl" ON)
option(BUILD_SOURCES "Build and install sources" OFF)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
   # Mac OS X specific code
   SET(CMAKE_CXX_COMPILER "g++")
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

option(STATIC_LIBS "Build static libraries" OFF)
if(STATIC_LIBS)
    set(LIB_TYPE STATIC)
else(STATIC_LIBS)
    set(LIB_TYPE SHARED)
endif(STATIC_LIBS)

# Set up installation directories
set(BIN_INSTALL_DIR "bin")
set(LIB_INSTALL_DIR "lib")
set(INC_INSTALL_DIR
    "include/${PROJECT_NAME_LOWER}-${PROJECT_VERSION_MAJOR}")
set(SHARE_INSTALL_DIR
    "share/${PROJECT_NAME_LOWER}-${PROJECT_VERSION_MAJOR}")

# Get necessary dependency information
if(NOT WIN32)
    list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
endif(NOT WIN32)
find_package(OpenRTM REQUIRED)

find_package(LibRoomba REQUIRED)

# Universal settings
#enable_testing()

# Subdirectories
add_subdirectory(cmake)
if(BUILD_DOCUMENTATION)
    add_subdirectory(doc)
endif(BUILD_DOCUMENTATION)
#if(BUILD_EXAMPLES)
#    add_subdirectory(examples)
#endif(BUILD_EXAMPLES)
if(BUILD_IDL)
    add_subdirectory(idl)
endif(BUILD_IDL)
add_subdirectory(include)
add_subdirectory(src)
#if(BUILD_TESTS)
#    add_subdirectory(test)
#endif(BUILD_TESTS)
#if(BUILD_TOOLS)
#    add_subdirectory(tools)
#endif(BUILD_TOOLS)
if(BUILD_SOURCES)
    add_subdirectory(src)
endif(BUILD_SOURCES)

# Package creation
include(InstallRequiredSystemLibraries)
set(PROJECT_EXECUTABLES ${PROJECT_NAME_LOWER}Comp
    "${PROJECT_NAME_LOWER}Comp")
set(cpack_options "${PROJECT_BINARY_DIR}/cpack_options.cmake")
configure_file("${PROJECT_SOURCE_DIR}/cmake/cpack_options.cmake.in"
    ${cpack_options} @ONLY)
set(CPACK_PROJECT_CONFIG_FILE ${cpack_options})
include(${CPACK_PROJECT_CONFIG_FILE})
include(CPack)


