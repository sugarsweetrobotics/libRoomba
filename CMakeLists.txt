cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(roomba)


set(PROJECT_VERSION 1.0.2 CACHE STRING "libroomba version")
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_REVISION 2)
set(PROJECT_VERSION_CANDIDATE 0)
set(PROJECT_DESCRIPTION "libroomba")
set(PROJECT_VENDOR "SugarSweetRobotics")
set(PROJECT_VENDOR_CONTACT "ysuga@sugarsweetrobotics.com")
set(PROJECT_AUTHOR "SugarSweetRobotics")
set(PROJECT_AUTHOR_SHORT "SSR")

# Add an "uninstall" target
#CONFIGURE_FILE ("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
#    "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
#ADD_CUSTOM_TARGET (uninstall "${CMAKE_COMMAND}" -P
#    "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

# Set up installation directories
set(BIN_INSTALL_DIR "bin")
set(LIB_INSTALL_DIR "lib")
set(MAN_INSTALL_DIR "share/man/man3")

set(INC_INSTALL_DIR
    "include/${PROJECT_NAME}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
set(SHARE_INSTALL_DIR
    "share/${PROJECT_NAME}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")

# Subdirectories
add_subdirectory(cmake)
add_subdirectory(libaqua)
add_subdirectory(include)
add_subdirectory(doc)
add_subdirectory(src)

# Package creation
include(InstallRequiredSystemLibraries)
set(PROJECT_EXECUTABLES ${PROJECT_NAME}_demo
    "demo")

#example install
set(example_files example/CMakeLists.txt example/demo.cpp)
install(FILES ${example_files} DESTINATION ${SHARE_INSTALL_DIR}/example COMPONENT library)

set(cpack_options "${PROJECT_BINARY_DIR}/cpack_options.cmake")
configure_file("${PROJECT_SOURCE_DIR}/cmake/cpack_options.cmake.in"
    ${cpack_options} @ONLY)
set(CPACK_PROJECT_CONFIG_FILE ${cpack_options})
include(${CPACK_PROJECT_CONFIG_FILE})
include(CPack)


