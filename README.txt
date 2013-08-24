libRoomba


libRoomba is a c++ and python library for controlling Roomba (iRobot).

You can compile this library in Win, Mac, and Linux.


This git repository depends on libaqua library that is also written by me.



1. How to build

1.1 Clone this repository

1.2 Update submodule

# git submodule init
# git submodule update

1.3 Cmake

# mkdir build
# cd build
# cmake ../

or in windows, use cmake-gui that can be launched from START menu, then open CMakeLists.txt at the top level of this package,
You'd better set ${PATH_TO_YOUR_LIB}/libRoomba/build directory for the output directory, then configure and generate.

1.4 Build it

# make 

or in windows, you can find libRoomba.sln file in the build directory.
Open it and build it with your VC++.

