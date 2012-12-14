CC=g++
AR=ar
CFLAGS=-O2 -Wall -fPIC -I../include -c 
ARFLAGS=rv
OBJECTS=SerialPort.o Thread.o Roomba.o Transport.o



UNAME = ${shell uname}

ifeq ($(UNAME),Linux)
# for Linux
SOFILE=../bin/libRoomba.so.1.0
SONAME=libRoomba.so.1
SOFLAGS=-shared -Wl,-soname,$(SONAME) 

endif

ifeq ($(UNAME),Darwin)
# for MacOSX
SOFILE=../bin/libRoomba1.dylib
SONAME=libRoomba1.dylib
SOFLAGS=-dynamiclib

endif


TARGET=../lib/libRoomba.a ../lib/libRoomba.a $(SOFILE)

all: $(TARGET) 

.cpp.o:
	$(CC) $(CFLAGS) $<

$(SOFILE): $(OBJECTS)
	$(CC) $(SOFLAGS) -o $(SOFILE) $(OBJECTS)

../lib/libRoomba.a: $(OBJECTS)
	$(AR) $(ARFLAGS) ../lib/libRoomba.a $(OBJECTS)


clean:
	rm -rf *~ *.o $(TARGET)

