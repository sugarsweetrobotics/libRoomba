TARGET=lib/libysuga.a bin/roomba_demo


CFLAGS=-Wall -O2 
LDFLAGS=-Wall -O2

UNAME = ${shell uname}

TARGET_LIBDIR=/usr/local/share/libroomba/

INSTALL_PREFIX=/usr/

ifeq ($(UNAME),Linux)
#for Linux
SONAME=libRoomba.so.1
SOFILE=libRoomba.so.1.0
endif

ifeq ($(UNAME),Darwin)
#for MacOSX
SONAME=libRoomba.dylib
SOFILE=libRoomba1.dylib
endif



all: $(TARGET)


lib/libysuga.a:
	cd src;make;

bin/roomba_demo: lib/libysuga.a
	cd example;make;
.cpp.o:
	$(CC) $(CFLAGS) -c $<



install:
	mkdir -p ${TARGET_LIBDIR}
	mkdir -p ${TARGET_LIBDIR}bin
	mkdir -p ${TARGET_LIBDIR}include
#	mkdir -p ${TARGET_LIBDIR}lib
	cp bin/$(SOFILE) ${INSTALL_PREFIX}/lib
	cp bin/roomba_demo ${INSTALL_PREFIX}/bin
	cp include/*.h ${TARGET_LIBDIR}include
ifeq ($(UNAME),Linux)
	/sbin/ldconfig /usr/lib

endif


	ln -s ${INSTALL_PREFIX}/lib/$(SOFILE) ${INSTALL_PREFIX}/lib/$(SONAME)


uninstall:
	rm -rf  ${TARGET_LIBDIR}
	rm -rf  ${INSTALL_PREFIX}/lib/$(SOFILE)
	rm -rf ${INSTALL_PREFIX}/lib/$(SONAME) 


clean:
	cd src; make clean;
	cd example; make clean;
	rm -rf *~ lib/*.a