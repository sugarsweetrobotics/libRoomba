TARGET=lib/libysuga.a bin/demo


CFLAGS=-Wall -O2 
LDFLAGS=-Wall -O2

UNAME = ${shell uname}

TARGET_LIBDIR=/usr/local/share/libroomba/


ifeq (\$(UNAME),Linux)
#for Linux
SONAME=libRoomba.so.1
SOFILE=libRoomba.so.1.0
endif

ifeq ($(UNAME),Darwin)
#for MacOSX
SONAME=libRoomba1.dylib
SOFILE=libRoomba1.dylib
endif



all: $(TARGET)


lib/libysuga.a:
	cd src;make;

bin/demo: lib/libysuga.a
	cd example;make;
.cpp.o:
	$(CC) $(CFLAGS) -c $<



install:
	mkdir -p ${TARGET_LIBDIR}
	mkdir -p ${TARGET_LIBDIR}bin
	mkdir -p ${TARGET_LIBDIR}include
	mkdir -p ${TARGET_LIBDIR}lib
	cp bin/$(SOFILE) $(TARGET_LIBDIR)lib
	cp bin/demo ${TARGET_LIBDIR}bin
	cp include/*.h ${TARGET_LIBDIR}include
ifeq (\$(UNAME),Linux)
	/sbin/ldconfig $(TARGET_LIBDIR)
	ln -s $(TARGET_LIBDIR)$(SONAME) $(TARGET_LIBDIR)$(SOFILE)
endif


uninstall:
	rm -rf  ${TARGET_LIBDIR}


clean:
	cd src; make clean;
	cd example; make clean;
	rm -rf *~ lib/*.a