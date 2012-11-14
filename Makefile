TARGET=lib/libysuga.a bin/demo


CFLAGS=-Wall -O2 
LDFLAGS=-Wall -O2

UNAME = ${shell uname}

ifeq (\$(UNAME),Linux)
#for Linux
SONAME=libRoomba.so.1
SOFILE=libRoomba.so.1.0
TARGET_LIBDIR=/usr/lib/

endif

ifeq ($(UNAME),Darwin)
#for Linux
SONAME=libRoomba1.dylib
SOFILE=libRoomba1.dylib
TARGET_LIBDIR=/usr/lib/

endif



all: $(TARGET)


lib/libysuga.a:
	cd src;make;

bin/demo: lib/libysuga.a
	cd example;make;
.cpp.o:
	$(CC) $(CFLAGS) -c $<



install: 
	cp bin/$(SOFILE) $(TARGET_LIBDIR)
ifeq (\$(UNAME),Linux)
	/sbin/ldconfig $(TARGET_LIBDIR)
	ln -s $(TARGET_LIBDIR)$(SONAME) $(TARGET_LIBDIR)$(SOFILE)
endif


uninstall:
	rm ${TARGET_LIBDIR}${SOFILE}


clean:
	cd src; make clean;
	cd example; make clean;
	rm -rf *~ lib/*.a