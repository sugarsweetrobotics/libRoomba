TARGET=lib/libysuga.a bin/demo



CFLAGS=-Wall -O2 
LDFLAGS=-Wall -O2

SONAME=libRoomba.so.1
SOFILE=libRoomba.so.1.0
TARGET_LIBDIR=/usr/lib/

all: $(TARGET)


lib/libysuga.a:
	cd src;make;

bin/demo: lib/libysuga.a
	cd example;make;
.cpp.o:
	$(CC) $(CFLAGS) -c $<


UNAME=${shell uname}

ifeq ($(UNAME), Darwin)



endif





install: 
	cp bin/$(SOFILE) $(TARGET_LIBDIR)
	/sbin/ldconfig $(TARGET_LIBDIR)
	ln -s $(TARGET_LIBDIR)$(SONAME) $(TARGET_LIBDIR)$(SOFILE)

uninstall:

clean:
	cd src; make clean;
	cd example; make clean;
	rm -rf *~ lib/*.a