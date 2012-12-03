all: ../bin/roomba_demo


CFLAGS=-O2 -Wall -fPIC -I../include -c

LD=g++
LDFLAGS=-L../bin
#LIBROOMBA=../lib/libRoomba.a
.cpp.o:
	${CC} ${CFLAGS} $<

../bin/roomba_demo: demo.o
	${LD} ${LDFLAGS} -o ../bin/roomba_demo demo.o ../lib/libRoomba.a -lpthread

../lib/libysuga.a:
	cd ../src; make;

clean:
	rm -rf *.o *~ ../bin/roomba_demo