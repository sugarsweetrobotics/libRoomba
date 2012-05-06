all: ../bin/demo


CFLAGS=-O2 -Wall -fPIC -I../include -c

LD=g++
#LDFLAGS=-L../bin  -lRoomba1
LIBROOMBA=../lib/libRoomba.a
.cpp.o:
	${CC} ${CFLAGS} $<

../bin/demo: demo.o
	${LD} ${LDFLAGS} -o ../bin/demo demo.o $(LIBROOMBA)

../lib/libysuga.a:
	cd ../src; make;

clean:
	rm -rf *.o *~ ../bin/demo