CC = gcc
CFLAGS+= -Wall -std=gnu99 -g 

LDFLAGS=-pthread 


CFLAGS+=$(shell sdl2-config --cflags --libs)


HW=prgsem
BINARIES=prgsem-main


LDFLAGS+=$(shell sdl2-config --cflags --libs)

all: ${BINARIES}

OBJS=${patsubst %.c,%.o,${wildcard *.c}}

prgsem-main: ${OBJS}
	${CC} ${OBJS} ${LDFLAGS} -o $@

${OBJS}: %.o: %.c
	${CC} -c ${CFLAGS} $< -o $@

zip:
	zip ${HW}-brute.zip nucleo/prgsem-mbed.cpp ${wildcard *.c} ${wildcard *.h}

clean:
	rm -f ${BINARIES} ${OBJS}
	rm -f ${HW}-brute.zip