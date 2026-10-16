CC=gcc
CFLAGS=-Wall -O2 -march=native
LDLIBS=-lm

CHK_LIBOSMOCORE_RV=$(shell pkg-config --exists libosmocore; echo $$?)
ifeq ($(CHK_LIBOSMOCORE_RV),0)
CFLAGS+=`pkg-config libosmocore --cflags` -DHAVE_LIBOSMOCORE
LDLIBS+=`pkg-config libosmocore --libs`
endif

OBJS=ir77_ambe_decode

all: $(OBJS)

ir77_ambe_decode: ir77_ambe_decode.o ambe.o ecc.o frame.o math.o synth.o tables.o tone.o

clean:
	rm -f *.o $(OBJS)
