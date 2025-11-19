CC = gcc
CFLAGS := -Wall -O3
LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
	INCLUDE = ../../bin/unix64/release
	LIB = bmapi64
else
	INCLUDE = ../../bin/unix32/release
	LIB = bmapi
endif
ARCH := $(shell getconf LONG_BIT)

all: example

clean:
	rm -f bmapi_receive_only.o bmapi_receive_only

example:
	$(CC) $(CFLAGS) bmapi_receive_only.c -I../../include -L$(INCLUDE) -L. -l$(LIB) -lusb-1.0 -o bmapi_receive_only

