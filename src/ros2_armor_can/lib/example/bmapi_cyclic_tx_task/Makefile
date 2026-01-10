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
	rm -f bmapi_cyclic_tx_task.o bmapi_cyclic_tx_task

example:
	$(CC) $(CFLAGS) bmapi_cyclic_tx_task.c -I../../include -L$(INCLUDE) -L. -l$(LIB) -lusb-1.0 -o bmapi_cyclic_tx_task 

