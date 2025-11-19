CC = gcc
CFLAGS := -Wall -O3
LBITS := $(shell getconf LONG_BIT)
ARCH := $(shell getconf LONG_BIT)
INCLUDE := .

all: spitest 

clean:
	rm -f spitest.o spitest

spitest:
	$(CC) $(CFLAGS) spitest.c -I$(INCLUDE) -L$(INCLUDE) -L. -o spitest 

