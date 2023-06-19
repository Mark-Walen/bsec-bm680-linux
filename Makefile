CC = gcc
CFLAGS = -Wall

all: main bme68x driver

main: basic.c bme68x driver
	$(CC) basic.c bme68x/bme68x.c \
	common/driver.so \
	bsec_integration/bsec_integration.c \
	-o basic.o \
	-I./inc -I./common -I./bme68x -I./bsec_integration \
	-L./lib -lalgobsec -lm

driver: common/driver.c bme68x
	$(CC) $(CFLAGS) -shared common/driver.c -o common/driver.so -I./bme68x -lrt -li2c -fPIC

bme68x: bme68x/bme68x.c
	$(CC) $(CFLAGS) -c bme68x/bme68x.c -o bme68x/bme68x.o

clean:
	rm *.o