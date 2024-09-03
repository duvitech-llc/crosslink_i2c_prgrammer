CC = gcc
CFLAGS = -Wall
LIBS = -lpigpio -li2c

all: xlink-i2c

xlink-i2c: xlink-i2c.c
	$(CC) $(CFLAGS) -o xlink-i2c xlink-i2c.c $(LIBS)

clean:
	rm -f xlink-i2c
