CC = gcc
CFLAGS = -Wall
LIBS = -li2c

all: xlink-i2c

i2c_example: xlink-i2c.c
	$(CC) $(CFLAGS) -o xlink-i2c xlink-i2c.c $(LIBS)

clean:
	rm -f xlink-i2c