
CFLAGS=-I/usr/include/ncursesw -I. -Wall
LIBS=-lm -lncurses
HEADERS = edison-9dof-i2c.h
OBJ = edison-9dof-i2c.o

%.o: %.c $(HEADERS)
	$(CC) -c -o $@ $< $(CFLAGS)

all: test calibrate-mag calibrate-acc-gyro

test: test.o $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

calibrate-acc-gyro: calibrate-acc-gyro.o $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

calibrate-mag: calibrate-mag.o $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f *.o test calibrate-acc-gyro
