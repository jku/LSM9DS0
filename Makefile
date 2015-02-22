
CFLAGS=-I. -Wall
CURSES_CFLAGS=-I/usr/include/ncursesw

LIBS=-lm
CURSES_LIBS=-lncurses

HEADERS = edison-9dof-i2c.h
OBJ = edison-9dof-i2c.o
BINS = test calibrate-mag calibrate-acc-gyro

all: $(BINS)

test: test.o $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

calibrate-acc-gyro: calibrate-acc-gyro.o $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

calibrate-mag: calibrate-mag.o $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(CURSES_CFLAGS) $(LIBS) $(CURSES_LIBS)


%.o: %.c $(HEADERS)
	$(CC) -c -o $@ $< $(CFLAGS)

calibrate-mag.o: calibrate-mag.c $(HEADERS)
	$(CC) -c -o $@ $< $(CFLAGS) $(CURSES_CFLAGS)


.PHONY: clean

clean:
	rm -f *.o $(BINS)
