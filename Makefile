
CFLAGS=-I. -Wall
LIBS=-lm
HEADERS = edison-9dof-i2c.h
OBJ = edison-9dof-i2c.o

%.o: %.c $(HEADERS)
	$(CC) -c -o $@ $< $(CFLAGS)

test: test.o $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

calibrate-acc-gyro: calibrate-acc-gyro.o $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f *.o test calibrate-acc-gyro
