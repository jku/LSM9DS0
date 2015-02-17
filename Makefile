
CFLAGS=-I. -Wall
LIBS=-lm
HEADERS = edison-9dof-i2c.h
OBJ = test.o edison-9dof-i2c.o

%.o: %.c $(HEADERS)
	$(CC) -c -o $@ $< $(CFLAGS)

test: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm *.o test
