CC=gcc
CPPFLAGS=-Wall   -O3 -std=gnu99
#CPPFLAGS=   -g  -std=gnu99

# SRC=src/utils.o src/priority_queue.o src/ai.o src/pacman.o
# TARGET=pacman
SOURCES = $(filter-out $(wildcard pacman*), $(wildcard *.c))
OBJECTS = $(SOURCES:.c=.o)

test: pacman_testing.o $(OBJECTS)
	$(CC) $^ pacman_testing.c -o $(TARGET) $(LDFLAGS)

all: $(SRC)
	$(CC) -o $(TARGET) $(SOURCES) $(CPPFLAGS) -lncurses -lm

clean:
	rm -f $(TARGET) src/*.o
