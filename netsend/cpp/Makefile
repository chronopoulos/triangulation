CC = g++
FLAGS = -Wall -llo

all:
	$(CC) $(FLAGS) main.cpp mcp3008Spi.cpp -o main

test: test.cpp
	$(CC) $(FLAGS) test.cpp -o test

clean:
	rm -f *.o main
