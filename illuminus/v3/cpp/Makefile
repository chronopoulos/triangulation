CC = g++
FLAGS = -Wall -O3

all:
	$(CC) $(FLAGS) main.cpp mcp3008Spi.cpp -o main

test: test.cpp
	$(CC) $(FLAGS) test.cpp -o test

clean:
	rm -f *.o main
