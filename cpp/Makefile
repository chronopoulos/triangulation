CC = g++
FLAGS = -Wall -O3 -l bcm2835

all:
	$(CC) -o main main.cpp mcp3008Spi.cpp $(FLAGS)

clean:
	rm -f *.o main
