/***********************************************************************
 * mcp3008SpiTest.cpp. Sample program that tests the mcp3008Spi class.
 * an mcp3008Spi class object (a2d) is created. the a2d object is instantiated
 * using the overloaded constructor. which opens the spidev0.0 device with
 * SPI_MODE_0 (MODE 0) (defined in linux/spi/spidev.h), speed = 1MHz &
 * bitsPerWord=8.
 *
 * call the spiWriteRead function on the a2d object 20 times. Each time make sure
 * that conversion is configured for single ended conversion on CH0
 * i.e. transmit ->  byte1 = 0b00000001 (start bit)
 *                   byte2 = 0b1000000  (SGL/DIF = 1, D2=D1=D0=0)
 *                   byte3 = 0b00000000  (Don't care)
 *      receive  ->  byte1 = junk
 *                   byte2 = junk + b8 + b9
 *                   byte3 = b7 - b0
 *    
 * after conversion must merge data[1] and data[2] to get final result
 *
 *
 *
 * *********************************************************************/
#include "mcp3008Spi.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


using namespace std;


int main(void)
{


    mcp3008Spi a2d("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);

    int a2dVal = 0;
    unsigned char data[3];
 
    while(true)
    {

        data[0] = 1;            // start bit
        data[1] = 0b10000000;   // single ended, channel 0
        data[2] = 0;            // don't care
 
        a2d.spiWriteRead(data, sizeof(data) );

        a2dVal = 0;
        a2dVal = data[1];
        a2dVal &= 0b00000011;
        a2dVal <<= 8;
        a2dVal |= data[2];
        cout << a2dVal << endl;

        sleep(1);

    }
    return 0;
}
