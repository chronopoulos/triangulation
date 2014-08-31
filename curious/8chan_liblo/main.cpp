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
#include "lo/lo.h"


using namespace std;


int main(void)
{

    lo_address pd = lo_address_new(NULL, "8000");
    //lo_address pd = lo_address_new("10.1.15.127", "8000");

    mcp3008Spi a2d("/dev/spidev0.1", SPI_MODE_0, 1000000, 8);

    int tmpVal = 0;
    int bs[8]; // "board sample"

    unsigned int i;
    unsigned char data[3];
 
    while(true)
    {

        for (i=0; i<8; i++) {
            data[0] = 1;            // start bit
            data[1] = 0b10000000 | (i << 4);   // single ended, channel i
            data[2] = 0;            // don't care
            a2d.spiWriteRead(data, sizeof(data) );
            tmpVal = 0;
            tmpVal = data[1];
            tmpVal &= 0b00000011;
            tmpVal <<= 8;
            tmpVal |= data[2];

            bs[i] = tmpVal;
            cout << tmpVal << " ";
        }

        cout << endl;
        lo_send(pd, "/photodiode", "iiiiiiii", bs[0], bs[1], bs[2], bs[3], bs[4], bs[5], bs[6], bs[7]);

        usleep(30000); // 30 ms

    }

    return 0;

}
