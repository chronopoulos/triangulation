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

static lo_address pd = lo_address_new(NULL, "8000");
//static lo_address pd = lo_address_new("10.1.13.139", "8000");
static mcp3008Spi adc0("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
static mcp3008Spi adc1("/dev/spidev0.1", SPI_MODE_0, 1000000, 8);

static int calibration[16];
static int bs[16]; // "board sample"
static int threshold[16];
static int Lextreme[2];
static int Rextreme[2];

static int cof=0;

static int cofSwitch=0;
static int cofSent=0;

void calibrate(void)
{

    unsigned char data[3];
    int tmpVal = 0;
    unsigned int i;

    for (i=0; i<8; i++) {
        data[0] = 1;            // start bit
        data[1] = 0b10000000 | (i << 4);   // single ended, channel i
        data[2] = 0;            // don't care
        adc0.spiWriteRead(data, sizeof(data) );
        tmpVal = 0;
        tmpVal = data[1];
        tmpVal &= 0b00000011;
        tmpVal <<= 8;
        tmpVal |= data[2];

        calibration[i] = tmpVal;
        threshold[i] = tmpVal/26;   // ~30 for calibration=800
    }

    
    for (i=0; i<8; i++) {
        data[0] = 1;            // start bit
        data[1] = 0b10000000 | (i << 4);   // single ended, channel i
        data[2] = 0;            // don't care
        adc1.spiWriteRead(data, sizeof(data) );
        tmpVal = 0;
        tmpVal = data[1];
        tmpVal &= 0b00000011;
        tmpVal <<= 8;
        tmpVal |= data[2];

        calibration[8+i] = tmpVal;
        threshold[8+i] = tmpVal/26;   // ~30 for calibration=800
    }

}

void takeBoardSample(void)
{

    unsigned char data[3];
    int tmpVal = 0;
    unsigned int i;

    for (i=0; i<8; i++) {
        data[0] = 1;            // start bit
        data[1] = 0b10000000 | (i << 4);   // single ended, channel i
        data[2] = 0;            // don't care
        adc0.spiWriteRead(data, sizeof(data) );
        tmpVal = 0;
        tmpVal = data[1];
        tmpVal &= 0b00000011;
        tmpVal <<= 8;
        tmpVal |= data[2];
        if ((calibration[i] - tmpVal) < threshold[i]){
            tmpVal = 1023;
        }
        bs[i] = tmpVal;
        cout << tmpVal << " ";
    }

    cout << "  ";
    
    for (i=0; i<8; i++) {
        data[0] = 1;            // start bit
        data[1] = 0b10000000 | (i << 4);   // single ended, channel i
        data[2] = 0;            // don't care
        adc1.spiWriteRead(data, sizeof(data) );
        tmpVal = 0;
        tmpVal = data[1];
        tmpVal &= 0b00000011;
        tmpVal <<= 8;
        tmpVal |= data[2];
        if ((calibration[8+i] - tmpVal) < threshold[i]){
            tmpVal = 1023;
        }
        bs[8+i] = tmpVal;
        cout << tmpVal << " ";
    }

    cout << endl;

}

void calculateExtrema(void)
{
    int tmp, i;

    int index=-1;
    int min=1024;
    for (i=0; i<8; i++){
        tmp = bs[i];
        if (tmp<min){
            index = i;
            min = tmp;
        }
    }
    if (min==1023){
        Lextreme[0] = -1;
    } else {
        Lextreme[0] = index;
    }
    Lextreme[1] = min;

    index=-1;
    min=1024;
    for (i=8; i<16; i++){
        tmp = bs[i];
        if (tmp<min){
            index = i;
            min = tmp;
        }
    }
    if (min==1023){
        Rextreme[0] = -1;
    } else {
        Rextreme[0] = index-8;
    }
    Rextreme[1] = min;
}

int minOfBoardSample(void)
{
    int tmp, i;
    int min=1024;
    for (i=0; i<16; i++){
        tmp = bs[i];
        if (tmp<min){
            min = tmp;
        }
    }

    return min;
}

void calculate_cof(void)
{
    if (cofSwitch) {
        if (minOfBoardSample() == 1023){
            cofSwitch=0;
            cofSent=0;
        }
    } else {
        if (minOfBoardSample() < 1023){
            cof = (cof+7) % 12;
            cofSwitch = 1;
        }
    }
}


void sendOSC(void)
{
    if (cofSwitch and !cofSent){
        lo_send(pd, "/cof", "i", cof);
        cofSent = 1;
    }

    lo_send(pd, "/photodiode", "iiiiiiiiiiiiiiii", bs[0], bs[1], bs[2], bs[3], bs[4], bs[5], bs[6], bs[7],
                                                    bs[8], bs[9], bs[10], bs[11], bs[12], bs[13], bs[14], bs[15]);
}


int main(void)
{

    calibrate();
    while(1){

        takeBoardSample();
        calculate_cof();
        sendOSC();
        usleep(30000); // 30 ms
    }

    return 0;

}
