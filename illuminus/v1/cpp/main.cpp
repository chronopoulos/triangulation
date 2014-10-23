/*
Sensor-polling script for Triangulation, Illuminus version
Chris Chronopoulos, 20141022
*/

#include "mcp3008Spi.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


using namespace std;

// ADC objects
static mcp3008Spi adc0("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
static mcp3008Spi adc1("/dev/spidev0.1", SPI_MODE_0, 1000000, 8);

// board sample structures
static int calibration[16];
static int droneThreshold[16];
static int tripleBuffer[3][16];
static unsigned int iCurrent=2;
static unsigned int iMinus1=1;
static unsigned int iMinus2=0;

// cof = 'circle of fifths'
static int cof=0;
static int cofSwitch=0;
static int cofSent=0;

// hits
static int hitThreshold1 = 20;
static int hitThreshold2 = -5;

// scale
static int scale[] =   {0, 2, 4, 7, 9,
                        12, 14, 16, 19, 21,
                        24, 26, 28, 31, 33, 36};

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
        droneThreshold[i] = tmpVal/26;   // ~30 for calibration=800
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
        droneThreshold[8+i] = tmpVal/26;   // ~30 for calibration=800
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
        if ((calibration[i] - tmpVal) < droneThreshold[i]){
            tmpVal = 1023;
        }
        tripleBuffer[iCurrent][i] = tmpVal;
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
        if ((calibration[8+i] - tmpVal) < droneThreshold[i]){
            tmpVal = 1023;
        }
        tripleBuffer[iCurrent][8+i] = tmpVal;
    }

}

int minOfBoardSample(void)
{
    int tmp, i;
    int min=1024;
    for (i=0; i<16; i++){
        tmp = tripleBuffer[iCurrent][i];
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

void calculateHits(void)
{
    int i, diff1, diff2;

    for (i=0; i<16; i++){
        // this is calculating each diff twice
        // to save cycles, maintain a buffer
        diff1 = tripleBuffer[iMinus2][i] - tripleBuffer[iMinus1][i];
        diff2 = tripleBuffer[iMinus1][i] - tripleBuffer[iCurrent][i];
        if ( (diff1 > hitThreshold1) && (diff2 < hitThreshold2) ){
            cout << "hit " << scale[i] << " " << diff1 << ";" << endl;
            break; // just send the lowest note for now
        }
    }

}


void sendCOF(void)
{
    if (cofSwitch and !cofSent){
        cout << "cof " << cof << ";" << endl;
        cofSent = 1;
    }
}

void sendBoardSample(void)
{
    int i;
    cout << "bs ";
    for (i=0; i<16; i++){
        cout << tripleBuffer[iCurrent][i] << " ";
    }
    cout << ";" << endl;
}

void cycleIndices(void)
{
    iCurrent = (iCurrent+1) % 3;
    iMinus1 = (iMinus1+1) % 3;
    iMinus2 = (iMinus2+1) % 3;
}


int main(void)
{

    calibrate();
    while(1){
        takeBoardSample();
        calculate_cof();
        calculateHits();
        sendCOF();
        sendBoardSample();
        cycleIndices();
        usleep(30000); // 30 ms
    }

    return 0;

}
