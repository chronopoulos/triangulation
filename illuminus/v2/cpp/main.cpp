/*
Sensor-polling script for Triangulation, Illuminus version
Chris Chronopoulos, 20141022
*/

#include "mcp3008Spi.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>


using namespace std;

// ADC objects
static mcp3008Spi adc0("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
static mcp3008Spi adc1("/dev/spidev0.1", SPI_MODE_0, 1000000, 8);

// board sample structures
static int calibration[16];
static int droneThreshold[16];
static int doubleBuffer[2][16];
static unsigned int iCurrent=2;
static unsigned int iMinus1=1;

// cof = 'circle of fifths'
static int cof=0;
static int cofSwitch=0;
static int cofSent=0;

// hits
static int hitThreshold1 = 10;
static int hitThreshold2 = -3;
static int potentialHitChanL = -1;
static int potentialHitVelL = 0;
static int potentialHitChanR = -1;
static int potentialHitVelR = 0;

// scale
static int scale[] =   {0, 2, 4, 7, 9,
                        12, 14, 16, 19, 21,
                        24, 26, 28, 31, 33, 36};

// spatial harmonics
static float aveL, aveR;
static float rollL, rollR;
static float yawL, yawR;

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
        doubleBuffer[iCurrent][i] = tmpVal;
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
        doubleBuffer[iCurrent][8+i] = tmpVal;
    }

}

int minOfBoardSample(void)
{
    int tmp, i;
    int min=1024;
    for (i=0; i<16; i++){
        tmp = doubleBuffer[iCurrent][i];
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

void sendCOF(void)
{
    if (cofSwitch and !cofSent){
        cout << "cof " << cof << ";" << endl;
        cofSent = 1;
    }
}

void followUpPotentialHits(void)
{
    int diff;

    if (potentialHitChanL > 0) {
        diff = doubleBuffer[iMinus1][potentialHitChanL] - doubleBuffer[iCurrent][potentialHitChanL];
        if (diff < hitThreshold2){
            cout << "hitL " << scale[potentialHitChanL] << " " << potentialHitVelL << ";" << endl;
        }
    }

    if (potentialHitChanR > 0) {
        diff = doubleBuffer[iMinus1][potentialHitChanR] - doubleBuffer[iCurrent][potentialHitChanR];
        if (diff < hitThreshold2){
            cout << "hitR " << scale[potentialHitChanR] << " " << potentialHitVelR << ";" << endl;
        }
    }

}

void findPotentialHits(void)
{
    int i, diff, currentMax;

    potentialHitChanL = -1;
    currentMax = 0;
    for (i=0; i<8; i++) {
        diff = doubleBuffer[iMinus1][i] - doubleBuffer[iCurrent][i];
        if ( (diff > hitThreshold1) && (diff > currentMax) ) {
            potentialHitChanL = i;
            currentMax = diff;
        }
    }
    potentialHitVelL = currentMax;

    potentialHitChanR = -1;
    currentMax = 0;
    for (i=8; i<16; i++) {
        diff = doubleBuffer[iMinus1][i] - doubleBuffer[iCurrent][i];
        if ( (diff > hitThreshold1) && (diff > currentMax) ) {
            potentialHitChanR = i;
            currentMax = diff;
        }
    }
    potentialHitVelR = currentMax;
}

void computeHarmonics(void)
{
    int i, tmp;

    // ave

        tmp=0;
        for (i=0; i<8; i++){
            tmp += doubleBuffer[iCurrent][i];
        }
        aveL = tmp / 8.;

        tmp=0;
        for (i=8; i<16; i++){
            tmp += doubleBuffer[iCurrent][i];
        }
        aveR = tmp / 8.;

    cout << "aves " << aveL << " " << aveR << ";" << endl;

    // roll

        tmp = 0;
        for (i=0; i<8; i+=2){
            tmp += doubleBuffer[iCurrent][i];
        }
        for (i=1; i<8; i+=2){
            tmp -= doubleBuffer[iCurrent][i];
        }
        rollL = tmp / 8.;

        tmp = 0;
        for (i=8; i<16; i+=2){
            tmp += doubleBuffer[iCurrent][i];
        }
        for (i=9; i<16; i+=2){
            tmp -= doubleBuffer[iCurrent][i];
        }
        rollR = tmp / 8.;

        cout << "rolls " << rollL << " " << rollR << ";" << endl;

    // yaw

        tmp = 0;
        for (i=0; i<4; i++){
            tmp += doubleBuffer[iCurrent][i];
        }
        for (i=4; i<8; i++){
            tmp -= doubleBuffer[iCurrent][i];
        }
        yawL = tmp / 8.;

        tmp = 0;
        for (i=8; i<12; i++){
            tmp += doubleBuffer[iCurrent][i];
        }
        for (i=12; i<16; i++){
            tmp -= doubleBuffer[iCurrent][i];
        }
        yawR = tmp / 8.;

        cout << "yaws " << yawL << " " << yawR << ";" << endl;

}

void sendBoardSample(void)
{
    int i;
    cout << "bs ";
    for (i=0; i<16; i++){
        cout << doubleBuffer[iCurrent][i] << " ";
    }
    cout << ";" << endl;
}

void cycleIndices(void)
{
    // for 2nd order, could just do logical negation,
    //   but this is more general to higher-order buffering
    iCurrent = (iCurrent+1) % 2;
    iMinus1 = (iMinus1+1) % 2;
}


int main(void)
{

    int i;
    time_t t1, t2;
    double seconds;

    calibrate();

    // warm up
    for (int i=0; i<4; i++){
        takeBoardSample();
        cycleIndices();
    }

    while(1){
    time(&t1);
    //for (i=0; i<1000; i++){

        takeBoardSample();

        calculate_cof();
        sendCOF();

        sendBoardSample();

        computeHarmonics();

        followUpPotentialHits();
        findPotentialHits();

        cycleIndices();
        usleep(15000); // 15 ms
    }
    time(&t2);
    seconds = difftime(t2,t1);
    cout << "Time elapsed in seconds: " << seconds << endl;

    return 0;

}
