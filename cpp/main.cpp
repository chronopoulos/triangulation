/*
Sensor-polling script for Triangulation
Chris Chronopoulos, 20141022
*/

#include "mcp3008Spi.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include <bcm2835.h>

#define GPIO_LEFT RPI_GPIO_P1_07
#define GPIO_RIGHT RPI_GPIO_P1_11

using namespace std;

// ADC objects
static mcp3008Spi adc0("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
static mcp3008Spi adc1("/dev/spidev0.1", SPI_MODE_0, 1000000, 8);

// doubleBuffer indices
static unsigned int iCurrent=1;
static unsigned int iMinus1=0;

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
static float aveL = 1023;
static float aveR = 1023;
static float rollL = 0;
static float rollR = 0;
static float pitchL = 0;
static float pitchR = 0;



void cookData(int* bg, int* raw, int* cooked, int threshFactor)
{

    int diff, i;
    for (i=0; i<16; i++) {
        diff = bg[i] - raw[i];
        if ( diff < (bg[i]/threshFactor) ) {
            cooked[i] = 1023;
        } else {
            //cooked[i] = raw[i]; // old way
            cooked[i] = (raw[i] - bg[i]) * 1023 / bg[i] + 1023; // new scaling
        }
    }

}

void takeBoardSample(int *buffer)
/* assumes buffer is a 16-element int array */
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
        buffer[i] = tmpVal;
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
        buffer[8+i] = tmpVal;
    }
}

int minOfBoardSample(int* boardSample)
{
    int tmp, i;
    int min=1024;
    for (i=0; i<16; i++){
        tmp = boardSample[i];
        if (tmp<min){
            min = tmp;
        }
    }

    return min;
}

void calculate_cof(int* boardSample)
{
    if (cofSwitch) {
        if (minOfBoardSample(boardSample) == 1023){
            cofSwitch=0;
            cofSent=0;
        }
    } else {
        if (minOfBoardSample(boardSample) < 1023){
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

void followUpPotentialHits(int *doubleBuff)
{
    int diff;

    if (potentialHitChanL > 0) {
        diff = doubleBuff[iMinus1*16+potentialHitChanL] - doubleBuff[iCurrent*16+potentialHitChanL];
        if (diff < hitThreshold2){
            cout << "hitL " << scale[potentialHitChanL] << " " << potentialHitVelL << ";" << endl;
            if (aveR < 20.){
                cout << "modeselektor bang;" << endl;
            }
        }
    }

    if (potentialHitChanR > 0) {
        diff = doubleBuff[iMinus1*16+potentialHitChanR] - doubleBuff[iCurrent*16+potentialHitChanR];
        if (diff < hitThreshold2){
            cout << "hitR " << scale[potentialHitChanR] << " " << potentialHitVelR << ";" << endl;
            if (aveL < 20.){
                cout << "modeselektor bang;" << endl;
            }
        }
    }

}

void findPotentialHits(int *doubleBuff)
{
    int i, diff, currentMax;

    potentialHitChanL = -1;
    currentMax = 0;
    for (i=0; i<8; i++) {
        diff = doubleBuff[iMinus1*16+i] - doubleBuff[iCurrent*16+i];
        if ( (diff > hitThreshold1) && (diff > currentMax) ) {
            potentialHitChanL = i;
            currentMax = diff;
        }
    }
    potentialHitVelL = currentMax;

    potentialHitChanR = -1;
    currentMax = 0;
    for (i=8; i<16; i++) {
        diff = doubleBuff[iMinus1*16+i] - doubleBuff[iCurrent*16+i];
        if ( (diff > hitThreshold1) && (diff > currentMax) ) {
            potentialHitChanR = i;
            currentMax = diff;
        }
    }
    potentialHitVelR = currentMax;
}

void computeHarmonics(int *boardSample)
{
    int i, tmp;

    // ave

        tmp=0;
        for (i=0; i<8; i++){
            tmp += boardSample[i];
        }
        aveL = tmp / 8.;

        tmp=0;
        for (i=8; i<16; i++){
            tmp += boardSample[i];
        }
        aveR = tmp / 8.;

    cout << "aves " << aveL << " " << aveR << ";" << endl;

    // roll

        tmp = 0;
        for (i=0; i<8; i+=2){
            tmp += boardSample[i];
        }
        for (i=1; i<8; i+=2){
            tmp -= boardSample[i];
        }
        rollL = tmp / 8.;

        tmp = 0;
        for (i=8; i<16; i+=2){
            tmp += boardSample[i];
        }
        for (i=9; i<16; i+=2){
            tmp -= boardSample[i];
        }
        rollR = tmp / 8.;

        cout << "rolls " << rollL << " " << rollR << ";" << endl;

    // pitch

        tmp = 0;
        tmp += -2*(boardSample[0] + boardSample[1])
        tmp += -1*(boardSample[2] + boardSample[3])
        tmp +=  1*(boardSample[4] + boardSample[5])
        tmp +=  2*(boardSample[6] + boardSample[7])
        pitchL = tmp / 8.;

        tmp = 0;
        tmp += -2*(boardSample[8] + boardSample[9])
        tmp += -1*(boardSample[10] + boardSample[11])
        tmp +=  1*(boardSample[12] + boardSample[13])
        tmp +=  2*(boardSample[14] + boardSample[15])
        pitchR = tmp / 8.;

        cout << "pitches " << pitchL << " " << pitchR << ";" << endl;

}

void sendBoardSample(int *boardSample)
{
    int i;
    cout << "bs ";
    for (i=0; i<16; i++){
        cout << boardSample[i] << " ";
    }
    cout << ";" << endl;
}

void printBackground(int *bg)
{
    int i;
    cout << "bg ";
    for (i=0; i<16; i++){
        cout << bg[i] << " ";
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


void toggleLEDs(int on)
{
    if (on) {
        bcm2835_gpio_write(GPIO_LEFT, HIGH);
        bcm2835_gpio_write(GPIO_RIGHT, HIGH);
    } else {
        bcm2835_gpio_write(GPIO_LEFT, LOW);
        bcm2835_gpio_write(GPIO_RIGHT, LOW);
    }
}

int main(void)
{

    // initialize GPIO control
    if (!bcm2835_init()) return 1;
    bcm2835_gpio_fsel(GPIO_LEFT, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(GPIO_RIGHT, BCM2835_GPIO_FSEL_OUTP);

    // board sample structures
    int background[16];
    int rawData[16];
    int doubleBuffer[32];

    // warm up (still necessary?)
    for (int i=0; i<4; i++){
        takeBoardSample(rawData);
        cycleIndices();
    }

    // main loop
    while(1){

        toggleLEDs(0);
        takeBoardSample(background);
        toggleLEDs(1);
        takeBoardSample(rawData);

        cookData(background, rawData, doubleBuffer+iCurrent*16, 26);
        sendBoardSample(doubleBuffer+iCurrent*16);
        //printBackground(background);

        calculate_cof(doubleBuffer+iCurrent*16);
        sendCOF();

        computeHarmonics(doubleBuffer+iCurrent*16);

        followUpPotentialHits(doubleBuffer);
        findPotentialHits(doubleBuffer);

        cycleIndices();
        usleep(15000); // 15 ms
    }

    // timing loop
    /*
    time_t t1, t2;
    double seconds;
    int i;
    time(&t1);
    for (i=0; i<10000; i++){
        toggleLEDs(0);
        takeBoardSample(background);
        toggleLEDs(1);
        takeBoardSample(rawData);
    } 
    time(&t2);
    seconds = difftime(t2,t1);
    cout << "Time elapsed in seconds: " << seconds << endl;
    */

    return 0;

}
