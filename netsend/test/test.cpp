#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>


using namespace std;

int main(void)
{
    int i;
    int inc=0;
    while(1){

        for (i=0; i<16; i++){
            if (i==15) {
                cout << inc*16 + i << ";\n";
            } else {
                cout << inc*16 + i << " ";
            }
        }
        cout.flush();
        inc++;
        usleep(1000000); // 1 s
    }

    return 0;

}
