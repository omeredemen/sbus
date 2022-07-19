#include <cstdio>
#include <unistd.h>

#include "sbus.h"

sbus_bridge::SBusSerialPort sbus;

int main(){

    int cnt=0;
    if(!sbus.connectSerialPort()){
        printf("serial port couldn't open");
        return -1;
    }

    while(1){
    sbus.transmitSerialSBusMessage();
    usleep(7800); //7800; en az 2900
    //cnt++;
    }

    return 0;
}