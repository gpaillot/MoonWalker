/* 
 * File:   MyMCP2515.h
 * Author: verbi
 *
 * Created on 4 décembre 2016, 16:23
 */

#ifndef MYMCP2515_H
#define MYMCP2515_H


#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <iostream>
#include <wiringPiSPI.h>
#include <pigpio.h>

class MyMCP2515 {
public:
    MyMCP2515();
    virtual ~MyMCP2515();
    void doInit();
    void doDebug();
    void doSendMsg(int theIdentifier, char* theData, int theLength, char theFrameType);
    void doReceiveMsg();
    void doHandleIRQ(int thePin);
private:
    void writeCommand(char theCommand);
    char readCommand(char theCommand);
    void writeReg(char theAddress, char theData);
    char readReg(char theAddress);
};

#endif /* MYMCP2515_H */

