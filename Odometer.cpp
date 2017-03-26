/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Motors.cpp
 * Author: matthieu
 *
 * Created on March 6, 2017, 4:42 PM
 */
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <iostream>
#include <wiringPiSPI.h>
#include <pigpio.h>
#include <bitset>
#include <ctime>
#include <math.h>
#include <sys/time.h>
#include "Odometer.h"
#include "MyMCP2515.h"
#include "MyDE0Nano.h"
#include "tourelle.h"
#include "globals.h"
#include "Motors.h"

#define PI 3.141592653
#define nTicksTour 2708.0

/*buld of the motor. address corresponds to the one of the CAN's PCB (ex: Minibot = 0x708)*/
MyOdometers::MyOdometers( MyDE0Nano *nano, int side)
{
    this_nano = nano;
    this_side = side;// 1 for the right motor, 2 for the left motor
    
}
MyOdometers::~MyOdometers() // destructeur de la classe odometer 
{
    
}
    
double MyOdometers::getOdometersPosition()
{
    double nTicks;
    //unsigned char buf[4] = {0x00, 0x00, 0x00, 0x00};
    char buf[4] = {0x00, 0x00, 0x00, 0x00};
    if(this_side == 1) {
    makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
    this_nano->readWriteReg(READ, 0x09, (signed char*)buf, 4); // register read of PosEgdeTicks
    int right_odometer = spi2data(buf); // converting char value into int value //minus sign for MW
    nTicks = right_odometer; 
    printf("right odometer position : %f\n",(((double) nTicks)/nTicksTour*360));
    //printf("right odometer position : %f\n",(((double) nTicks)));
    }
    else if (this_side == 2) {
    makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
    this_nano->readWriteReg(READ, 0x08, (signed char*)buf, 4);
    int left_odometer = spi2data(buf); // register read of PosEgdeTicks //minus sign for MW
    nTicks = left_odometer;  
    printf("Left odometer position : %f\n",(((double) nTicks)/nTicksTour*360));
    //printf("Left odometer position : %f\n",(((double) nTicks)));
    }
    return nTicks/nTicksTour*360;
    // dist parcourue et vitesse
}

double MyOdometers::getOdometersSpeed()
{
    return 0;
}
/*double MyMotors::getSpeed() {
    double nTicks;
    double pos1,pos2;
    //unsigned char buf[4] = {0x00, 0x00, 0x00, 0x00};
    char buf[4] = {0x00, 0x00, 0x00, 0x00};
    if(this_motor == 1) {
    makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
    this_nano->readWriteReg(READ, 0x01, (signed char*)buf, 4); // register read of PosEgdeTicks
    int right_wheel = -spi2data(buf); // converting char value into int value //minus sign for MW
    //printf("Right wheel : %f\n",(((double)right_wheel)/nTicksTour)*360.0);
    nTicks = right_wheel; 
    }
    else if (this_motor == 2) {
    makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
    this_nano->readWriteReg(READ, 0x02, (signed char*)buf, 4);
    int left_wheel = -spi2data(buf); // register read of PosEgdeTicks //minus sign for MW
    //printf("Left wheel : %f\n",(((double)left_wheel)/nTicksTour)*360.0);
    nTicks = left_wheel;
    }
    pos1 = nTicks;
    
    time_sleep(0.01);
    
    if(this_motor == 1) {
    makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
    this_nano->readWriteReg(READ, 0x01, (signed char*)buf, 4); // register read of PosEgdeTicks
    int right_wheel = -spi2data(buf); // converting char value into int value //minus sign for MW
    //printf("Right wheel : %f\n",(((double)right_wheel)/nTicksTour)*360.0);
    nTicks = right_wheel; 
    }
    else if (this_motor == 2) {
    makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
    this_nano->readWriteReg(READ, 0x02, (signed char*)buf, 4);
    int left_wheel = -spi2data(buf); // register read of PosEgdeTicks //minus sign for MW
    //printf("Left wheel : %f\n",(((double)left_wheel)/nTicksTour)*360.0);
    nTicks = left_wheel; 
    }
    pos2 = nTicks;
    //printf("Speeeed : %f\n",(pos2-pos1)*2*PI/(0.005*nTicksTour));
    
    return (pos2-pos1)*2*PI/(0.01*nTicksTour); // minus sign for MW only 
}*/

