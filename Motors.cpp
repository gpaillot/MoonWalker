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
#include <time.h>
#include "MyMCP2515.h"
#include "MyDE0Nano.h"
#include "tourelle.h"
#include "globals.h"
#include "Motors.h"

#define PI 3.141592653
#define nTicksTour 9500.0

/*buld of the motor. address corresponds to the one of the CAN's PCB (ex: Minibot = 0x708)*/
MyMotors::MyMotors(MyMCP2515 *myCan, MyDE0Nano *nano, int address, int motor)
{
    this_can = myCan;
    this_nano = nano;
    this_address = address;
    this_motor = motor; // 1 for the right motor, 2 for the left motor
    
}
MyMotors::~MyMotors() 
{
    // Rien à faire dans le destructeur... 
}

/*set the brake if activate is true, release them is it is false*/
void MyMotors::setBrake(bool activate) {
    int this_mask;
    if (this_motor == 2){
        this_mask = mask_brake2;
    }
    else if (this_motor == 1){
        this_mask = mask_brake; 
    }
    if(activate)
    {
        makeData(data,GPLAT+offset,this_mask,this_mask,0x00,true);
        this_can->doSendMsg(this_address,data,3,0x00);
    }
    else
    {
        makeData(data,GPLAT+offset,this_mask,0x00,0x00,true);
        this_can->doSendMsg(this_address,data,3,0x00);
    }
}

/*turn on the led if activate is true, off if false */
void MyMotors::setLed(bool activate){
    
    if(activate)
    {
        makeData(data,GPLAT+offset,mask_led,mask_led,0x00,true);
        this_can->doSendMsg(this_address,data,3,0x00);
    }
    else
    {
        makeData(data,GPLAT+offset,mask_led,0x00,0x00,true);
        this_can->doSendMsg(this_address,data,3,0x00);
    }
}

/*set the speed of the tourelle (positive: CCW, negative:CW)*/
void MyMotors::setSpeed(int speed){
    
    if(this_motor ==1)
    {
    /*makeData(data,GPLAT+offset, mask_brake,0x00,0x00,true);// release brake
    this_can->doSendMsg(this_address,data,3,0x00);
    time_sleep(0.001);
    
    makeData(data,T1CON+offset, 0xb3,0x80,0x00,true);//T1CON
    this_can->doSendMsg(this_address,data, 3,0x00); 
    time_sleep(0.001);
    
    makeData(data,PR1+offset, 0xff, 0xff,0x00,true);//PR1
    this_can->doSendMsg(this_address,data, 3,0x00);
    time_sleep(0.001);*/
    
    makeData(data,PWM1+offset, 0xff,duty_zero + speed,0x00,true);//PWM1 set to speed
    this_can->doSendMsg(this_address,data, 3,0x00);
    time_sleep(0.001);
    }
    else if (this_motor == 2)
    {
    speed = -speed; // For MoonWalker only 
    /*makeData(data,GPLAT+offset, mask_brake2,0x00,0x00,true);// release brake
    this_can->doSendMsg(this_address,data,3,0x00);
    time_sleep(0.001);
    
    makeData(data,T2CON+offset, 0xb3,0x80,0x00,true);//T1CON
   this_can->doSendMsg(this_address,data, 3,0x00); 
    time_sleep(0.001);
    
    makeData(data,PR2+offset, 0xff, 0xff,0x00,true);//PR1
    this_can->doSendMsg(this_address,data, 3,0x00);
    time_sleep(0.001);*/
    
    makeData(data,PWM2+offset, 0xff,duty_zero + speed,0x00,true);//PWM1 set to speed
    this_can->doSendMsg(this_address,data, 3,0x00);
    time_sleep(0.001);
        
    }
}  
    
double MyMotors::getPosition()
{
    double nTicks;
    //unsigned char buf[4] = {0x00, 0x00, 0x00, 0x00};
    char buf[4] = {0x00, 0x00, 0x00, 0x00};
    if(this_motor == 1) {
    makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
    this_nano->readWriteReg(READ, 0x02, (signed char*)buf, 4); // register read of PosEgdeTicks
    int right_wheel = spi2data(buf); // converting char value into int value //minus sign for MW
    nTicks = right_wheel; 
    printf("Right wheel position : %f\n",(((double) nTicks)/nTicksTour*360));
    }
    else if (this_motor == 2) {
    makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
    this_nano->readWriteReg(READ, 0x01, (signed char*)buf, 4);
    int left_wheel = -spi2data(buf); // register read of PosEgdeTicks //minus sign for MW
    nTicks = left_wheel;  
    printf("Left wheel position : %f\n",(((double) nTicks)/nTicksTour*360));
    }
    return -nTicks/nTicksTour*360;
    // dist parcourue et vitesse
}

double MyMotors::getSpeed() {
    double nTicks;
    double pos1,pos2;
    //unsigned char buf[4] = {0x00, 0x00, 0x00, 0x00};
    char buf[4] = {0x00, 0x00, 0x00, 0x00};
    if(this_motor == 1) {
    makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
    this_nano->readWriteReg(READ, 0x02, (signed char*)buf, 4); // register read of PosEgdeTicks
    int right_wheel = -spi2data(buf); // converting char value into int value //minus sign for MW
    //printf("Right wheel : %f\n",(((double)right_wheel)/nTicksTour)*360.0);
    nTicks = right_wheel; 
    }
    else if (this_motor == 2) {
    makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
    this_nano->readWriteReg(READ, 0x01, (signed char*)buf, 4);
    int left_wheel = spi2data(buf); // register read of PosEgdeTicks //minus sign for MW
    //printf("Left wheel : %f\n",(((double)left_wheel)/nTicksTour)*360.0);
    nTicks = left_wheel;
    }
    pos1 = nTicks;
    
    time_sleep(0.01);
    
    if(this_motor == 1) {
    makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
    this_nano->readWriteReg(READ, 0x02, (signed char*)buf, 4); // register read of PosEgdeTicks
    int right_wheel = -spi2data(buf); // converting char value into int value //minus sign for MW
    //printf("Right wheel : %f\n",(((double)right_wheel)/nTicksTour)*360.0);
    nTicks = right_wheel; 
    }
    else if (this_motor == 2) {
    makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
    this_nano->readWriteReg(READ, 0x01, (signed char*)buf, 4);
    int left_wheel = spi2data(buf); // register read of PosEgdeTicks //minus sign for MW
    //printf("Left wheel : %f\n",(((double)left_wheel)/nTicksTour)*360.0);
    nTicks = left_wheel; 
    }
    pos2 = nTicks;
    //printf("Speeeed : %f\n",(pos2-pos1)*2*PI/(0.005*nTicksTour));
    
    return -(pos2-pos1)*2*PI/(0.01*nTicksTour); // minus sign for MW only 
}

double MyMotors::getSpeedWithMean(int number_of_sample) 
{
    printf("getSpeedWithMean \n");
    double pos1, pos2;
    long int time_start, time_end;
    struct timeval  tv;
    int address_of_SPI, i, speed;
    double diff_pos = 0.0;
    if(this_motor == 1)
    {
        address_of_SPI = 0x02;
    }
    else if(this_motor ==2)
    {
        address_of_SPI = 0x01;
    }
    else 
    {
        printf("This motor is not defined");
        i = number_of_sample+1;
    }

    //unsigned char buf[4] = {0x00, 0x00, 0x00, 0x00};
    char buf[4] = {0x00, 0x00, 0x00, 0x00};
    
    for(i=0;i<number_of_sample; i++)
    {
        
        makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
        this_nano->readWriteReg(READ, address_of_SPI, (signed char*)buf, 4);
        gettimeofday(&tv, NULL);
        time_start =  (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;// register read of PosEgdeTicks
        speed = spi2data(buf); // converting char value into int value //minus sign for MW
        pos1 = (double) speed; 

        time_sleep(0.01);
        
        makeData(buf, 0x00, 0x00, 0x00, 0x00, false);
        this_nano->readWriteReg(READ, address_of_SPI, (signed char*)buf, 4);
        gettimeofday(&tv, NULL);
        time_end =  (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;// register read of PosEgdeTicks
        speed = spi2data(buf); // converting char value into int value //minus sign for MW
        pos2 = (double) speed; 
        
        diff_pos +=(pos2-pos1)*2*PI/((time_end-time_start)*nTicksTour);

    }
    return (diff_pos/number_of_sample); // minus sign for MW only 
}

void MyMotors::initMotor()
{
    
    if(this_motor ==1)
    {
    makeData(data,GPLAT+offset, mask_brake,0x00,0x00,true);// release brake
    this_can->doSendMsg(this_address,data,3,0x00);
    time_sleep(0.001);
    
    makeData(data,T1CON+offset, 0xb3,0x80,0x00,true);//T1CON
    this_can->doSendMsg(this_address,data, 3,0x00); 
    time_sleep(0.001);
    
    makeData(data,PR1+offset, 0xff, 0xff,0x00,true);//PR1
    this_can->doSendMsg(this_address,data, 3,0x00);
    time_sleep(0.001);
    }
    else if (this_motor == 2)
    {
    makeData(data,GPLAT+offset, mask_brake2,0x00,0x00,true);// release brake
    this_can->doSendMsg(this_address,data,3,0x00);
    time_sleep(0.001);
    
    makeData(data,T2CON+offset, 0xb3,0x80,0x00,true);//T1CON
   this_can->doSendMsg(this_address,data, 3,0x00); 
    time_sleep(0.001);
    
    makeData(data,PR2+offset, 0xff, 0xff,0x00,true);//PR1
    this_can->doSendMsg(this_address,data, 3,0x00);
    time_sleep(0.001);
    
    }
}  
