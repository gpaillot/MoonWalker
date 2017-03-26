/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Motors.h
 * Author: matthieu
 *
 * Created on March 6, 2017, 2:49 PM
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "MyMCP2515.h"
#include "MyDE0Nano.h"

#define offset 0x1c
#define duty_zero 128
/*Addresses of users registers of MCP2502*/
#define GPLAT 0x02
#define T1CON 0x05
#define T2CON 0x06
#define PR1 0x07
#define PR2 0x08
#define PWM1 0x09 
#define PWM2 0x0a

/*Some mask for users registers*/
#define mask_brake 0x10  
#define mask_brake2 0x20
#define mask_led 0x40 

class MyMotors{
public: 
    MyMotors(MyMCP2515 *myCan, MyDE0Nano *nano, int address, int motor); // constructeur de l'objet 
    virtual ~MyMotors(); // destructeur de l'objet 
    void setBrake(bool activate);
    void setLed(bool activate);
    void setSpeed(int speed);
    double getPosition();
    double getSpeed();
    double getSpeedWithMean(int number_of_sample);
    void initMotor(); 
private:
    MyMCP2515 *this_can;
    MyDE0Nano *this_nano;
    int this_address;
    int this_motor;
    char data[3];
};


#endif /* MOTORS_H */

