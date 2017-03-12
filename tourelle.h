/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   tourelle.h
 * Author: GuiP
 *
 * Created on 1 mars 2017, 0:32
 */

#ifndef TOURELLE_H
#define TOURELLE_H

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
#define mask_led 0x40 

class MyTourelle{
public: 
    MyTourelle(MyMCP2515 *myCan, MyDE0Nano *nano, int address);
    virtual ~MyTourelle();
    void setBrake(bool activate);
    void setLed(bool activate);
    void setSpeed(int speed);
    int getNumberBeacon();
    double *getDataBeacon();
    int nb_falling();
    int nb_rising();
private:
    MyMCP2515 *this_can;
    MyDE0Nano *this_nano;
    int this_address;
    int ticks_number_per_rotation;
    double beacon_radius;
    char data[3];
};

#endif /* TOURELLE_H */

