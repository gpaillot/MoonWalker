/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Electrovannes.h
 * Author: matthieu
 *
 * Created on March 20, 2017, 7:43 PM
 */

#ifndef ELECTROVANNES_H
#define ELECTROVANNES_H

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
#define mask_led_vanne 0x40 
#define mask_vanne_1 0x01
#define mask_vanne_2 0x02
#define mask_vanne_3 0x04
#define mask_vanne_4 0x08


class MyVannes{
public: 
    MyVannes(MyMCP2515 *myCan, int address);
    virtual ~MyVannes();
    void setVanne(int vanne);
    void setLed(bool activate);
private:
    MyMCP2515 *this_can;
    int this_address;
    char data[3];
};


#endif /* ELECTROVANNES_H */

