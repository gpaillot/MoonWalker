/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
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
#include "Electrovannes.h"

#define PI 3.141592653

MyVannes::MyVannes(MyMCP2515 *myCan, int address)
{
    this_can = myCan;
    this_address = address;// 1 for the right motor, 2 for the left motor
    
}
MyVannes::~MyVannes() 
{
    
}
   
void MyVannes::setLed(bool activate){
    
    if(activate)
    {
        makeData(data,GPLAT+offset,mask_led_vanne,mask_led_vanne,0x00,true);
        this_can->doSendMsg(this_address,data,3,0x00);
    }
    else
    {
        makeData(data,GPLAT+offset,mask_led_vanne,0x00,0x00,true);
        this_can->doSendMsg(this_address,data,3,0x00);
    }
}

void MyVannes::setVanne(int vanne){
    
    switch(vanne) {
        case 1:
            makeData(data,GPLAT+offset,mask_vanne_1,mask_vanne_1,0x00,true);
            this_can->doSendMsg(this_address,data,3,0x00);
            break;
        case 2:
            makeData(data,GPLAT+offset,mask_vanne_2,mask_vanne_2,0x00,true);
            this_can->doSendMsg(this_address,data,3,0x00);
            break;
        case 3:
            makeData(data,GPLAT+offset,mask_vanne_3,mask_vanne_3,0x00,true);
            this_can->doSendMsg(this_address,data,3,0x00);
            break;
        case 4:
            makeData(data,GPLAT+offset,mask_vanne_4,mask_vanne_4,0x00,true);
            this_can->doSendMsg(this_address,data,3,0x00);
            break;
            
    }
}
