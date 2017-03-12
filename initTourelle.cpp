/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <pigpio.h>
#include <ctime>
#include <sys/time.h>
#include "MyMCP2515.h"
#include "initTourelle.h"


void initTourelle()
{
    MyMCP2515 *MyCAN = new MyMCP2515();
    MyCAN->doInit();
    time_sleep(1);
    gpioInitialise();
    
    /*
    int offset = 0x1C;
    bool activate = true;
    char buf_led[3];
    buf_led[0] = (char) offset+0x02;
    buf_led[1] = (char) 0b01000000;
    
    if(activate)
    {
        buf_led[2] = (char) 0b01000000;
    }
    else
    {
        buf_led[2] = (char) 0b00000000;
    }
    MyCAN->doSendMsg(0x708, buf_led, 3, 0x00); 
    */
    
     char buf[] = {0x1E, 0x10,0x00};
     MyCAN->doSendMsg(0x508,buf, 3,0x00); 
     time_sleep(0.01);
     
     char buf1[] = {0x21, 0xb3,0x80};
     MyCAN->doSendMsg(0x508,buf1, 3,0x00); 
     time_sleep(0.01);
     
     char buf2[] = {0x23, 0xff, 0xff};
     MyCAN->doSendMsg(0x508,buf2, 3,0x00);
     time_sleep(0.01);
     int command = 0;
     char buf11[] = {0x25, 0xff,128+30};
     MyCAN->doSendMsg(0x508,buf11, 3,0x00);
     
    delete MyCAN;
//    delete nano;
}