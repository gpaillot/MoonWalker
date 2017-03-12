

/* 
 * File:   main.cpp
 * Author: GuiP
 *
 * Created on 28 février 2017, 17:23
 */


#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <pigpio.h>
#include <ctime>
#include <sys/time.h>
#include "MyMCP2515.h"



using namespace std;

/*
 * 
 */
void stop() {
    
    MyMCP2515 *MyCAN = new MyMCP2515();
    MyCAN->doInit();
    
    
    //brake roues + tourelle
    char buf[] = {0x1E, 0x70,0x30};
    MyCAN->doSendMsg(0x508,buf, 3,0x00); // brake roues
    time_sleep(0.01);
    
    char buf1[] = {0x1E, 0x10,0x10};
    MyCAN->doSendMsg(0x708,buf1, 3,0x00); // brake tourelle
    time_sleep(0.01);
    
    
    //roues
    char buf2[] = {0x1E, 0x40,0x40};
    MyCAN->doSendMsg(0x708,buf2, 3,0x00); // led
    time_sleep(0.01);
    char buf3[] = {0x21, 0xb3,0x80};
    MyCAN->doSendMsg(0x708,buf3, 3,0x00); // T1CON (roue de gauche quand on lit bien l'écran) 
    time_sleep(0.01);
    char buf4[] = {0x22, 0xb3, 0x80};
    MyCAN->doSendMsg(0x708,buf4, 3, 0x00); // T2CON (roue de droite quand on lit bien l'écran)
    time_sleep(0.01);
    char buf5[] = {0x23, 0xff, 0xff};
    MyCAN->doSendMsg(0x708,buf5, 3, 0x00); // PR1
    time_sleep(0.01);
    char buf6[] = {0x24, 0xff, 0xff};
    MyCAN->doSendMsg(0x708,buf6, 3, 0x00); // PR2
    time_sleep(0.01);
    char buf7[] = {0x25, 0xff,0x80};
    MyCAN->doSendMsg(0x708,buf7, 3,0x00); // reset PWM1 left
    time_sleep(0.01);
    char buf8[] = {0x26, 0xff,0x80};
    MyCAN->doSendMsg(0x708,buf8, 3,0x00); // reset PWM1 left
    time_sleep(0.01);
    char buf666[] = {0x1E, 0x70,0x00};
    MyCAN->doSendMsg(0x508,buf666, 3,0x00); // retirer brake roues
    time_sleep(0.01);
    //MyCAN->doSendMsg(0x708,{0x1E, 0x70,0x70}, 3,0x00) // brake
    
    //tourelle
    char buf9[] = {0x21, 0xb3,0x80};
    MyCAN->doSendMsg(0x508,buf9, 3,0x00); // T1CON
    time_sleep(0.01);
    char buf10[] = {0x23, 0xff, 0xff};
    MyCAN->doSendMsg(0x508,buf10, 3, 0x00); // PR1
    time_sleep(0.01);
    char buf11[] = {0x25, 0xff,0x80}; // reset
    MyCAN->doSendMsg(0x508,buf11, 3,0x00);
    char buf12[] = {0x1E, 0x10,0x00};
    MyCAN->doSendMsg(0x708,buf12, 3,0x00); // retirer brake tourelle
    time_sleep(0.01);
    
    delete MyCAN;
    //delete nano;
    
}

void roue()
{
    MyMCP2515 *MyCAN = new MyMCP2515();
    MyCAN->doInit();
    gpioInitialise();
    
    char buf[] = {0x1E, 0x70,0x00};
    MyCAN->doSendMsg(0x708,buf, 3,0x00); // brake roues
    time_sleep(0.01);
    char buf3[] = {0x21, 0xb3,0x80};
    MyCAN->doSendMsg(0x708,buf3, 3,0x00); // T1CON (roue de gauche quand on lit bien l'écran) 
    time_sleep(0.01);
    char buf5[] = {0x23, 0xff, 0xff};
    MyCAN->doSendMsg(0x708,buf5, 3, 0x00); // PR1
    time_sleep(0.01);
    char buf7[] = {0x25, 0xff,160};
    MyCAN->doSendMsg(0x708,buf7, 3,0x00); // reset PWM1 left
    time_sleep(0.01);
    
    delete MyCAN;
}

