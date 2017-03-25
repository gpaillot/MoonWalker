/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: GuiP
 *
 * Created on 28 février 2017, 23:35
 */

#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <iostream>
#include <wiringPiSPI.h>
#include <pigpio.h>
#include <bitset>
#include <ctime>
#include <sys/time.h>
#include <unistd.h>
#include "MyMCP2515.h"
#include "MyDE0Nano.h"
#include "Motors.h"
#include "Controllers_gr1.hpp"
#include "CtrlStruct_gr1.h"
#include "initTourelle.h"
#include "stop.h"
#include "globals.h"
#include "tourelle.h"
#include "ctrl_io.h"
#include "Wheels_gr1.hpp"
#include <pthread.h>
#include <time.h>
#include "Electrovannes.h"
#include "Odometer.h"
#include "MyIncludes_gr1.h"
using namespace std;

/*
 * 
 * 
 * 
 */
struct args {
    CtrlStruct *MyStruct;
    MyMotors motorR;
    MyMotors motorL;
    
};

void * ThreadMotorR(void *atab)
{ 
    args *In_pthread = (args*) atab;
    while(1)
    {
        In_pthread->MyStruct->inputs->r_wheel_speed = (In_pthread->motorR.getSpeed()); 
        //printf("%f \t",In_pthread->MyStruct->inputs->r_wheel_speed);
    }
}
void * ThreadMotorL(void *atab)
{ 
    args *In_pthread = (args*) atab;
    while(1)
    {
        In_pthread->MyStruct->inputs->l_wheel_speed = (In_pthread->motorL.getSpeed());
        //printf("%f \n",In_pthread->MyStruct->inputs->l_wheel_speed);
    }
}


int main(int argc, char** argv) {
    
    gpioInitialise();
    MyMCP2515 *MyCAN = new MyMCP2515();
    MyCAN->doInit();
    MyDE0Nano *nano = new MyDE0Nano();
    args *atab = (args*) malloc(sizeof(args));
    MyTourelle tourelle(MyCAN, nano, 0x508);
    MyMotors motorsright(MyCAN, nano, 0x708,1);
    MyMotors motorsleft(MyCAN, nano, 0x708,2);
    
    MyOdometers odoright(nano,1);
    
    MyVannes electrovannes(MyCAN,0x408);
    nano->reset();
    
    CtrlIn *In;
    CtrlOut *Out;
    
    In = (CtrlIn*) malloc(sizeof(CtrlIn));
    Out = (CtrlOut*) malloc(sizeof(CtrlOut));
    
    CtrlStruct *MyStruct;
    In->r_wheel_speed = 0.0;
    In->l_wheel_speed = 0.0;
    
    double xsiR[2];
    
    MyStruct = init_CtrlStruct(In,Out);
    atab->motorR = motorsright;
    atab->motorL = motorsleft;
    atab->MyStruct = MyStruct;
    
    pthread_t threadMotorRight,threadMotorLeft;
    int retRight = pthread_create(&threadMotorRight, NULL, ThreadMotorR, (void*)atab);
    int retLeft = pthread_create(&threadMotorLeft, NULL, ThreadMotorL, (void*)atab);
    if(retRight)
    {
        printf("failed in thread for right motor creation\n");
    }
    if(retLeft)
    {
        printf("failed in thread for right motor creation\n");
    }
    double *KpKi = Kp_Ki_Computation(0.01,0.01);
    printf("Kp = %f \n", KpKi[0]);
    printf("Ki = %f \n", KpKi[1]);
    MiddleLevelController(0.3,0.0,MyStruct->struct_control->Speed_ref);
    double duration;
    clock_t start,end;
    while(MyStruct->)
     { 
     start = clock(); 
     LowLevelController(MyStruct,MyStruct->struct_control->Speed_ref,1*KpKi[0],1*KpKi[1],MyStruct->struct_control->command);
     motorsright.setSpeed(MyStruct->struct_control->command[0]);
     //printf("%f \n",MyStruct->struct_control->command[0]);
     motorsleft.setSpeed(MyStruct->struct_control->command[1]);
     //printf("Speed ref right : %f\n",MyStruct->struct_control->Speed_ref[0]);
     //odoright.getOdometersPosition();
     xsiR = xsiRWheels(MyStruct);
     computePosition(MyStruct,xsiR);
     //displayWheels(MyStruct);
     //printf("wheel_ref_gauche: %f \n",wheel_ref[1]);
     //printf("wheel_ref_droite: %f \n",wheel_ref[0]);
     //printf("commande_gauche: %f \n",commande_vitesse[1]);
     //printf("comande_droite: %f \n",commande_vitesse[0]);
     duration = (double) (clock()-start)/CLOCKS_PER_SEC;
     //printf("time duration = %f \n \n \n", duration);
     
     
     
     }
    
    
    int speedLeft = 0;
    int speedRight = 10;
    
    /*while(1)
    {
     electrovannes.setLed(true);
    time_sleep(2);
    electrovannes.setLed(false);
    time_sleep(2);
        
    }
    electrovannes.setLed(true);
    time_sleep(2);
    electrovannes.setLed(false);
    time_sleep(2);
    electrovannes.setLed(true);
    time_sleep(2);
    electrovannes.setLed(false);
    time_sleep(2);
    electrovannes.setLed(true);
    time_sleep(2);
    electrovannes.setLed(false);
    time_sleep(2);
    electrovannes.setLed(true);
    time_sleep(2);
    electrovannes.setLed(false);
  
    printf("SetVannes============================== \n");
    electrovannes.setVanne(4);
    time_sleep(2.98);*/
  



    
    
    
    
    
    
    //motorsright.setLed(true);
    //tourelle.setLed(false);
    //tourelle.setSpeed(-25);
    motorsleft.setSpeed(speedLeft);
    motorsright.setSpeed(speedRight);
    time_sleep(2.98);
    // motorsleft.getSpeed();
    //motorsright.getSpeed();
    //tourelle.setBrake(true);
    //time_sleep(1);
    motorsleft.setBrake(true);
    motorsright.setBrake(true);
    //motorsright.setBrake(true);
    motorsleft.getPosition();
    motorsright.getPosition();
    
    return 0;
}

