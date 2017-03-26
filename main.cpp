/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: matthieu
 *
 *  Created on March 6, 2017, 4:42 PM
 */

#include <stdio.h>
#include <time.h>
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
#include "Odometry_gr1.hpp"
using namespace std;

struct globalStruct {
    CtrlStruct *MyStruct;
    MyMotors motorR;
    MyMotors motorL;
    MyOdometers odoR;
    MyOdometers odoL;
    pthread_mutex_t *mutex;
    
};

void * ThreadMotorR(void *atab)
{ 
    globalStruct *In_pthread = (globalStruct*) atab;
    while(1)
    {
        if(1)//pthread_mutex_lock(In_pthread->mutex)==0)
        {
            In_pthread->MyStruct->inputs->r_wheel_speed = (In_pthread->motorR.getSpeed()); 
            if(1)//pthread_mutex_unlock(In_pthread->mutex) ==0)
            {
                printf("%f \t",In_pthread->MyStruct->inputs->r_wheel_speed);
            }
            else
            {
               printf("ERROR : mutex unlocking failed \n "); 
            }
        }
        else
        {
            printf("ERROR : mutex locking failed \n ");
        }
    }
}
void * ThreadMotorL(void *atab)
{ 
    globalStruct *In_pthread = (globalStruct*) atab;
    while(1)
    {
        if(1)//pthread_mutex_lock(In_pthread->mutex)==0)
        {
            In_pthread->MyStruct->inputs->l_wheel_speed = (In_pthread->motorL.getSpeed()); 
            if(1)//pthread_mutex_unlock(In_pthread->mutex) ==0)
            {
                printf("%f \t",In_pthread->MyStruct->inputs->l_wheel_speed);
            }
            else
            {
               printf("ERROR : mutex unlocking failed \n "); 
            }
        }
        else
        {
            printf("ERROR : mutex locking failed \n");
        }
    }
}

/*void * ThreadOdoL(void *atab)
{ 
    globalStruct *In_pthread = (globalStruct*) atab;
    while(1)
    {
        if(pthread_mutex_lock(In_pthread->mutex)==0)
        {
            In_pthread->MyStruct->inputs->l_odo_speed = (In_pthread->odoL.getOdometersSpeed()); 
            if(pthread_mutex_unlock(In_pthread->mutex) ==0)
            {
                //printf("%f \t",In_pthread->MyStruct->inputs->r_odo_speed);
            }
            else
            {
               printf("ERROR : mutex unlocking failed \n "); 
            }
        }
        else
        {
            printf("ERROR : mutex locking failed \n ");
        }
    }
}

void * ThreadOdoR(void *atab)
{ 
    globalStruct *In_pthread = (globalStruct*) atab;
    while(1)
    {
        if(pthread_mutex_lock(In_pthread->mutex)==0)
        {
            In_pthread->MyStruct->inputs->r_odo_speed = (In_pthread->odoR.getOdometersSpeed()); 
            if(pthread_mutex_unlock(In_pthread->mutex) ==0)
            {
               // printf("%f \t",In_pthread->MyStruct->inputs->r_odo_speed);
            }
            else
            {
               printf("ERROR : mutex unlocking failed "); 
            }
        }
        else
        {
            printf("ERROR : mutex locking failed ");
        };
    }
}

void * ThreadComputePosition(void *atab)
{ 
    globalStruct *In_pthread = (globalStruct*) atab;
    int counter = 0; // counter is used for the first iteration trough the loop 
    long int time_start, time_end;
    struct timeval  tv;
    while(1)
    {
        gettimeofday(&tv, NULL);
        time_start =  (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
        if(counter ==0)
        {
            if(pthread_mutex_lock(In_pthread->mutex)==0)
            {
                xsiRWheels(In_pthread->MyStruct,0);
                computePosition(In_pthread->MyStruct); 
                if(pthread_mutex_unlock(In_pthread->mutex) ==0) {}   
                else
                {
                    printf("ERROR : mutex unlocking failed "); 
                }
            }
            else
            {
                printf("ERROR : mutex locking failed ");
            }
        }
        else
        {
            if(pthread_mutex_lock(In_pthread->mutex)==0)
            {
                xsiRWheels(In_pthread->MyStruct,0);
                computePosition(In_pthread->MyStruct); 
                if(pthread_mutex_unlock(In_pthread->mutex) ==0) {}   
                else
                {
                    printf("ERROR : mutex unlocking failed "); 
                }
            }
            else
            {
                printf("ERROR : mutex locking failed ");
            }
            
        }
        
        
   
    }
}*/



int main(int argc, char** argv) {
    printf("Hello from main test \n");
    gpioInitialise();
    //initialization of CAN bus 
    MyMCP2515 *MyCAN = new MyMCP2515();
    MyCAN->doInit();
    //initialization of DE0 nao
    MyDE0Nano *nano = new MyDE0Nano();
    nano->reset();
    //creation of object tourelle
    MyTourelle tourelle(MyCAN, nano, 0x508);
    
    //Creation and initialization of object motors for RIGHT MOTOR
    MyMotors motorsright(MyCAN, nano, 0x708, 1);
    motorsright.initMotor();
    
    //Creation and initialization of object motors for LEFT MOTOR
    MyMotors motorsleft(MyCAN, nano, 0x708, 2);
    motorsleft.initMotor();
    
    //Creation and initailization of Odometer RIGHT & LEFT
    MyOdometers odoright(nano, 1);
    MyOdometers odoleft(nano, 2);
    
    // Creation and initialization of vannes
    MyVannes electrovannes(MyCAN, 0x408);
    
    
    CtrlIn *In;
    CtrlOut *Out;
    
    
    In = (CtrlIn*) malloc(sizeof(CtrlIn));
    Out = (CtrlOut*) malloc(sizeof(CtrlOut));

    In->r_wheel_speed = 0.0;
    In->l_wheel_speed = 0.0;
    printf("Hello from main 2 \n");
    CtrlStruct *MyStruct;
    globalStruct *atab = (globalStruct*) malloc(sizeof(globalStruct));
    MyStruct = init_CtrlStruct(In,Out);
    printf("Hello from main 3\n");
    atab->motorR = motorsright;
    atab->motorL = motorsleft;
    atab->MyStruct = MyStruct;
    
    pthread_t threadMotorRight,threadMotorLeft, threadOdoRight, threadOdoLeft, threadPosCompute;
    int retRight = pthread_create(&threadMotorRight, NULL, ThreadMotorR, (void*)atab);
    int retLeft = pthread_create(&threadMotorLeft, NULL, ThreadMotorL, (void*)atab);
    int retOdoRight = 0;//pthread_create(&threadOdoRight, NULL, ThreadOdoR, (void*)atab);
    int retOdoLeft = 0;//pthread_create(&threadOdoLeft, NULL, ThreadOdoL, (void*)atab);
    int retPosCompute = 0;//pthread_create(&threadPosCompute, NULL, ThreadComputePosition, (void*)atab);
    if(retRight || retLeft || retOdoRight || retOdoLeft || retPosCompute)
    {
        printf("failed in one/several thread creation\n");
    }
    printf("Hello from main 4\n");
    double *KpKi = Kp_Ki_Computation(0.01,0.01);
    printf("Kp = %f \n", KpKi[0]);
    printf("Ki = %f \n", KpKi[1]);
    MiddleLevelController(0.3,0.0,MyStruct->struct_control->Speed_ref);
    double duration;
    clock_t start,end;
    while(1)
     { 
     start = clock(); 
     LowLevelController(MyStruct,MyStruct->struct_control->Speed_ref,1*KpKi[0],1*KpKi[1],MyStruct->struct_control->command);
     motorsright.setSpeed(MyStruct->struct_control->command[0]);
     //printf("%f \n",MyStruct->struct_control->command[0]);
     motorsleft.setSpeed(MyStruct->struct_control->command[1]);
     //printf("Speed ref right : %f\n",MyStruct->struct_control->Speed_ref[0]);
     //odoright.getOdometersPosition();
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

