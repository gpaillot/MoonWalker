/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   globals.h
 * Author: verbi
 *
 * Created on 7 décembre 2016, 14:06
 */

#ifndef GLOBALS_H
#define GLOBALS_H
void makeData( char* data,  char data0,  char data1, 
			 char data2, char data3, bool forCAN);
int spi2data (char* data);
int maxdata(int data1, int data2);




#endif /* GLOBALS_H */

