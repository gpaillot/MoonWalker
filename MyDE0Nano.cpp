/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MyDE0Nano.cpp
 * Author: verbi
 * 
 * Created on 6 décembre 2016, 14:52
 */

#include "MyDE0Nano.h"


#define DEO_NANO_RESET 19 // or 4 ??

#define CHANNEL 0
#define FREQ 500000

#define HIGH 1
#define LOW 0

using namespace std;


MyDE0Nano::MyDE0Nano() {
	if(wiringPiSPISetup(CHANNEL, FREQ) ==-1){
        cout << "Problem with winringPI";
    }
    if (gpioInitialise() < 0)
    {
        fprintf(stderr, "pigpio initialisation failed\n");
    }
    
	m_msg_length =4; 
	m_spi_register_length = 16;
	m_spi_register_length_mask = 0b00001111;
	
	reset();
    return;
}

MyDE0Nano::~MyDE0Nano() {
}

void MyDE0Nano::readWriteReg(DE0Nano_action action, DE0Nano_spi address, signed char* buf, int length)
{
	readWriteReg(action, (unsigned char) address, buf, length);
}

void MyDE0Nano::readWriteReg(DE0Nano_action action, unsigned char address, signed char* buf, int length)
{
	if (length != m_msg_length)
	{
		cout << "ERROR: MyDEONano.cpp - Please provide a buffer that has the good length" << endl;
	}
	unsigned char in_buf[m_msg_length+1];
	if(action == WRITE)
		in_buf[0] = (unsigned char) 0b10000000 + (unsigned char) (address & m_spi_register_length_mask);
	else
		in_buf[0] = (unsigned char) 0b00000000 + (unsigned char) (address & m_spi_register_length_mask);
	
	for (int i=1; i< m_msg_length+1; i++)
	{
		in_buf[i] = buf[i-1];
	}

    if (!wiringPiSPIDataRW(CHANNEL, in_buf, m_msg_length+1))
	{
		cout << "ERROR: MyDE0Nano.cpp - Could not send data over SPI" << endl;
	}
	if(action == READ)
	{
		for (int i=1; i< m_msg_length+1; i++)
		{
			buf[i-1] = in_buf[i];
		}
	}

	return;
}

void MyDE0Nano::reset()
{
	gpioWrite(DEO_NANO_RESET, HIGH); /* on */
	time_sleep(0.1);
    gpioWrite(DEO_NANO_RESET, LOW); /* on */
	time_sleep(0.1);
}

