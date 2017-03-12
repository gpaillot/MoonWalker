/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MyDE0Nano.h
 * Author: verbi
 *
 * Created on 6 décembre 2016, 14:52
 */

#ifndef MYDE0NANO_H
#define MYDE0NANO_H

#include <pigpio.h>
#include "MyMCP2515.h"

enum DE0Nano_action {READ, WRITE};
enum DE0Nano_spi {POSL = 0x00, POSR = 0x01, POST = 0x02, 
				  SPEEDL = 0x03, SPEEDR = 0x04, SPEEDT = 0x05};


class MyDE0Nano {
public:
	MyDE0Nano();
	virtual ~MyDE0Nano();
	void readWriteReg(DE0Nano_action action, DE0Nano_spi address, signed char* buf, int length);
	void readWriteReg(DE0Nano_action action, unsigned char address, signed char* buf, int length);
	void reset();
private:
	int m_msg_length;
	int m_spi_register_length;
	unsigned char m_spi_register_length_mask;

};

#endif /* MYDE0NANO_H */

