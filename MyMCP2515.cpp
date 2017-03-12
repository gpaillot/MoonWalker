/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MyMCP2515.cpp
 * Author: verbi
 * 
 * Created on 4 décembre 2016, 16:23
 */

#include "MyMCP2515.h"

#define WAITING_TIME 0.001


// Pin Definition

// Pin Definition
#define MCP2515_RESET 23
#define MCP2515_INT   24

// Error Messages
#define MYCAN_ERR_INIT          "Unable to init MCP2515"
#define MYCAN_ERR_FRAME_TYPE    "Unknown frame type - unable to send msg"

// MCP2515 Registers
#define MCP2515_TX_STD_FRAME        0x00
#define MCP2515_TX_REMOTE_FRAME     0x01

// CAN configuration registers

#define MCP2515_BFPCTRL         0x0C
#define MCP2515_TXRTSCTRL       0x0D
#define MCP2515_CANSTAT         0x0E
#define MCP2515_CANCTRL         0x0F

#define MCP2515_TEC             0x1C
#define MCP2515_REC             0x1D

#define MCP2515_CNF3       0x28
#define MCP2515_CNF2       0x29
#define MCP2515_CNF1       0x2A

#define MCP2515_CANINTE    0x2B
#define MCP2515_CANINTF    0x2C
#define MCP2515_EFLG       0x2D

// CAN Receive Mask/Filter registers

#define MCP2515_RXM0SIDH   0x20
#define MCP2515_RXM0SIDL   0x21
#define MCP2515_RXM0EID8   0x22
#define MCP2515_RXM0EID0   0x23

#define MCP2515_RXM1SIDH   0x24
#define MCP2515_RXM1SIDL   0x25
#define MCP2515_RXM1EID8   0x26
#define MCP2515_RXM1EID0   0x27

#define MCP2515_RXF0SIDH   0x00
#define MCP2515_RXF0SIDL   0x01
#define MCP2515_RXF0EID8   0x02
#define MCP2515_RXF0EID0   0x03

#define MCP2515_RXF1SIDH   0x04
#define MCP2515_RXF1SIDL   0x05
#define MCP2515_RXF1EID8   0x06
#define MCP2515_RXF1EID0   0x07

#define MCP2515_RXF2SIDH   0x08
#define MCP2515_RXF2SIDL   0x09
#define MCP2515_RXF2EID8   0x0A
#define MCP2515_RXF2EID0   0x0B

#define MCP2515_RXF3SIDH   0x10
#define MCP2515_RXF3SIDL   0x11
#define MCP2515_RXF3EID8   0x12
#define MCP2515_RXF3EID0   0x13

#define MCP2515_RXF4SIDH   0x14
#define MCP2515_RXF4SIDL   0x15
#define MCP2515_RXF4EID8   0x16
#define MCP2515_RXF4EID0   0x17

#define MCP2515_RXF5SIDH   0x18
#define MCP2515_RXF5SIDL   0x19
#define MCP2515_RXF5EID8   0x1A
#define MCP2515_RXF5EID0   0x1B

// CAN Transmit Control/Header/Data registers

#define MCP2515_TXB0CTRL   0x30
#define MCP2515_TXB0SIDH   0x31
#define MCP2515_TXB0SIDL   0x32
#define MCP2515_TXB0EID8   0x33
#define MCP2515_TXB0EID0   0x34
#define MCP2515_TXB0DLC    0x35
#define MCP2515_TXB0D0     0x36
#define MCP2515_TXB0D1     0x37
#define MCP2515_TXB0D2     0x38
#define MCP2515_TXB0D3     0x39
#define MCP2515_TXB0D4     0x3A
#define MCP2515_TXB0D5     0x3B
#define MCP2515_TXB0D6     0x3C
#define MCP2515_TXB0D7     0x3D

#define MCP2515_TXB1CTRL   0x40
#define MCP2515_TXB1SIDH   0x41
#define MCP2515_TXB1SIDL   0x42
#define MCP2515_TXB1EID8   0x43
#define MCP2515_TXB1EID0   0x44
#define MCP2515_TXB1DLC    0x45
#define MCP2515_TXB1D0     0x46
#define MCP2515_TXB1D1     0x47
#define MCP2515_TXB1D2     0x48
#define MCP2515_TXB1D3     0x49
#define MCP2515_TXB1D4     0x4A
#define MCP2515_TXB1D5     0x4B
#define MCP2515_TXB1D6     0x4C
#define MCP2515_TXB1D7     0x4D

#define MCP2515_TXB2CTRL   0x50
#define MCP2515_TXB2SIDH   0x51
#define MCP2515_TXB2SIDL   0x52
#define MCP2515_TXB2EID8   0x53
#define MCP2515_TXB2EID0   0x54
#define MCP2515_TXB2DLC    0x55
#define MCP2515_TXB2D0     0x56
#define MCP2515_TXB2D1     0x57
#define MCP2515_TXB2D2     0x58
#define MCP2515_TXB2D3     0x59
#define MCP2515_TXB2D4     0x5A
#define MCP2515_TXB2D5     0x5B
#define MCP2515_TXB2D6     0x5C
#define MCP2515_TXB2D7     0x5D

// CAN Receive Control/Header/Data registers

#define MCP2515_RXB0CTRL   0x60
#define MCP2515_RXB0SIDH   0x61
#define MCP2515_RXB0SIDL   0x62
#define MCP2515_RXB0EID8   0x63
#define MCP2515_RXB0EID0   0x64
#define MCP2515_RXB0DLC    0x65
#define MCP2515_RXB0D0     0x66
#define MCP2515_RXB0D1     0x67
#define MCP2515_RXB0D2     0x68
#define MCP2515_RXB0D3     0x69
#define MCP2515_RXB0D4     0x6A
#define MCP2515_RXB0D5     0x6B
#define MCP2515_RXB0D6     0x6C
#define MCP2515_RXB0D7     0x6D

#define MCP2515_RXB1CTRL   0x70
#define MCP2515_RXB1SIDH   0x71
#define MCP2515_RXB1SIDL   0x72
#define MCP2515_RXB1EID8   0x73
#define MCP2515_RXB1EID0   0x74
#define MCP2515_RXB1DLC    0x75
#define MCP2515_RXB1D0     0x76
#define MCP2515_RXB1D1     0x77
#define MCP2515_RXB1D2     0x78
#define MCP2515_RXB1D3     0x79
#define MCP2515_RXB1D4     0x7A
#define MCP2515_RXB1D5     0x7B
#define MCP2515_RXB1D6     0x7C
#define MCP2515_RXB1D7     0x7D

#define MCP2515_CMD_RESET      0xC0
#define MCP2515_CMD_WRITE      0x02
#define MCP2515_CMD_READ       0x03
#define MCP2515_CMD_RTS        0x80
#define MCP2515_CMD_BITMOD     0x05
#define MCP2515_CMD_STATUS     0xA0
#define MCP2515_CMD_RXSTATUS   0xB0


// SPI connexion
#define CHANNEL 1
#define FREQ 500000

#define HIGH 1
#define LOW 0

using namespace std;


MyMCP2515::MyMCP2515() {
    if(wiringPiSPISetup(CHANNEL, FREQ) ==-1){
        cout << "Problem with winringPI";
    }
    if (gpioInitialise() < 0)
    {
        fprintf(stderr, "pigpio initialisation failed\n");
    }
    
    gpioWrite(MCP2515_RESET, HIGH); /* on */
    gpioWrite(MCP2515_RESET, LOW); /* on */
    gpioWrite(MCP2515_RESET, HIGH); /* on */
    
    this->doInit();
    return;
}

MyMCP2515::~MyMCP2515() {
}

void MyMCP2515::writeCommand(char theCommand) {
    unsigned char buf[] = {theCommand};
    wiringPiSPIDataRW(CHANNEL, buf, 1);

    //bcm2835_spi_transfern(buf, sizeof(buf));
}

char MyMCP2515::readCommand(char theCommand) {
    unsigned char buf[] = {theCommand, 0x00, 0x00};
    wiringPiSPIDataRW(CHANNEL, buf, 3);

    //bcm2835_spi_transfern(buf, sizeof(buf));
    char x = buf[1];         // ?? python: x = FromSPI[1]
    return x;
}

void MyMCP2515::writeReg(char theAddress, char theData) {    
    unsigned char buf[] = {(char) MCP2515_CMD_WRITE, theAddress, theData};
    wiringPiSPIDataRW(CHANNEL, buf, 3);
    //bcm2835_spi_transfern(buf, sizeof(buf));
}

char MyMCP2515::readReg(char theAddress) {
    unsigned char buf[] = {(char) MCP2515_CMD_READ, theAddress, 0x00};
    wiringPiSPIDataRW(CHANNEL, buf, 3);
    //bcm2835_spi_transfern(buf, sizeof(buf));
    char x = buf[2]; // python: FromSPI[2]
    return x;
}

void MyMCP2515::doInit() {
    // Software Reset
    this->writeCommand((char) MCP2515_CMD_RESET);

    // Set Configuration Mode
    this->writeReg((char) MCP2515_CANCTRL, 0x80);
    
    /*
    Bit Timing Configuration
        125 kHz CAN baud rate with Fosc = 16 MHz

        Fosc        = 16MHz
        BRP         =   7  (divide by 8)
        Sync Seg    = 1TQ
        Prop Seg    = 1TQ
        Phase Seg1  = 3TQ
        Phase Seg2  = 3TQ

        TQ = 2 * (1/Fosc) * (BRP+1) = 1us
        CAN Bit Time = (1+1+3+3) 1 us = 8 us -> 125kHz
        Bus speed = 1/(Total # of TQ) * TQ   

        The CAN Configuration Registers are then assembled as follows:
        CNF1 = SJW1 + (BRP-1) = 0x07
        CNF2 = (BTLMODE_SET + (PHSEG1-1)*8 + (PRSEG-1)) = 0x90
        CNF3 = (SOF_DISABLE + WAKFIL_DISABLE + (PHSEG2-1)) = 0x02
        */
    
    this->writeReg( (char) MCP2515_CNF1, 0x01); // 0x07 for minibot
    this->writeReg( (char) MCP2515_CNF2, 0x90);
    this->writeReg( (char) MCP2515_CNF3, 0x02);

    // Configure initialization of message transmission
    // TX0RTS, TX1RTS, TX2RTS not used : B0RTSM, B1RTSM, B2RTSM = 0

    this->writeReg( (char) MCP2515_TXRTSCTRL, 0x00) ;
      
    // Configure Masks and Filters

    this->writeReg( (char) MCP2515_RXM0SIDH, 0x00); 
    this->writeReg( (char) MCP2515_RXM0SIDL, 0x00); 
    this->writeReg( (char) MCP2515_RXM0EID8, 0x00); 
    this->writeReg( (char) MCP2515_RXM0EID0, 0x00);

    this->writeReg( (char) MCP2515_RXM1SIDH, 0x00);
    this->writeReg( (char) MCP2515_RXM1SIDL, 0x00);
    this->writeReg( (char) MCP2515_RXM1EID8, 0x00);
    this->writeReg( (char) MCP2515_RXM1EID0, 0x00); 

    this->writeReg( (char) MCP2515_RXF0SIDH, 0x00); 
    this->writeReg( (char) MCP2515_RXF0SIDL, 0x00); 
    this->writeReg( (char) MCP2515_RXF0EID8, 0x00); 
    this->writeReg( (char) MCP2515_RXF0EID0, 0x00); 

    this->writeReg( (char) MCP2515_RXF1SIDH, 0x00); 
    this->writeReg( (char) MCP2515_RXF1SIDL, 0x00); 
    this->writeReg( (char) MCP2515_RXF1EID8, 0x00); 
    this->writeReg( (char) MCP2515_RXF1EID0, 0x00); 

    this->writeReg( (char) MCP2515_RXF2SIDH, 0x00); 
    this->writeReg( (char) MCP2515_RXF2SIDL, 0x00); 
    this->writeReg( (char) MCP2515_RXF2EID8, 0x00); 
    this->writeReg( (char) MCP2515_RXF2EID0, 0x00); 

    this->writeReg( (char) MCP2515_RXF3SIDH, 0x00); 
    this->writeReg( (char) MCP2515_RXF3SIDL, 0x00); 
    this->writeReg( (char) MCP2515_RXF3EID8, 0x00); 
    this->writeReg( (char) MCP2515_RXF3EID0, 0x00); 

    this->writeReg( (char) MCP2515_RXF4SIDH, 0x00); 
    this->writeReg( (char) MCP2515_RXF4SIDL, 0x00); 
    this->writeReg( (char) MCP2515_RXF4EID8, 0x00); 
    this->writeReg( (char) MCP2515_RXF4EID0, 0x00); 

    this->writeReg( (char) MCP2515_RXF5SIDH, 0x00); 
    this->writeReg( (char) MCP2515_RXF5SIDL, 0x00); 
    this->writeReg( (char) MCP2515_RXF5EID8, 0x00); 
    this->writeReg( (char) MCP2515_RXF5EID0, 0x00);         

    // Set Normal Operation Mode
    this->writeReg( (char) MCP2515_CANCTRL, 0x00);

    // Configura Receive buffer RXB0 et RXB1
    // RXM = 11 : turn mask/filters off
    // BUKT = 1 : enable Rollover

    this->writeReg( (char) MCP2515_RXB0CTRL, 0xff);
    this->writeReg( (char) MCP2515_RXB1CTRL, 0xff);

    // Clear all interrupt flags
    // Disable all interrupts except
    //   Bit 7 MERRE Message Interrupt Enable
    //   Bit 2 TX0E Transmit Buffer 0 Empty
    //   Bit 1 RX1E Receive  Buffer 1 Full
    //   Bit 0 RX0E Receive  Buffer 0 Full 

    this->writeReg( (char) MCP2515_CANINTF, 0x00);
    this->writeReg( (char) MCP2515_CANINTE, 0x87);
}
  
void MyMCP2515::doDebug() {
    cout << "CNF1     = " << hex << (int) this->readReg( (char) MCP2515_CNF1) << std::endl;
    cout << "CNF2     = " << hex << (int) this->readReg( (char) MCP2515_CNF2) << endl;
    cout << "CNF3     = " << hex << (int) this->readReg( (char) MCP2515_CNF3) << endl;
    cout << "CANCTRL  = " << hex << (int) this->readReg( (char) MCP2515_CANCTRL) << endl;
    cout << "CANSTAT  = " << hex << (int) this->readReg( (char) MCP2515_CANSTAT) << endl;
    cout << "TXB0CTRL = " << hex << (int) this->readReg( (char) MCP2515_TXB0CTRL) << endl;
    cout << "TXB1CTRL = " << hex << (int) this->readReg( (char) MCP2515_TXB1CTRL) << endl;
    cout << "CANINTE  = " << hex << (int) this->readReg( (char) MCP2515_CANINTE) << endl;
    cout << "CANINTF  = " << hex << (int) this->readReg( (char) MCP2515_CANINTF) << endl;
    cout << "EFLG     = " << hex << (int) this->readReg( (char) MCP2515_EFLG) << endl;
    cout << "TEC      = " << hex << (int) this->readReg( (char) MCP2515_TEC) << endl;
    cout << "REC      = " << hex << (int) this->readReg( (char) MCP2515_REC) << endl;
        
    cout << "Status   = " << hex << (int) this->readCommand( (char) MCP2515_CMD_STATUS) << endl;
    cout << "RxStatus = " << hex << (int) this->readCommand( (char) MCP2515_CMD_RXSTATUS) << endl;
    cout << "Int      = " << hex << (int) gpioRead(MCP2515_INT) << endl;
}
        
void MyMCP2515::doSendMsg(int theIdentifier, char* theData, int theLength, char theFrameType) {
    this->writeReg( (char) MCP2515_TXB0SIDH, (theIdentifier >> 3));
    this->writeReg( (char) MCP2515_TXB0SIDL, (theIdentifier << 5) & 0xe0);// The 5 LSB = 0

    // Write the Data Length (4 LSB) and the RTR (Remote Transmission Request) bit
    if (theFrameType == (char) MCP2515_TX_STD_FRAME) {
        this->writeReg( (char) MCP2515_TXB0DLC, (theLength & 0x0f));// RTR = 0;
    } else if (theFrameType == (char) MCP2515_TX_REMOTE_FRAME) {
        this->writeReg( (char) MCP2515_TXB0DLC, (theLength | 0xf0));// RTR = 1;
    } else {
        cout << "Error : Illegal FrameType in DoSendMsg" << endl;
    }

    // Write the Data
    int i = 0;
    char x = (char) MCP2515_TXB0D0;
    while (i < theLength) {
    	this->writeReg(x, theData[i]);
        x += 1;
        i += 1;
    }

    // Send Message (TXREQ = 1) with highest priority (TXP = 11)
    this->writeReg( (char) MCP2515_TXB0CTRL, 0x0f);
    time_sleep(WAITING_TIME);
}

void MyMCP2515::doReceiveMsg() {
    char TheRxStatus = this->readCommand( (char) MCP2515_CMD_RXSTATUS);

    if ((TheRxStatus & 0xc0) == 0){
        return;
    }

    if ((TheRxStatus & 0x40) != 0) {         //Message in RXB0
        cout << "Message in RXBO";
        int TheRxStatusTemp = TheRxStatus >> 3;
        int TheRxStatusTempTemp = TheRxStatusTemp & 0x07;
        cout << "Mst Type = " << hex << TheRxStatusTempTemp;
        cout << "RXB0CTRL = " << hex << this->readReg( (char) MCP2515_RXB0CTRL);
        cout << "RXB0SIDH = " << hex << this->readReg( (char) MCP2515_RXB0SIDH);
        cout << "RXB0SIDL = " << hex << this->readReg( (char) MCP2515_RXB0SIDL);
        cout << "RXB0DLC  = " << hex << this->readReg( (char) MCP2515_RXB0DLC);
        cout << "RXB0D0   = " << hex << this->readReg( (char) MCP2515_RXB0D0);
        cout << "RXB0D1   = " << hex << this->readReg( (char) MCP2515_RXB0D1);
        cout << "RXB0D2   = " << hex << this->readReg( (char) MCP2515_RXB0D2);
        cout << "RXB0D3   = " << hex << this->readReg( (char) MCP2515_RXB0D3);
        int temp1 = this->readReg( (int) MCP2515_RXB0SIDH);
        temp1 <<= 3;
        int temp2 = this->readReg( (int) MCP2515_RXB0SIDL);
        temp2 >>= 5;
        int temp3 = temp1 | temp2;
        cout << "Identif  = " << hex << temp3;

        if ((TheRxStatus & 0x80) != 0) {       // Message in RXB1
            cout << "Message in RXB1";
	}
            
        return;
    }
}

void MyMCP2515::doHandleIRQ(int thePin) {
    char MyCANINTF = this->readReg( (char) MCP2515_CANINTF);

    if ((MyCANINTF & 0x03) != 0) {
        cout <<  "IRQ from CAN - RX : " << hex << MyCANINTF << endl;
        this->doReceiveMsg();          // Receive the message, just display it !
        this->writeReg( (char) MCP2515_CANINTF, 
                this->readReg( (char) MCP2515_CANINTF) & 0xfc);     // Clear RX1IF & RX0IF
    }  
    
    if ((MyCANINTF & 0x04) != 0) {      // TX0E Transmit Buffer 0 Empty
        cout <<  "IRQ from CAN - TX : " << hex << MyCANINTF << endl;
        this->writeReg( (char) MCP2515_CANINTF, 
                this->readReg( (char) MCP2515_CANINTF) & 0xfb);     // Clear TX0IF
    }
    
    if ((MyCANINTF & 0x80) != 0) {      
        cout << "IRQ from CAN - Message Error" << endl;
        this->writeReg( (char) MCP2515_CANINTF, 
                this->readReg( (char) MCP2515_CANINTF) & 0x7f);     // Clear MERRF
    }  
    
}

  

  