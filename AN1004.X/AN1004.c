
/********************************************************************
 *                                                                    
 *                     Software License Agreement                      
 *                                                                     
 * The software supplied herewith by Microchip Technology Incorporated 
 * (the "Company") for its PICmicro® Microcontroller is intended and   
 * supplied to you, the Company’s customer, for use solely and         
 * exclusively on Microchip PICmicro Microcontroller products.         
 *                                                                     
 * The software is owned by the Company and/or its supplier, and is     
 * protected under applicable copyright laws. All rights are reserved.  
 * Any use in violation of the foregoing restrictions may subject the  
 * user to criminal sanctions under applicable laws, as well as to     
 * civil liability for the breach of the terms and conditions of this  
 * license.                                                             
 *                                                                      
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,   
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED   
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A         
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,   
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR          
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.                    
 *                                                                     
 *******************************************************************
 *
 *   Filename:           AN1004.c
 *   Date:               September 1, 2005
 *   File Version:       1.0
 *   Compiled using:     MPLAB IDE 7.20.00.0
 *                       MPLAB C18 2.40
 *
 *   Author:             Chris Parris
 *   Company:            Microchip Technology Inc.
 *
 *******************************************************************
 *
 *   Files required:     p18f1220.h
 *
 *******************************************************************
 *
 *   Purpose:
 *
 *   This application note is intended to serve as a reference for
 *   manually communicating with Microchip's 93XXXX series serial
 *   EEPROM devices, that is, without relying on a hardware serial
 *   port to handle the Microwire operations.
 *
 *******************************************************************
 *
 *   Program Description:
 *
 *   This program illustrates the following operations:
 *    - EWEN commmand
 *    - WRITE commmand
 *    - READY/BUSY polling
 *    - READ commmand
 *    - EWDS commmand
 *   All 8-bit operations were tested using the 93LC66A device, which
 *   features an 8-bit word organization. For the write operation,
 *   the value 0x55 is written to address 0x010. This same address is
 *   accessed by the read operation.
 *
 *   All 16-bit operations were tested using the 93LC66B device, which
 *   features a 16-bit word organization. For the write operation,
 *   the value 0x55AA is written to address 0x10. This same address is
 *   accessed by the read operation.
 *
 *   All timings assume a 10 MHz crystal oscillator is used. If a
 *   different crystal frequency is desired, care must be taken to
 *   avoid violating device timing specs.
 *
 *******************************************************************
 *
 *   Port A Pin Descriptions:
 *
 *   CS             bit = 3
 *
 *   Port B Pin Descriptions:
 *
 *   SCK            bit = 0
 *   DO (PIC-side)  bit = 1
 *   DI (PIC-side)  bit = 4
 *
 ** I N C L U D E S ************************************************/
#include <xc.h>

/** C O N F I G ****************************************************/
#pragma config OSC = HS         // High-speed oscillator
#pragma config FSCM = OFF       // Fail-Safe clock monitor disabled
#pragma config IESO = OFF       // Int./Ext. Switch-Over disabled
#pragma config PWRT = OFF       // Power-Up Timer disabled
#pragma config BOR = OFF        // Brown Out Detect disabled
#pragma config WDT = OFF        // Watchdog Timer disabled
#pragma config MCLRE = ON       // Master Clear enabled
#pragma config STVR = ON        // Stack Overflow Reset enabled
#pragma config LVP = OFF        // Low-voltage Programming disabled
#pragma config CP0 = OFF, CP1 = OFF, CPB = OFF, CPD = OFF
#pragma config WRT0 = OFF, WRT1 = OFF, WRTB = OFF, WRTC = OFF, WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTRB = OFF

/** D E F I N E S **************************************************/
#define EWEN_CMD    0b10011000          // EWEN command
#define EWDS_CMD    0b10000000          // EWDS command
#define WRITE_CMD   0b10100000          // WRITE command
#define READ_CMD    0b11000000          // READ command
#define CS          PORTAbits.RA3       // Chip select pin, PORTA pin 3
#define SCK         PORTBbits.RB0       // Clock pin, PORTB pin 0
#define DO          PORTBbits.RB1       // Data output pin, PORTB pin 1
#define DI          PORTBbits.RB4       // Data input pin, PORTB pin 4

/* This constant is used to determine how many bits are transferred
 * during each command. The provided value is for a 93XX66A device,
 * and must be altered if a different device is to be used.
 */
#define NUMBITS     12                  // # of bits of EWEN cmd.

/** V A R I A B L E S **********************************************/
unsigned char command;                  // Command byte variable
unsigned int address;                   // Address word variable
unsigned char buffer;                   // I/O buffer variable

/** P R O T O T Y P E S ********************************************/
void init(void);                        // Init. function
void SendCommand(void);                 // Command output function
void bitout(void);                      // Bit output function
void bitin(void);                       // Bit input function
void byteout(void);                     // Byte output function
void bytein(void);                      // Byte input function
void poll(void);                        // Ready/Busy polling function

void Ewen(void);                        // Erase/Write Enable function
void Ewds(void);                        // Erase/Write Disable function
void WriteX8(unsigned char);            // 8-bit Write function
void WriteX16(unsigned int);            // 16-bit Write function
unsigned char ReadX8(void);             // 8-bit Read function
unsigned int ReadX16(void);             // 16-bit Read function

void main(void)
{
    //unsigned char data;                 // 8-bit Data byte
    unsigned int data;                  // 16-bit Data word

    init();                             // Initialize PIC

    Ewen();                             // Send EWEN command

    // Function calls for x8 devices
    data = 0x55;                        // Assign 0x55 to data
    address = 0x10;                     // Assign 0x10 to address
    WriteX8(data);                      // Output data byte
    data = ReadX8();                    // Input data byte

    // Function calls for x16 devices (make sure you update the NUMBITS
    //  constant if you switch to an x16 device)
    //data = 0x55AA;                      // Assign 0x55AA to data
    //address = 0x10;                     // Assign 0x10 to address
    //WriteX16(data);                     // Output data word
    //data = ReadX16();                   // Input data word

    Ewds();                             // Send EWDS command

    while(1){};                         // Loop here forever
} // end main(void)

/********************************************************************
 * Function:        void init(void)
 *
 * Description:     This function initializes the PICmicro
 *                  microcontroller.
 *******************************************************************/
void init(void)
{
    ADCON1 = 0x7F;                      // Configure digital I/O
    PORTA = 0;                          // Clear all PORTA pins
    TRISA = 0b11110111;                 // Configure PORTA I/O
    PORTB = 0;                          // Clear all PORTB pins
    TRISB = 0b11111100;                 // Configure PORTB I/O
} // end init(void)

/********************************************************************
 * Function:        void Ewen(void)
 *
 * Description:     This function sends an EWEN command to the device.
 *                  Once this command has been given, writing to the
 *                  device array will be enabled, and will remain as
 *                  such until an EWDS command is given or power is
 *                  removed from the device.
 *******************************************************************/
void Ewen(void)
{
    command = EWEN_CMD;                 // Load EWEN command value
    address = 0;                        // Clear address word
    CS = 1;                             // Enable Chip Select
    SendCommand();                      // Output command to device
    CS = 0;                             // Disable Chip Select
} // end Ewen(void)

/********************************************************************
 * Function:        void WriteX8(unsigned char data)
 *
 * Description:     This function writes the 8-bit value stored in
 *                  data to the serial EEPROM device, at the location
 *                  specified by address.
 *******************************************************************/
void WriteX8(unsigned char data)
{
    command = WRITE_CMD;                // Load WRITE command value
    CS = 1;                             // Enable Chip Select
    SendCommand();                      // Output command to device
    buffer = data;                      // Copy data to buffer
    byteout();                          // Output byte
    CS = 0;                             // Disable Chip Select

    poll();                             // Begin ready/busy polling
} // end WriteX8(unsigned char data)

/********************************************************************
 * Function:        void WriteX16(unsigned int data)
 *
 * Description:     This function writes the 16-bit value stored in
 *                  data to the serial EEPROM device, at the location
 *                  specified by address.
 *******************************************************************/
void WriteX16(unsigned int data)
{
    command = WRITE_CMD;                // Load WRITE command value
    CS = 1;                             // Enable Chip Select
    SendCommand();                      // Output command to device
    buffer = (char)(data >> 8);         // Copy data MSB to buffer
    byteout();                          // Output byte
    buffer = (char)data;                // Copy data LSB to buffer
    byteout();                          // Output byte
    CS = 0;                             // Disable Chip Select

    poll();                             // Begin ready/busy polling
} // end WriteX16(unsigned int data)

/********************************************************************
 * Function:        unsigned char ReadX8(void)
 *
 * Description:     This function reads an 8-bit value from the
 *                  serial EEPROM device, from the location
 *                  specified by address, returns it.
 *******************************************************************/
unsigned char ReadX8(void)
{
    command = READ_CMD;                 // Load READ command value
    CS = 1;                             // Enable Chip Select
    SendCommand();                      // Output command to device
    bytein();                           // Input byte from device
    CS = 0;                             // Disable Chip Select

    return buffer;                      // Return value
} // end ReadX8(void)

/********************************************************************
 * Function:        unsigned int ReadX16(void)
 *
 * Description:     This function reads a 16-bit value from the
 *                  serial EEPROM device, from the location
 *                  specified by address, returns it.
 *******************************************************************/
unsigned int ReadX16(void)
{
    unsigned int retval;                // Return value variable

    command = READ_CMD;                 // Load READ command value
    CS = 1;                             // Enable Chip Select
    SendCommand();                      // Output command to device
    bytein();                           // Input byte from device
    retval = buffer;                    // Copy byte to retval MSB
    bytein();                           // Input byte from device
    retval = (retval << 8) | buffer;    // Copy byte to retval LSB
    CS = 0;                             // Disable Chip Select

    return retval;                      // Return value
} // end ReadX16(void)

/********************************************************************
 * Function:        void Ewds(void)
 *
 * Description:     This function sends an EWDS command to the device.
 *                  It is good practice to always send this command
 *                  after completing a write, so as to avoid the
 *                  corruption of data stored in the device.
 *******************************************************************/
void Ewds(void)
{
    command = EWDS_CMD;                 // Load EWDS command value
    address = 0;                        // Clear address word
    CS = 1;                             // Enable Chip Select
    SendCommand();                      // Output command to device
    CS = 0;                             // Disable Chip Select
} // end Ewds(void)

/********************************************************************
 * Function:        void poll(void)
 *
 * Description:     This function brings CS high to initiate the
 *                  Ready/Busy polling feature. DO is then continuously
 *                  polled to see when it goes high, thus indicating
 *                  that the write cycle has completed.
 *******************************************************************/
void poll(void)
{
    CS = 1;                             // Set CS high
    Nop();                              // Avoid violating Tsv
    while (DI == 0) {};                 // Wait until DI is high
    CS = 0;                             // Bring CS low
} // end poll(void)

/********************************************************************
 * Function:        void bitout(void)
 *
 * Description:     This function outputs the MSb of buffer to the
 *                  serial EEPROM device.
 *******************************************************************/
void bitout(void)
{
    if (buffer & 0x80)                  // Check if next bit is a 1
    {
        DO = 1;                         // If so, send a 1
    }
    else                                // Otherwise
    {
        DO = 0;                         // Send a 0
    }
    SCK = 1;                            // Bring SCK high to latch data
    Nop();                              // Avoid violating Tckh
    SCK = 0;                            // Bring SCK low for next bit
} // end bitout(void)

/********************************************************************
 * Function:        void bitin(void)
 *
 * Description:     This function inputs a bit from the serial EEPROM
 *                  device and stores it in the LSb of buffer.
 *******************************************************************/
void bitin(void)
{
    buffer &= 0xFE;                     // Assume next bit will be 0
    SCK = 1;                            // Bring SCK to latch data
    Nop();                              // Avoid violating Tckh
    SCK = 0;                            // Bring SCK low
    if (DI == 1)                        // Check if DI is high
    {
        buffer |= 0x01;                 // If high, set next bit
    }
} // end bitin(void)

/********************************************************************
 * Function:        void byteout(void)
 *
 * Description:     This function outputs the byte specified in
 *                  buffer to the serial EEPROM device.
 *******************************************************************/
void byteout(void)
{
    unsigned char i;                    // Loop counter

    for (i = 0; i < 8; i++)             // Loop through each bit
    {
        bitout();                       // Output bit
        buffer = buffer << 1;           // Rotate left for next bit
    }
} // end byteout(void)

/********************************************************************
 * Function:        void bytein(void)
 *
 * Description:     This function inputs a byte from the serial
 *                  EEPROM device and stores it in buffer.
 *******************************************************************/
void bytein(void)
{
    unsigned char i;                    // Loop counter

    buffer = 0;
    for (i = 0; i < 8; i++)             // Loop through each bit
    {
        buffer = buffer << 1;           // Rotate left for next bit
        bitin();                        // Input bit
    }
} // end bytein(void)

/********************************************************************
 * Function:        void SendCommand(void)
 * 
 * Description:     This function sends the Start bit and opcode
 *                  specified in the MSb's of command, as well as
 *                  the required number of address or dummy bits,
 *                  to the serial EEPROM device.
 *******************************************************************/
void SendCommand(void)
{
    static unsigned char i;             // Loop counter
    static unsigned int cmd_addr;       // Variable for command & address
    static unsigned int temp;           // Temp. variable

    cmd_addr = address;                 // Copy address to cmd_addr;
    // First, align address bits to be combined with command
    for (i = 0; i < (16-NUMBITS); i++)  // Skip through unused addr. bits
    {
        cmd_addr = cmd_addr << 1;       // Rotate left to skip bit
    }

    // Next, combine command into address word
    cmd_addr &= 0x1FFF;                 // Mask off upper 3 bits
    temp = command;                     // Copy command value to temp
    cmd_addr |= (temp<<8);              // Combine address & command

    // Finally, output entire command to device
    for (i = 0; i < NUMBITS; i++)       // Loop through each bit
    {
        buffer = (char)(cmd_addr >> 8); // Copy address MSB to buffer
        bitout();                       // Output next bit
        cmd_addr = cmd_addr << 1;       // Rotate left for next bit
    }
} // end SendCommand(void)
