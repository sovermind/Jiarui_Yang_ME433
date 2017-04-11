#include "spi.h"                   
#include <xc.h>

void initSPI1() {
    //make pin 17, which is RPB8, as SDO1
    RPB8Rbits.RPB8R = 0b0011;
    //set up chip select pin as output
    TRISBbits.TRISB15 = 0;  
    CS = 1;
    //make pin 3, which is RPA1, as SDI1
    SDI1Rbits.SDI1R = 0b0000;
    //sck is B14
    
    //set up SPI1
    SPI1CON = 0;     //turn off the spi module and reset it
    SPI1BUF;         //clear rx buffer by reading from it
    SPI1BRG = 0x1;      //SPI4BRG = (48000000/(2*desired))-1, fastest rate = 12MHz
    SPI1STATbits.SPIROV = 0;    //clear overflow bit
    SPI1CONbits.CKE = 1;        //data changes when clock goes from hi to low
    SPI1CONbits.CKP = 0;
    SPI1CONbits.MSTEN = 1;      //master operation
    //SPI1CONbits.MODE16 = 1;
    SPI1CONbits.ON = 1;
    
}

char SPI1_IO(char write) {
    SPI1BUF = write;
    while(!SPI1STATbits.SPIRBF) {
        ;
    }
    return SPI1BUF;
}

void DAQ_write(char channel, unsigned char value) {
    char a = value >> 4;         //This automatically fill the first 4 bits as 0000, channel A
    a ^=1<<6;
    a ^=1<<5;
    a ^=1<<4;
    if (channel == 1)
    {
        a ^= 1<<7;               //invert the first bit, which makes it 0111. channel B
    }
    char b = value << 4;         //This makes the last four bits 0000
    
    CS = 0;
    SPI1_IO(a);
    SPI1_IO(b);
    CS = 1;
}