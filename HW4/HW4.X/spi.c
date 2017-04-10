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
    SPI1CONbits.ON = 1;
    
}

char SPI1_IO(char write) {
    SPI1BUF = write;
    while(!SPI1STATbits.SPIRBF) {
        ;
    }
    return SPI1BUF;
}