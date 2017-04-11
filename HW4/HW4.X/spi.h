#ifndef SPI__H__
#define SPI__H__

#include <xc.h> 

#define CS LATBbits.LATB15

void initSPI1();
char SPI1_IO(char write);
void DAQ_write(char channel, char value);

#endif
