#ifndef SPI__H__
#define SPI__H__

#define CS LATBbits.LATB15

void initSPI1();
char SPI1_IO(char write);

#endif
