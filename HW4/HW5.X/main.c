#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"
#include <math.h>

// DEVCFG0
#pragma config DEBUG = 0b11 // no debugging
#pragma config JTAGEN = 0 // no jtag
#pragma config ICESEL = 0b11 // use PGED1 and PGEC1
#pragma config PWP = 0b111111111 // no write protect
#pragma config BWP = 0 // no boot write protect
#pragma config CP = 1 // no code protect

// DEVCFG1
#pragma config FNOSC = 0b011 // use primary oscillator with pll
#pragma config FSOSCEN = 0 // turn off secondary oscillator
#pragma config IESO = 0 // no switching clocks
#pragma config POSCMOD = 0b10 // high speed crystal mode
#pragma config OSCIOFNC = 1 // disable secondary osc
#pragma config FPBDIV = 0b00 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = 0b11 // do not enable clock switch
#pragma config WDTPS = 0b10100 // use slowest wdt
#pragma config WINDIS = 1 // wdt no window mode
#pragma config FWDTEN = 0 // wdt disabled
#pragma config FWDTWINSZ = 0b11 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = 0b001 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = 0b111 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = 0b001 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = 0b001 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = 0 // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = 0 // allow multiple reconfigurations
#pragma config IOL1WAY = 0 // allow multiple reconfigurations
#pragma config FUSBIDIO = 1 // USB pins controlled by USB module
#pragma config FVBUSONIO = 1 // USB BUSON controlled by USB module

#define EXPANDER_ADDR 0b0100000  //connect A2 A1 A0 all to ground to get 0

void initExpander() {
    //turn off the analog
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
    
    //initialize pin GP0-3 as output and GP 4-7 as input
    i2c_master_start();
    i2c_master_send(EXPANDER_ADDR<<1|0);//send address, write mode
    i2c_master_send(0x00);              //write to IODIR
    i2c_master_send(0b11110000);        //set GP0-3 output, GP 4-7 input
    i2c_master_stop();
    
    //set GP0-3 low
    i2c_master_start();
    i2c_master_send(EXPANDER_ADDR<<1|0);//send address, write mode
    i2c_master_send(0x09);              //write to GPIO
    i2c_master_send(0b00000000);
    i2c_master_stop();
    
    //turn on the pull up resistor for PG4-7
//    i2c_master_start();
//    i2c_master_send(EXPANDER_ADDR<<1|0);//send address, write mode
//    i2c_master_send(0x06);              //write to GPPU
//    i2c_master_send(0b10000000);        
//    i2c_master_stop();
}

unsigned char getExpander() {
    i2c_master_start();
    i2c_master_send(EXPANDER_ADDR<<1|0);//send address, write mode
    i2c_master_send(0x09);              //write to GPIO
    i2c_master_restart();
    i2c_master_send(EXPANDER_ADDR<<1|1);//send address, read mode
    unsigned char r = i2c_master_recv();//save the value
    i2c_master_ack(1);                  //tell slave that master get the value
    i2c_master_stop();
    
    return r;
}

void setExpander(char pin, char level) {
    unsigned char value_now = getExpander();
    value_now ^= (-level ^ value_now) & (1 << pin); //http://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit-in-c-c
    i2c_master_start();
    i2c_master_send(EXPANDER_ADDR<<1|0);//send address, write mode
    i2c_master_send(0x09);              //write to GPIO
    i2c_master_send(value_now);        //set pin to level
    i2c_master_stop();
}

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1;
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;

    //initialize Expander
    initExpander();
    
    __builtin_enable_interrupts();

    while(1) {
	    // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
		  // remember the core timer runs at half the sysclk
//        _CP0_SET_COUNT(0);
//        int waitTime = 24000;   //1kHz updating rate
//        
//        while(_CP0_GET_COUNT() < waitTime) {
//            ;
//        }
        
        //read value from GP7
        unsigned char in_value = getExpander();
        unsigned char GP7_value = (in_value>>7)&1;
        if (GP7_value == 1) {
            setExpander(0,1);
        }
        else {
            setExpander(0,0);
        }
        
    }
}