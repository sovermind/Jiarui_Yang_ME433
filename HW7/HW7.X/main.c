#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"
#include "ILI9163C.h"
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

#define IMU_ADDR 0b1101011  //default address for IMU board 0b1101011

void writetoIMU(unsigned char register_addr, unsigned char send_value);
//all functions to display on screen, copied from HW 6
//draw the char c on x,y location with color
void display_character(unsigned char c, unsigned short x, unsigned short y, unsigned short char_color, unsigned short back_color) {
    char ascii_row = c - 0x20;
    int i=0;
    int j=0;
    
    for (i=0;i<5;i++) {
        if ((x+i) < 128) {
            for (j=0;j<8;j++) {
                if ((ASCII[ascii_row][i]>>j)&1 == 1) {
                    if ((y+j)<128) {
                        LCD_drawPixel(x+i,y+j,char_color);    //write the charactor color
                    }
                }
                else {
                    if ((y+j)<128) {
                        LCD_drawPixel(x+i,y+j,back_color);    //clear the background color
                    }
                }
            }
        }
    }
}

void display_String(unsigned char* c,unsigned short x, unsigned short y, unsigned short char_color, unsigned short back_color){
    char cc = *c;
    unsigned short count =0;
    while (cc != 0) {
        display_character(cc,x+count*5,y,char_color,back_color);
        count ++;
        cc = *(c+count);
    }
}

//draw the progress bar at x,y position with bar_color and can define the thickness
//It will clear all other pixels within (x+thickness) locations
//thickness and width are in pixels
void draw_bar(unsigned short thickness,unsigned short width, unsigned short x, unsigned short y, unsigned short bar_color, unsigned short back_color) {
    int w = 0;
    int t = 0;
    //check if the bar will exceed the boundry
    if (x+width > 128) {
        width = 127 - x;
    }
    if (y+thickness > 128) {
        thickness = 127 - y;
    }
    
    for (w = 0;w<128;w++) {
        for (t = y;t<y+thickness;t++) {
            if (w<x || w>(x+width)) {
                LCD_drawPixel(w,t,back_color);
            }
            else {
                LCD_drawPixel(w,t,bar_color);
            }
        }
    }
}

void initIMU() {
    //turn off the analog
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    //i2c init
    i2c_master_setup();
    //set up accl and gyro
    writetoIMU(0x10,0b10000010);  //CTRL1_XL accelerator
    writetoIMU(0x11,0b10001000);  //CTRL2_G gyro
    writetoIMU(0x12,0b00000100);  //CTRL3_C control
    
}

unsigned char readIMU(unsigned char register_addr) {
    i2c_master_start();
    i2c_master_send(IMU_ADDR<<1|0);//send address, write mode
    i2c_master_send(register_addr);              //write to register
    i2c_master_restart();
    i2c_master_send(IMU_ADDR<<1|1);//send address, read mode
    unsigned char r = i2c_master_recv();//save the value
    i2c_master_ack(1);                  //tell slave that master get the value
    i2c_master_stop();
    
    return r;

}

void IMU_read_multiple(unsigned char address, unsigned char register_addr, unsigned char * data, int length) {
    
}

void writetoIMU(unsigned char register_addr, unsigned char send_value) {
    i2c_master_start();
    i2c_master_send(IMU_ADDR<<1|0);//send address, write mode
    i2c_master_send(register_addr);              //write to register
    i2c_master_send(send_value);        //sent in value
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

    //initialize IMU
    initIMU();
    //init SPI1 and LCD
    SPI1_init();
    LCD_init();
    
    __builtin_enable_interrupts();
    //clear the screen
    LCD_clearScreen(WHITE);
//    display_String("Hello, world",28,32,BLACK,WHITE);
    
    //read from who am I
    unsigned char who_am_i_addr = 0x0F;
    unsigned char who_am_i_value = readIMU(who_am_i_addr);
    if (who_am_i_value == 0b01101001){
        display_String("Connected!",15,15,BLACK,WHITE);
    }

    

    while(1) {
	    // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
		  // remember the core timer runs at half the sysclk
//        _CP0_SET_COUNT(0);
//        int waitTime = 24000;   //1kHz updating rate
//        
//        while(_CP0_GET_COUNT() < waitTime) {
//            ;
//        }
        
    }
}