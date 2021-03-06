/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "i2c_master_noint.h"
#include "ILI9163C.h"
#include "math.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define IMU_ADDR 0b1101011  //default address for IMU board 0b1101011
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

float temp = 0; 
float g_x = 0;       //35mdps/LSB from data sheet page 15
float g_y = 0;
float g_z = 0;
float a_x = 0;      //0.061mg/LSB from data sheet page 15
float a_y = 0;
float a_z = 0;

float roll = 0;
float pitch = 0;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
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
void draw_h_bar(unsigned short thickness,unsigned short width, unsigned short x, unsigned short y, unsigned short bar_color, unsigned short back_color) {
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

//draw the verticle bar, will clear a region of bar width and specified y region
void draw_v_bar(unsigned short y_up_lim,unsigned short y_low_lim, unsigned short thickness,unsigned short width, unsigned short x, unsigned short y, unsigned short bar_color, unsigned short back_color) {
    int w = 0;
    int t = 0;
    
    for (w = x;w<x+width;w++) {
        for (t = y_up_lim;t<y_low_lim;t++) {
            if (t>=y && t<=y+thickness) {
                LCD_drawPixel(w,t,bar_color);
            }
            else {
                LCD_drawPixel(w,t,back_color);
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

void IMU_read_multiple(unsigned char register_addr, unsigned char * data, int length) {
    i2c_master_start();
    i2c_master_send(IMU_ADDR<<1|0);//send address, write mode
    i2c_master_send(register_addr);              //write to register
    i2c_master_restart();
    i2c_master_send(IMU_ADDR<<1|1);//send address, read mode
    int count = 0;
    while (1) {
        data[count] = i2c_master_recv();//save the value
        if (count == (length-1)) {
            i2c_master_ack(1);                  //tell slave no more byte should be send
            break;
        } 
        else {
            i2c_master_ack(0);                  //tell slave to send another byte
        }
        count ++;
    }
    i2c_master_stop();
}

void writetoIMU(unsigned char register_addr, unsigned char send_value) {
    i2c_master_start();
    i2c_master_send(IMU_ADDR<<1|0);//send address, write mode
    i2c_master_send(register_addr);              //write to register
    i2c_master_send(send_value);        //sent in value
    i2c_master_stop();
}

void read_all_value() {
    int data_length = 14;   //which means all data
    unsigned char alldata[data_length];
    IMU_read_multiple(0x20,alldata,data_length);
    //seperate all data
    unsigned char L_bits[data_length/2];
    unsigned char H_bits[data_length/2];
    int i=0;
    for (i = 0;i<data_length/2;i++) {
        L_bits[i] = alldata[i*2];
        H_bits[i] = alldata[i*2+1];
    }
    signed short temp_raw = (H_bits[0])<<8|(L_bits[0]);
    signed short g_x_raw = (H_bits[1])<<8|(L_bits[1]);
    signed short g_y_raw = (H_bits[2])<<8|(L_bits[2]);
    signed short g_z_raw = (H_bits[3])<<8|(L_bits[3]);
    signed short a_x_raw = (H_bits[4])<<8|(L_bits[4]);
    signed short a_y_raw = (H_bits[5])<<8|(L_bits[5]);
    signed short a_z_raw = (H_bits[6])<<8|(L_bits[6]);
        
    float temp = temp_raw*0.0625; 
    g_x = g_x_raw*35.0/1000.0;       //35mdps/LSB from data sheet page 15
    g_y = g_y_raw*35.0/1000.0;
    g_z = g_z_raw*35.0/1000.0;
    a_x = a_x_raw*0.061/1000.0;      //0.061mg/LSB from data sheet page 15
    a_y = a_y_raw*0.061/1000.0;
    a_z = a_z_raw*0.061/1000.0;
    a_z = -a_z;
}
/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */

    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1;
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;
    //initialize IMU
    initIMU();
    //init SPI1 and LCD
    SPI1_init();
    LCD_init();
    
    //clear the screen
    LCD_clearScreen(WHITE);
//    display_String("Hello, world",28,32,BLACK,WHITE);
    
    //read from who am I
    unsigned char who_am_i_addr = 0x0F;
    unsigned char who_am_i_value = readIMU(who_am_i_addr);
    if (who_am_i_value == 0b01101001){
        display_String("Connected!",15,10,BLACK,WHITE);
    }

//    //test to see if i can read from accel
//    unsigned char test1 = readIMU(0x28);
//    unsigned char test2 = readIMU(0X29);
//    unsigned char str[100];
//    sprintf(str,"L = %x, H = %x",test1,test2);
//    display_String(str,15,25,BLACK,WHITE);
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
            // remember the core timer runs at half the sysclk
            _CP0_SET_COUNT(0);
            int waitTime = 4800000;   //1kHz updating rate

            read_all_value();
            char str[100];
    //        sprintf(str,"a_x = %.3f   ",a_x);
    //        display_String(str,15,35,BLACK,WHITE);
    //        sprintf(str,"a_y = %.3f   ",a_y);
    //        display_String(str,15,45,BLACK,WHITE);
    //        sprintf(str,"a_z = %.3f   ",a_z);
    //        display_String(str,15,55,BLACK,WHITE);
            //compute pitch and roll use atan2()
            //pitch => around x axis; roll =>around y axis
            //both pitch and roll are opposite as what indicated on the IMU board
            //x, y directions are also opposite
            pitch = (atan2(a_y,a_z)*180/M_PI);
            sprintf(str,"pitch = %.3f   ",pitch);
            display_String(str,15,25,BLACK,WHITE);

            roll = (atan2(-a_x,a_z)*180/M_PI);
            sprintf(str,"roll = %.3f   ",roll);
            display_String(str,15,35,BLACK,WHITE);

            unsigned short y_words_end = 44;

            //draw the bar feature here
            //draw x direction bar
            unsigned short x_bar_x = 0;
            unsigned short x_bar_y = 75;
            unsigned short x_bar_t = 5;
            unsigned short x_bar_w = 0;
            int x_length = roll/90.0*40.0;
            if (x_length <=0){
                x_bar_x = 64;
                x_bar_w = -x_length;
            }
            else {
                x_bar_w = x_length;
                x_bar_x = 64-x_length;
            }
            draw_h_bar(x_bar_t,x_bar_w,x_bar_x,x_bar_y,RED,WHITE);

            //draw y direction bar
            unsigned short y_bar_x = 64;
            unsigned short y_bar_y = 0;
            unsigned short y_bar_t = 0;
            unsigned short y_bar_w = 5;
            int y_length = pitch/90.0*40.0;
            if (y_length <=0){
                y_bar_y = 75 + y_length;
                y_bar_t = -y_length;
                draw_v_bar(y_words_end,75,y_bar_t,y_bar_w,y_bar_x,y_bar_y,GREEN,WHITE);
                draw_v_bar(80,90,5,y_bar_w,y_bar_x,81,WHITE,WHITE);
            }
            else {
                y_bar_y = 80;
                y_bar_t = y_length;
                draw_v_bar(80,127,y_bar_t,y_bar_w,y_bar_x,y_bar_y,GREEN,WHITE);
                draw_v_bar(65,75,5,y_bar_w,y_bar_x,66,WHITE,WHITE);
            }


            while(_CP0_GET_COUNT() < waitTime) {
                ;
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
