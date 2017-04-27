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
#include <stdio.h>
#include <xc.h>
#include "ILI9163C.h"
#include "i2c_master_noint.h"
#include "math.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define IMU_ADDR 0b1101011  //default address for IMU board 0b1101011

uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0;
int startTime = 0;

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
bool print_IMU = false;
int IMU_count = 0;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
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
        
    temp = temp_raw*0.0625; 
    g_x = g_x_raw*35.0/1000.0;       //35mdps/LSB from data sheet page 15
    g_y = g_y_raw*35.0/1000.0;
    g_z = g_z_raw*35.0/1000.0;
    a_x = a_x_raw*0.061/1000.0;      //0.061mg/LSB from data sheet page 15
    a_y = a_y_raw*0.061/1000.0;
    a_z = a_z_raw*0.061/1000.0;
    a_z = -a_z;
}

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
        USB_DEVICE_CDC_INDEX index,
        USB_DEVICE_CDC_EVENT event,
        void * pData,
        uintptr_t userData
        ) {
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *) userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch (event) {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *) pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch (event) {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData;
            if (configuredEventData->configurationValue == 1) {
                /* Update LED to show configured state */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t) & appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void) {
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if (appData.isConfigured == false) {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    } else {
        retVal = false;
    }

    return (retVal);
}

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

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    //LCD and IMU initialization
    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1;
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 0;
    //initialize IMU
    initIMU();
    //init SPI1 and LCD
    SPI1_init();
    LCD_init();

    //clear the screen
    LCD_clearScreen(WHITE);
    IMU_count = 0;
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

    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    //appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];

    startTime = _CP0_GET_COUNT();
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    /* Update the application state machine based
     * on the current state */

    switch (appData.state) {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if (appData.isConfigured) {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if (APP_StateReset()) {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if (appData.isReadComplete == true) {
                appData.isReadComplete = false;
                appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);

                if (appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
                
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
            if (appData.isReadComplete && (appData.readBuffer[0] == 'r')) {
                print_IMU = true;
                IMU_count = 0;
            }
            else if (appData.isReadComplete && (appData.readBuffer[0] != 'r')) {
                appData.state = APP_STATE_SCHEDULE_READ;
                print_IMU = false;
            }
            if (print_IMU) {
                appData.state = APP_STATE_CHECK_TIMER;
            }
            break;
                
        case APP_STATE_CHECK_TIMER:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

            //get the IMU display for 100 Hz
            if (appData.isReadComplete || (_CP0_GET_COUNT() - startTime > (48000000 / 2 / 100) && IMU_count < 101)) {
                appData.state = APP_STATE_SCHEDULE_WRITE;
                IMU_count ++;
            }
            if (IMU_count == 101) {
                IMU_count == 0;
                print_IMU = false;
            }
            
//            if (appData.isReadComplete && (appData.readBuffer[0] == 'r')) {
//                appData.state = APP_STATE_SCHEDULE_WRITE;
//                print_IMU = true;
//            }
//            else if (appData.isReadComplete && (appData.readBuffer[0] != 'r')) {
//                appData.state = APP_STATE_SCHEDULE_READ;
//            }

            break;


        case APP_STATE_SCHEDULE_WRITE:

            if (APP_StateReset()) {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

//            len = sprintf(dataOut, "%d\r\n", IMU_count);
//            i++;
            
            //get the IMU data and print it out
            read_all_value();
            len = sprintf(dataOut,"%d ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f\r\n",IMU_count,a_x,a_y,a_z,g_x,g_y,g_z);
            if (appData.isReadComplete) {
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle,
                        appData.readBuffer, 1,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
            } else {
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle, dataOut, len,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                startTime = _CP0_GET_COUNT();
            }
            break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if (appData.isWriteComplete == true) {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}



/*******************************************************************************
 End of File
 */