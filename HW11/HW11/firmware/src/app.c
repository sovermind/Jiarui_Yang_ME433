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
#include "i2c_master_noint.h"
#include "math.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

#define IMU_ADDR 0b1101011  //default address for IMU board 0b1101011
#define BUFFER_CAP 100

int len, i = 0;
int startTime = 0;
int collect_size = 400;
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

/* Mouse Report */
MOUSE_REPORT mouseReport APP_MAKE_BUFFER_DMA_READY;
MOUSE_REPORT mouseReportPrevious APP_MAKE_BUFFER_DMA_READY;

float temp = 0; 
float g_x = 0;       //35mdps/LSB from data sheet page 15
float g_y = 0;
float g_z = 0;
float a_x = 0;      //0.061mg/LSB from data sheet page 15
float a_y = 0;
float a_z = 0;

float roll = 0;
float pitch = 0;
int IMU_count = 0;


float MAF_buffer[BUFFER_CAP];
float IIR_buffer[BUFFER_CAP];
float FIR_buffer[BUFFER_CAP];

int MAF_buffer_count = 0;
int IIR_buffer_count = 0;
int FIR_buffer_count = 0;

float MAF_data[3];

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void writetoIMU(unsigned char register_addr, unsigned char send_value);

void buffer_clear() {
    int i = 0;
    int j = 0;
    for (i=0;i<BUFFER_CAP;i++){
        MAF_buffer[i] = 0;
        IIR_buffer[i] = 0;
        FIR_buffer[i] = 0;
    }
    MAF_buffer_count = 0;
    IIR_buffer_count = 0;
    FIR_buffer_count = 0;
    
    for (j = 0;j<3;j++) {
        MAF_data[j] = 0;
    }
    
}

//function to perform MAF
void MAF(int ave_num) {
    MAF_buffer[MAF_buffer_count] = a_x;
    MAF_buffer[MAF_buffer_count+ave_num] = a_y;
    MAF_buffer[MAF_buffer_count+ave_num*2] = a_z;
    MAF_buffer_count ++;
    if (MAF_buffer_count == ave_num) {
        MAF_buffer_count = 0;
    }
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;
    
    int i = 0;
    for (i=0;i<ave_num;i++)
    {
        sum_x = sum_x + MAF_buffer[i];
        sum_y = sum_y + MAF_buffer[i+ave_num];
        sum_z = sum_z + MAF_buffer[i+ave_num*2];
    }
    
    MAF_data[0] = sum_x/ave_num;
    MAF_data[1] = sum_y/ave_num;
    MAF_data[2] = sum_z/ave_num;
}

float IIR(float a, float b) {
    IIR_buffer[1] = a_z;
    float ave = a*IIR_buffer[0]+b*IIR_buffer[1];
    IIR_buffer[0] = IIR_buffer[1];
    return ave;
}

float FIR(int sample_numb, float *gains) {
    //make sure that new value always at the end of the array
    if (FIR_buffer_count == sample_numb) {
        FIR_buffer_count = sample_numb - 1;
    }
    //shift array value left
    float sum = 0;
    int i = 0;
    for (i = 0;i<sample_numb-1;i++) {
        FIR_buffer[i] = FIR_buffer[i+1];
        sum = sum + gains[i]*FIR_buffer[i];
    }
    
    sum = sum + gains[sample_numb-1]*a_z;
    //put new value to the end of array
    FIR_buffer[FIR_buffer_count] = a_z;
    FIR_buffer_count ++;
    
    return sum;

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

void APP_USBDeviceHIDEventHandler(USB_DEVICE_HID_INDEX hidInstance,
        USB_DEVICE_HID_EVENT event, void * eventData, uintptr_t userData) {
    APP_DATA * appData = (APP_DATA *) userData;

    switch (event) {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* This means the mouse report was sent.
             We are free to send another report */

            appData->isMouseReportSendBusy = false;
            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* Dont care for other event in this demo */
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* save Idle rate received from Host */
            appData->idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*) eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->idleRate), 1);

            /* On successfully receiving Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function driver returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;

        case USB_DEVICE_HID_EVENT_SET_PROTOCOL:
            /* Host is trying set protocol. Now receive the protocol and save */
            appData->activeProtocol = *(USB_HID_PROTOCOL_CODE *) eventData;

            /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_HID_EVENT_GET_PROTOCOL:

            /* Host is requesting for Current Protocol. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->activeProtocol), 1);

            /* On successfully receiving Idle rate, the Host would acknowledge
              back with a Zero Length packet. The HID function driver returns
              an event USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the
              application upon receiving this Zero Length packet from Host.
              USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates
              this control transfer event is complete */
            break;

        case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;

        default:
            break;
    }
}

/*******************************************************************************
  Function:
    void APP_USBDeviceEventHandler (USB_DEVICE_EVENT event,
        USB_DEVICE_EVENT_DATA * eventData)

  Summary:
    Event callback generated by USB device layer.

  Description:
    This event handler will handle all device layer events.

  Parameters:
    None.

  Returns:
    None.
 */

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED * configurationValue;
    switch (event) {
        case USB_DEVICE_EVENT_SOF:
            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            appData.setIdleTimer++;
            break;
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Device got deconfigured */

            appData.isConfigured = false;
            appData.isMouseReportSendBusy = false;
            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            //appData.emulateMouse = true;
            //BSP_LEDOn ( APP_USB_LED_1 );
            //BSP_LEDOn ( APP_USB_LED_2 );
            //BSP_LEDOff ( APP_USB_LED_3 );

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Device is configured */
            configurationValue = (USB_DEVICE_EVENT_DATA_CONFIGURED *) eventData;
            if (configurationValue->configurationValue == 1) {
                appData.isConfigured = true;

                //BSP_LEDOff ( APP_USB_LED_1 );
                //BSP_LEDOff ( APP_USB_LED_2 );
                //BSP_LEDOn ( APP_USB_LED_3 );

                /* Register the Application HID Event Handler. */

                USB_DEVICE_HID_EventHandlerSet(appData.hidInstance,
                        APP_USBDeviceHIDEventHandler, (uintptr_t) & appData);
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
            //BSP_LEDOff ( APP_USB_LED_1 );
            //BSP_LEDOn ( APP_USB_LED_2 );
            //BSP_LEDOn ( APP_USB_LED_3 );
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;

    }
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
    
    //LCD and IMU initialization
    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1;
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 0;
    //initialize IMU
    initIMU();
    
    IMU_count = 0;
    collect_size = 400;
    
    //init buffer arrays to all zeros
    buffer_clear();

    //read from who am I
    unsigned char who_am_i_addr = 0x0F;
    unsigned char who_am_i_value = readIMU(who_am_i_addr);
    if (who_am_i_value == 0b01101001){
        LATAbits.LATA4 = 1;
    }
    
    appData.state = APP_STATE_INIT;
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;
    appData.isConfigured = false;
    //appData.emulateMouse = true;
    appData.hidInstance = 0;
    appData.isMouseReportSendBusy = false;
    
    //get the start time
    startTime = _CP0_GET_COUNT();
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
//    static int8_t vector = 0;
//    static uint8_t movement_length = 0;
//    int8_t dir_table[] = {-4, -4, -4, 0, 4, 4, 4, 0};
    static uint8_t inc = 0;
    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle,
                        APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }
            break;
        }

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured. The 
             * isConfigured flag is updated in the
             * Device Event Handler */

            if (appData.isConfigured) {
                appData.state = APP_STATE_MOUSE_EMULATE;
            }
            break;

        case APP_STATE_MOUSE_EMULATE:
            
            // every 50th loop, or 20 times per second
//            if (movement_length > 50) {
//                appData.mouseButton[0] = MOUSE_BUTTON_STATE_RELEASED;
//                appData.mouseButton[1] = MOUSE_BUTTON_STATE_RELEASED;
//                appData.xCoordinate = (int8_t) dir_table[vector & 0x07];
//                appData.yCoordinate = (int8_t) dir_table[(vector + 2) & 0x07];
//                vector++;
//                movement_length = 0;
//            } 
            //get the IMU data
            read_all_value();
            MAF(4);
            
//            float IIR_data = IIR(0.5,0.5);
//            //Set FIR gains
//            int s_n = 7;
////            float gg[] = {0.0457,0.4543,0.4543,0.0457};    //four gains
//            float gg[] = {0.0264,0.1405,0.3331,0.3331,0.1405,0.0264};       //six gains
////            float gg[] = {0.0212,0.0897,0.2343,0.3094,0.2343,0.0897,0.0212};
//
//            float FIR_data = FIR(s_n,gg);
            
//            appData.xCoordinate = (int8_t) 2;
//            appData.yCoordinate = (int8_t) 1;
            
            if (inc > 20) {
                int8_t speed_x = (int)(MAF_data[0]/0.6*5.0);
                int8_t speed_y = (int)(MAF_data[1]/0.6*5.0);
//                int8_t speed_x = 1;
//                int8_t speed_y = 1;
                if (MAF_data[0] > 0) {
                    appData.xCoordinate = (int8_t) speed_x;
                }
                else {
                    appData.xCoordinate = (int8_t) -speed_x;
                }
                if (MAF_data[1] > 0) {
                    appData.yCoordinate = (int8_t) speed_y;
                }
                else {
                    appData.yCoordinate = (int8_t) -speed_y;
                }
                inc = 0;
            }
            else {
                appData.xCoordinate = (int8_t) 0;
                appData.yCoordinate = (int8_t) 0;
            }
            
            

            if (!appData.isMouseReportSendBusy) {
                /* This means we can send the mouse report. The
                   isMouseReportBusy flag is updated in the HID Event Handler. */

                appData.isMouseReportSendBusy = true;

                /* Create the mouse report */

                MOUSE_ReportCreate(appData.xCoordinate, appData.yCoordinate,
                        appData.mouseButton, &mouseReport);

                if (memcmp((const void *) &mouseReportPrevious, (const void *) &mouseReport,
                        (size_t)sizeof (mouseReport)) == 0) {
                    /* Reports are same as previous report. However mouse reports
                     * can be same as previous report as the coordinate positions are relative.
                     * In that case it needs to be sent */
                    if ((appData.xCoordinate == 0) && (appData.yCoordinate == 0)) {
                        /* If the coordinate positions are 0, that means there
                         * is no relative change */
                        if (appData.idleRate == 0) {
                            appData.isMouseReportSendBusy = false;
                        } else {
                            /* Check the idle rate here. If idle rate time elapsed
                             * then the data will be sent. Idle rate resolution is
                             * 4 msec as per HID specification; possible range is
                             * between 4msec >= idlerate <= 1020 msec.
                             */
                            if (appData.setIdleTimer
                                    >= appData.idleRate * 4) {
                                /* Send REPORT as idle time has elapsed */
                                appData.isMouseReportSendBusy = true;
                            } else {
                                /* Do not send REPORT as idle time has not elapsed */
                                appData.isMouseReportSendBusy = false;
                            }
                        }
                    }

                }
                if (appData.isMouseReportSendBusy == true) {
                    /* Copy the report sent to previous */
                    memcpy((void *) &mouseReportPrevious, (const void *) &mouseReport,
                            (size_t)sizeof (mouseReport));

                    /* Send the mouse report. */
                    USB_DEVICE_HID_ReportSend(appData.hidInstance,
                            &appData.reportTransferHandle, (uint8_t*) & mouseReport,
                            sizeof (MOUSE_REPORT));
                    appData.setIdleTimer = 0;
                }
//                movement_length++;
                inc++;
            }

            break;

        case APP_STATE_ERROR:

            break;

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

