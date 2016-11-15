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

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

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

    /* Initialize the task1queue */
    appData.task1queue = xQueueCreate(1, sizeof(unsigned char));
    appData.blockQueue = xQueueCreate(1, sizeof(unsigned char));

    
    /* Initialize USART communication */
    appData.app1USARTHandle = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READWRITE);
    DRV_USART_BufferEventHandlerSet( appData.app1USARTHandle, APP_USARTBufferEventHandler,
                                     (uintptr_t)&appData );
    
    /* Initialize "value" output pins */
    //Pins 37 - 30
    PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_E, 0x00FF);    
    
    /* Location Pins */
    //Pin 47
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_6);
    //Pin 46
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_1);
    //Pin 45
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_0);
    //Pin 44
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_10);
    //Pin 43
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8);
    //Pin 42
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_12);
    //Pin 41
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_13);
    //Pin 40
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_11); 
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
    int resend=0;
    SIGNAL_MSG signal_msg;
    ADC_MSG adc_msg;
    LF_MSG lf_msg;
    unsigned char msg_buffer[17];
    size_t length;

    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            // Clear output pins
            dbgOutputLoc(0x00);
            dbgOutputVal(0x00);
            
            if (appData.app1USARTHandle != DRV_HANDLE_INVALID)
            {
                appData.state = APP_STATE_SEND_USART_READY;
            }
            else
            {
                appData.state = APP_STATE_ERR;
            }
            
            break;
        }

        case APP_STATE_SEND_USART_READY:
        {
            dbgOutputLoc(0x01);
            
            
            //UNCOMMENT FOR SIGNAL

            //signal_msg = createSignalMsg(MSG_SUBJECT_SIGNAL,MSG_ADDR_R1_NAV_TH,
            //        MSG_ADDR_R2_NAV_TH,MSG_SIG_GO_STRAIGHT); //SUBJ, FROM, TO, SIGNAL
            //packSignalMsg(msg_buffer, signal_msg); //makes msg_buffer the message above
            //DRV_USART_BufferAddWrite(appData.app1USARTHandle, &(appData.app1BufferHandle), msg_buffer, 9);  


            //UNCOMMENT HERE FOR TEST ADC
            
            //adc_msg = createADCMsg(MSG_SUBJECT_ADC_DATA,MSG_ADDR_R1_NAV_TH,
            //        MSG_ADDR_R2_NAV_TH,'A','C'); 
            //SUBJ, FROM, TO, FRONT, LEFT, RIGHT
            //packADCMsg(msg_buffer, adc_msg);
            //DRV_USART_BufferAddWrite(appData.app1USARTHandle, &(appData.app1BufferHandle), msg_buffer, 16);  //16 for adc

            
            //UNCOMMENT FOR TEST LFM

            lf_msg = createLFMsg(MSG_SUBJECT_LINE_FOLLOWER_DATA,MSG_ADDR_R1_NAV_TH,
                    MSG_ADDR_R2_NAV_TH,'A','B','C');
            packLFMsg(msg_buffer,lf_msg);
            length = 15;
            DRV_USART_BufferAddWrite(appData.app1USARTHandle, &(appData.app1BufferHandle), msg_buffer, 15);  //15 for lf

            
             // Block until callback function sends to this queue (write completed)
            unsigned char waitBuf;
            xQueueReceive(appData.blockQueue, &waitBuf, portMAX_DELAY);

            // Switch to read state
            appData.state = APP_STATE_USART_REQUEST_READ;
            
            break;

        }
        
        case APP_STATE_USART_REQUEST_READ:
        {
            dbgOutputLoc(0x03);
            
            DRV_USART_BufferAddRead(appData.app1USARTHandle, &(appData.app1BufferHandle), &(appData.app1Read), 15); //read max bytes 

             // Block until callback function sends to this queue (read completed)
            unsigned char waitBuf;
            xQueueReceive(appData.blockQueue, &waitBuf, portMAX_DELAY);

            // Switch to sending to the nav thread
            appData.state = APP_STATE_USART_REQUEST_WRITE;
            
            break;
        }
        
        case APP_STATE_USART_REQUEST_WRITE:
        {
            dbgOutputLoc(0x05);
            
            //char msg[] = {'\r', '\n', 'R', 'X', 'D', ' '};
            
            if(appData.app1Read[0] != 2){ //indicates checksum is wrong
                //resend old message somehow
                resend=1;
            } 
            //resend = 1;
            if(resend == 1){
                //signal_msg = createSignalMsg(MSG_SUBJECT_SIGNAL,MSG_ADDR_R1_NAV_TH,
                //        MSG_ADDR_R2_NAV_TH,MSG_SIG_TURN_LEFT);
                //packSignalMsg(msg_buffer, signal_msg); //makes msg_buffer the message above

                //adc_msg = adc_msg;
                lf_msg = createLFMsg(MSG_SUBJECT_LINE_FOLLOWER_DATA,MSG_ADDR_R1_NAV_TH,
                        MSG_ADDR_R2_NAV_TH,'X','Z','X');
                packLFMsg(msg_buffer,lf_msg);
                DRV_USART_BufferAddWrite(appData.app1USARTHandle, &(appData.app1BufferHandle),msg_buffer, 15); //changed this line to send rec'd message

            }
            if(resend = 0){
                DRV_USART_BufferAddWrite(appData.app1USARTHandle, &(appData.app1BufferHandle),appData.app1Read, 15); //changed this line to send rec'd message
            }
            resend = 0;
            
            // Block until callback function sends to this queue (write completed)
            unsigned char waitBuf;
            xQueueReceive(appData.blockQueue, &waitBuf, portMAX_DELAY);

            // switch to request read state
            appData.state = APP_STATE_USART_REQUEST_READ;

            break;
        }
        
        case APP_STATE_ERR:
        {
            dbgOutputErr();
            //char msg[] = {'\r', '\n', 'X', 'X', 'X', ' '};

            //DRV_USART_BufferAddWrite(appData.app1USARTHandle, &(appData.app1BufferHandle),msg, 6); //changed this line to send rec'd message

            break;
        }

        /* The default state should never be executed. */
        default:
        {
            dbgOutputErr();
            break;
        }
    }
}

void APP_USARTBufferEventHandler(DRV_USART_BUFFER_EVENT event,
            DRV_USART_BUFFER_HANDLE handle, uintptr_t context)
    {
        switch(event)
        {
            case DRV_USART_BUFFER_EVENT_COMPLETE: ; // empty statement

                unsigned char garbage = 'g';
                xQueueSendFromISR(appData.blockQueue, &garbage, NULL);

                break;

            case DRV_USART_BUFFER_EVENT_ERROR:
                appData.state = -1;
                break;

            default:
                break;
        }
    }

/*******************************************************************************
 End of File
 */
