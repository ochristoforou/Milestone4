/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include <debug.h>
#include <app.h>

void dbgOutputVal(unsigned char outVal) {
    int bitArray[8];
    int multiplier = 1, i = 0;
    for (i = 0; i < 8; i++)
    {
        bitArray[i] = outVal & (multiplier);
        multiplier = multiplier * 2;
    }
    
    //Pin 37
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0, bitArray[0]);
    //Pin 36
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1, bitArray[1]);
    //Pin 35
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2, bitArray[2]);
    //Pin 34
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_3, bitArray[3]);
    //Pin 33
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4, bitArray[4]);
    //Pin 32
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_5, bitArray[5]);
    //Pin 31
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6, bitArray[6]);
    //Pin 30
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_7, bitArray[7]);    
}

void dbgOutputLoc(unsigned char outLoc){
    
    int bitArray[8];
    int multiplier = 1, i = 0;
    for (i = 0; i < 8; i++)
    {
        bitArray[i] = outLoc & (multiplier);
        multiplier = multiplier * 2;
    }
    
    //Pin 47
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_6, bitArray[0]);
    //Pin 46
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_1, bitArray[1]);
    //Pin 45
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_0, bitArray[2]);
    //Pin 44
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_10, bitArray[3]);
    //Pin 43
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, bitArray[4]);
    //Pin 42
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_12, bitArray[5]);
    //Pin 41
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_13, bitArray[6]);
    //Pin 40
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_11, bitArray[7]); 
}

void dbgOutputErr() {
    while (1) {
        dbgOutputLoc(0xFF);
    }   
}
/* *****************************************************************************
 End of File
 */
