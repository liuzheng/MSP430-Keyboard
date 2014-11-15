/** @file   TI_USBKBD_comm_protocol.c
 *  @brief  HID Communication protocol
 *  @author Luis Reynoso
 *  @date   July 2011
 *  @note   History:
 *          Date        Author      Comment
 *          07/2011     LR          File created and added to GIT
 */
/* --COPYRIGHT--,BSD
 * Copyright (c) 2011, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//
// Include files
//
#include "TI_USBKBD_public.h"
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/usb.h"        // USB-specific functions
#include "usbConstructs.h"
#include <string.h>

#ifdef _HID_
    #include "USB_API/USB_HID_API/UsbHid.h"
#endif



//
// Local function prototypes
//
static void CommProtocol_Parse(uint8_t * buffer);

//
// Local variables
//
/*! Local flag updated when there's new data on custom HID interface */
volatile tBool bDataReceived_custom;
/*! Receive buffer */
__no_init static uint8_t RxBuff[COMM_PROTOCOL_RX_SIZE];

//
// Public functions
//
/******************************************************************************
 * @brief   Initializes the HID Keyboard Buffer
 *  Initializes all queues and pointers
 *
 * @return  none
 *****************************************************************************/
void TI_USBKBD_CommProtocol_Init( void )
{
    uint8_t i;
    // Initialize variables and buffer
    bDataReceived_custom = FALSE_t;
    for (i=0; i < COMM_PROTOCOL_RX_SIZE; i++)
        RxBuff[i] = 0x00;
}



/******************************************************************************
 * @brief  Add key to the report buffer
 *   Checks the buffer to see if there's space and adds a new key
 *
 * @param keyvalue value of key being added to buffer
 * @return  TRUE_t if added, FALSE_t if buffer full
 *****************************************************************************/
void TI_USBKBD_CommProtocol_Poll( void )
{
    uint16_t count;

    if (bDataReceived_custom == TRUE_t)
    {
        // If data was received in "custom" interface, process it
        count = hidReceiveDataInBuffer((uint8_t *)RxBuff,COMM_PROTOCOL_RX_SIZE,HID_Custom);
        if (count >= 1)
        {
            CommProtocol_Parse(&RxBuff[0]);
        }
        bDataReceived_custom = FALSE_t; // Clear the flag
    }

    return;
}


/******************************************************************************
 * @brief  Processes the data from custom HID interface
 *   This function is only included as an example and can be used as a base
 *   for different applications
 *
 * @param buffer pointer to the received data
 * @return  none
 *****************************************************************************/
static void CommProtocol_Parse(uint8_t *buffer)
{
    char outString[COMM_PROTOCOL_TX_SIZE] = "";     // Holds the outgoing string

    switch (buffer[0])
    {
        case '1':
            // toggle LED1
            LED_PORT_W ^= LEDCAPS_4_7;
            // Prepare the outgoing string
            if (LED_PORT_W & LEDCAPS_4_7)
                strcpy(outString,"CAPS LED is ON\r\n");
            else
                strcpy(outString,"CAPS LED is OFF\r\n");
        break;
        case '2':
            // toggle LED2
            LED_PORT_W ^= LEDNUM_4_6;
            // Prepare the outgoing string
            if (LED_PORT_W & LEDNUM_4_6)
                strcpy(outString,"NUM LED is ON\r\n");
            else
                strcpy(outString,"NUM LED is OFF\r\n");
        break;
        default:
            // Invalid command
            strcpy(outString,"Invalid command\r\n\r\n");
        break;

    }
        // Wait until data is sent or error
         while (hidSendDataInBackground((BYTE*)outString, strlen(outString), HID_Custom, 0) == 1)
             ;

}

