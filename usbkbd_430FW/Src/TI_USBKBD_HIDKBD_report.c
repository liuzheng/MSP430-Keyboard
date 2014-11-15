/** @file   TI_USBKBD_HIDKBD_report.c
 *  @brief  Builds the HID Keyboard report
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


#ifdef _HID_
    #include "USB_API/USB_HID_API/UsbHid.h"
#endif

//
// Local function prototypes
//
static tBool FindEmptySlot( uint8_t * slot );
tBool TI_USBKBD_Report_IsFull( void );
tBool TI_USBKBD_Report_IsEmpty( void );


//
// Local variables
//
/*! HID Report buffer */
__no_init static ReportBuffer_t Report_buff;


//
// Public functions
//
/******************************************************************************
 * @brief   Initializes the HID Keyboard Buffer
 *  Initializes all queues and pointers
 *
 * @return  none
 *****************************************************************************/
void TI_USBKBD_Report_Init( void )
{
    uint8_t i;

    // Initialize all buffers and pointers
    Report_buff.Update = FALSE_t;
    Report_buff.EmptySlots.byte = ALL_SLOTS;
    Report_buff.Size = REPORT_BUFFER_SIZE;
    for (i=0; i <REPORT_BUFFER_SIZE; i++)
        Report_buff.Buff[i] = 0;
}



/******************************************************************************
 * @brief  Add key to the report buffer
 *   Checks the buffer to see if there's space and adds a new key
 *
 * @param keyvalue value of key being added to buffer
 * @return  TRUE_t if added, FALSE_t if buffer full
 *****************************************************************************/
tBool TI_USBKBD_Report_Addkey( key_t keyvalue )
{
    uint8_t s;

    // Run this in case they key is already in buffer
    TI_USBKBD_Report_Removekey(keyvalue);

    // Check if this is a modifier
    if ((keyvalue & 0xF0) == 0xE0)
    {
        Report_buff.Buff[MODIFIER_OFFSET] |= (1<<(keyvalue & 0x0F));
        Report_buff.Update = TRUE_t;
        return TRUE_t;
    }
    else
    {
        // This is an element array
        if (FindEmptySlot(&s) == TRUE_t)
        {
            // Write key in buffer
            Report_buff.Buff[s] = keyvalue;
            // Clear emptyslot bit
            Report_buff.EmptySlots.byte &= ~(1<<s);
            Report_buff.Update = TRUE_t;
            return TRUE_t;
        }
        else
            return FALSE_t;
    }

}

/******************************************************************************
 * @brief  Remove Key from buffer
 *  Checks if a key is in a buffer and removes it (sending a release to HID)
 *
 *  @param keyvalue Key being removed from buffer
 * @return  TRUE_t if removed, FALSE_t if not found
 *****************************************************************************/
tBool TI_USBKBD_Report_Removekey(key_t keyvalue)
{
    uint8_t i;

    // Check if this is a modifier
    if ((keyvalue & 0xF0) == 0xE0)
    {
        Report_buff.Buff[MODIFIER_OFFSET] &= ~(1<<(keyvalue & 0x0F));
        Report_buff.Update = TRUE_t;
    }
    for (i=DATA_OFFSET; i < REPORT_BUFFER_SIZE; i ++)
    {
        // Look for key in buffer
        if ( (((Report_buff.EmptySlots.byte) & (1<<i)) == 0) &&
             (Report_buff.Buff[i]) == keyvalue )
        {
            Report_buff.EmptySlots.byte |= (1<<i);
            Report_buff.Buff[i] = 0;
            Report_buff.Update = TRUE_t;
            return TRUE_t;
        }
    }

    // Key wasn't found
    return FALSE_t;
}


/******************************************************************************
 * @brief  Sends an updated report to HID
 *  Sends an updated report to HID interface if a key was pressed/released
 *
 * @return  None
 *****************************************************************************/
void TI_USBKBD_Report_Update(void)
{

    if (Report_buff.Update == TRUE_t)
    {
        while (USBHID_sendReport((uint8_t *)&Report_buff.Buff[0], HID_Keyboard) == kUSBHID_intfBusyError)
            ;
        Report_buff.Update = FALSE_t;
    }
}


/******************************************************************************
*
 * @brief   Checks if there are pending reports or if this module can sleep
 *  Checks the queues for pending data
 *
 * @return  tBool FALSE_t: Report pending to send, TRUE_t: No reports to send
 *****************************************************************************/
tBool TI_USBKBD_Report_CanSleep(void)
{
    if (Report_buff.Update == FALSE_t)
        return TRUE_t;
    else
        return FALSE_t;
}


/******************************************************************************
 * @brief  Find empty slot
 *
 * @param slot  Written with empty slot if found
 * @return  TRUE_t if slot found, FALSE_t if buffer is full
 *****************************************************************************/
static tBool FindEmptySlot( uint8_t * slot )
{
    uint8_t i;

    if (TI_USBKBD_Report_IsFull() == FALSE_t)
    {
        for (i=DATA_OFFSET; i < REPORT_BUFFER_SIZE; i++)
        {
            if (((Report_buff.EmptySlots.byte) & (1<<i)) != 0x00)
            {
                *slot = i;
                return TRUE_t;
            }
        }
    }
    // Nothing found, return error
    return FALSE_t;
}

/******************************************************************************
 * @brief  Checks if the report buffer is full
 *
 * @return  TRUE_t if full, FALSE_t if not full
 *****************************************************************************/
tBool TI_USBKBD_Report_IsFull( void )
{
    if (Report_buff.EmptySlots.byte == 0x00)
        return TRUE_t;
    else
        return FALSE_t;
}

/******************************************************************************
 * @brief  Checks if the report buffer is empty
 *
 * @return  TRUE_t if empty, FALSE_t if not empty
 *****************************************************************************/
tBool TI_USBKBD_Report_IsEmpty( void )
{
     if (Report_buff.EmptySlots.byte == ALL_SLOTS)
        return TRUE_t;
    else
        return FALSE_t;
}
