/** @file   TI_USBKBD_DKS.c
 *  @brief  Drive for Digital Keyscan
 *  @author Luis Reynoso
 *  @date   Apr 2011
 *  @note   History:
 *          Date        Author      Comment
 *          05/2011     LR          File created and added to GIT
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
// Macros
//

//
// Local prototypes
//
tBool isKeyMasked(uint8_t row, uint8_t col);

//
// Local variables
//
/*! KSO hardware mapping. Provides a Hardware Abstraction layer allowing
 *  for easier portability. Simply change the port and pin according to your
 *  hardware
 */
const USBKBD_Port_t USBKBD_DKS_DSO[KSO_PINS] = {
    { KSO0_PORT_2,  KSO0_2_0},
    { KSO1_PORT_2,  KSO1_2_1},
    { KSO2_PORT_2,  KSO2_2_2},
    { KSO3_PORT_2,  KSO3_2_3},
    { KSO4_PORT_2,  KSO4_2_4},
    { KSO5_PORT_2,  KSO5_2_5},
    { KSO6_PORT_2,  KSO6_2_6},
    { KSO7_PORT_2,  KSO7_2_7},
    { KSO8_PORT_3,  KSO8_3_0},
    { KSO9_PORT_3,  KSO9_3_1},
    { KSO10_PORT_3, KSO10_3_2},
    { KSO11_PORT_3, KSO11_3_3},
    { KSO12_PORT_3, KSO12_3_4},
    { KSO13_PORT_5, KSO13_5_0},
    { KSO14_PORT_5, KSO14_5_1},
    { KSO15_PORT_5, KSO15_5_4},
};

extern const USBKBD_scancodest_t USBKBD_scancodes_s;
__no_init uint8_t counter_key[KSO_PINS*KSI_PINS];
__no_init static  uint8_t    col_pressed_by_row[KSO_PINS];
__no_init static  uint8_t    col_pressed_by_row_prev[KSO_PINS];
__no_init static DKS_mode_t   DKS_mode;
__no_init static uint16_t     inactive_counter;

//
// Public functions
//
/******************************************************************************
*
 * @brief   Digital Keyscan initialization
 *  Initializes all registers, variables for digital keyscan
 *
 * @return  none
 *****************************************************************************/
void TI_USBKBD_DKS_Init(void)
{
    uint8_t i;
    // Init variables
    inactive_counter = 0;
    for (i=0; i < KSO_PINS; i++)
    {
        col_pressed_by_row[i] = 0;
        col_pressed_by_row_prev[i] = 0;
    }
    for (i=0; i < KSO_PINS*KSI_PINS; i++)
    {
        counter_key[i] = 0;
    }

    // Set KSO as output high to detect interrupts
    SET_KSO_INTERRUPT();

    // Check if a key is pressed
    if (KSI_PORTR_1 == 0x00)
    {
        // If no key pressed, wait for interrupt
        DKS_mode = DKS_interrupt;   // Set DKS in interrupt-column mode
        KSI_IES_1 &= ~KSI_ALL;      // Set rising edge interrupt
        KSI_IFG_1 &= ~KSI_ALL;      // clear flags
        KSI_IE_1 |= KSI_ALL;        // Enable KSI interrupt
        TI_USBKBD_TICKTIMER_STOP(); // Stop polling timer
    }
    else
    {
        // A key is pressed, work in polling mode
        SET_KSO_POLL();             // Set pins as inputs to start polling
        DKS_mode = DKS_polling;     // Set DKS in polling mode
        KSI_IE_1 &= ~KSI_ALL;       // Disable interrupts
        TI_USBKBD_TICKTIMER_START();// Start the polling timer

        inactive_counter = 0;
    }
}

/******************************************************************************
 *
 * @brief   Disable Digital Keyscan
 *  Disable the digital Keyscan
 *
 * @return  none
 *****************************************************************************/
void TI_USBKBD_DKS_Disable(void)
{
    // Disable DSO interrupt and timer
    SET_KSO_INTERRUPT();                // Set KSO as input with pull-down
    KSI_IE_1 &= ~KSI_ALL;               // Disable DKS interrupt
    DKS_mode = DKS_interrupt;           // Set DKS in interrupt mode (but disabled)
    TI_USBKBD_TICKTIMER_STOP();         // Stop polling timer
}




/******************************************************************************
 *
 * @brief   Digital Keyscan poll
 *  Polls the digital keys
 *
 * @return  none
 *****************************************************************************/
void TI_USBKBD_DKS_Poll(void)
{
    uint8_t row, col;
    uint8_t * port_ptr;
    uint8_t * port_dir;
    uint8_t key;
    tBool  key_pressed = FALSE_t;

    // Only check DKS if a key press was detected previosly
    if (DKS_mode == DKS_polling)
    {
        // Read all rows
        for (row=0; row < KSO_PINS; row++)
        {
            // Set pointers for PxOUT and PxDIR registers
            port_ptr = (uint8_t *) USBKBD_DKS_DSO[row].portw;
            port_dir = (uint8_t *) USBKBD_DKS_DSO[row].portw + 2;
            // Set KSO pin as output High
            *port_dir |= USBKBD_DKS_DSO[row].port_mask;
            *port_ptr |= USBKBD_DKS_DSO[row].port_mask;
            // Wait a few cycles before reading KSI
            __delay_cycles(DELAY_DKS_KSO);
            // Read the KSI pins (processed later to remove ghost keys)
            col_pressed_by_row[row] = KSI_PORTR_1;
            if (col_pressed_by_row[row] != 0x00)
                key_pressed = TRUE_t;   // Check if any key is pressed
            // Set KSO pin as input with pull-down to avoid shorts
            *port_ptr &= ~USBKBD_DKS_DSO[row].port_mask;
            *port_dir &= ~USBKBD_DKS_DSO[row].port_mask;
            // Set KSI pins as output low to bleed off the charge
            KSI_PORTDIR_1 |= KSI_ALL;
            KSI_PORTW_1 &= ~KSI_ALL;
            // Return KSI to Input
            KSI_PORTDIR_1 &= ~KSI_ALL;
        }

        // Process keys (row and col)
        for (row=0; row < KSO_PINS; row++)
        {
            // If a key is pressed in row, or was pressed before, check which column
            if ((col_pressed_by_row[row] != 0x00) || (col_pressed_by_row_prev[row] != 0x00))
            {
                 for (col =0; col < KSI_PINS; col++)
                {
                    // Scan which column is pressed in current row
                    key = (row*KSI_PINS)+col;
                    if (col_pressed_by_row[row] & (1<<col))
                    {
                        // If col+row is pressed, check debounce and ghost key
                        if (isKeyMasked(row,col) == FALSE_t)
                        {
                            // A key was pressed and is not masked
                            if (counter_key[key] < USBKBD_configconst_s.debounce_cycles)
                            {
                                // Just increment the debounce counter
                                counter_key[key]++;
                            }
                            else if (counter_key[key] == USBKBD_configconst_s.debounce_cycles)
                            {
                                // Add key to the report after debounce delay
                                 if ( TI_USBKBD_Report_Addkey(USBKBD_scancodes_s.keycode[key]) == TRUE_t)
                                        counter_key[key]++;
                            }
                        }
                    }
                    else if (counter_key[key] != 0)
                    {
                        // If current col+row is not pressed, remove from report
                        // if it was pressed previously (or just restart counter)
                        TI_USBKBD_Report_Removekey(USBKBD_scancodes_s.keycode[key]);
                        counter_key[key] = 0;
                    }
                }
            }
            // Back-up column value
            col_pressed_by_row_prev[row] = col_pressed_by_row[row];
        }
    }
    else
    {
        // An interrupt will force polling mode
    }

    // If any key is pressed, reset the inactive counter
    if (key_pressed == TRUE_t)
    {
        inactive_counter = 0;
    }
    else
    {
       // If no key pressed for some time, change to Interrupt mode
       if (inactive_counter++ >= USBKBD_configconst_s.inactive_timeout)
        {
                TI_USBKBD_TICKTIMER_STOP();
                SET_KSO_INTERRUPT();
                DKS_mode = DKS_interrupt;
                KSI_IFG_1 &= ~KSI_ALL;   // clear flags
                KSI_IE_1 |= KSI_ALL;
         }
     }
}

/******************************************************************************
 *
 * @brief   Checks for ghost keys
 *  checks if current column+row press is valid or it can be caused by potential
 *  ghost keys
 *
 *  @param row Row corresponding to key being checked
 *  @param col Column corresponding to key being checked
 *
 * @return  TRUE_t Key can be caused by a ghost key, FALSE_t key is valid
 *****************************************************************************/
tBool isKeyMasked(uint8_t row, uint8_t col)
{
    tBool ret = FALSE_t;
    uint8_t i;

    for (i=0; i < KSO_PINS; i++)
    {
        // Check if there are other columns pressed
        if ( (i != row) &&
             ( ((col_pressed_by_row[row] & col_pressed_by_row[i]) != 0x00) &&
               ((col_pressed_by_row[row] & col_pressed_by_row[i]) != (1<<col)) ) )
        {
            // Report if there's a potential ghost key (current col and row
            // is already pressed) so that the key is ignored
            ret = TRUE_t;
        }
    }

    return ret;
}

/******************************************************************************
*
 * @brief   KSI_Isr
 *  Wakes-up the mcu due to a key press and goes to polling mode
 *
 * @return  NONE
 *****************************************************************************/
#pragma vector=KSI_VECTOR
__interrupt void KSI_Isr(void)
{
    // Set KSO pins to start polling mode
    SET_KSO_POLL();
    DKS_mode = DKS_polling;
    KSI_IE_1 &= ~KSI_ALL;
    inactive_counter = 0;
    // Start the polling timer
     TI_USBKBD_TICKTIMER_START();

    LPM0_EXIT;     // exit LPM
    __no_operation();

}
