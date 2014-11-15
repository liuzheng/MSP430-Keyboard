/** @file   TI_USBKBD_ticktimer.c
 *  @brief  Tick timer driver
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

//
// Local Macros/defines
//

//
// Local function prototypes
//

//
// Local variables
//

//
// Public functions
//
/******************************************************************************
 * @brief   Initializes a generic tick timer
 *   Timer is halted after this function
 *
 * @return  none
 *****************************************************************************/
void TI_USBKBD_TickTimer_Init( void )
{
    // Configure Timer Trigger TA0.0
    // TA0.0 Period (32768/66= ~500Hz = 2ms)
    TA0CCR0 = USBKBD_configconst_s.ticktimer_div;

    TA0CCTL0 = CCIE;    // Enable Timer interrupt
    TA0CTL = TASSEL__ACLK + MC__STOP | TACLR;   // ACLK, stopped
}

/******************************************************************************
 * @brief   Signals a loop completion
 *   Useful to debug overflow
 *
 * @return  none
 *****************************************************************************/
void TI_USBKBD_TickTimer_LoopComplete( void )
{
}

/******************************************************************************
*
 * @brief   Tick Timer interrupt service routine
 *  Wakes-up the mcu, checks if there was an overflow (previous loop didn't finish
 *  in the specified interval
 *
 * @return  NONE
 *****************************************************************************/
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TickTimerIsr(void)
{
    TA0CCTL1 ^= CCIS0;                   // Create SW capture of CCR1
    //count = TA0CCR1;                    // Save result
    TA0CCTL1 ^= CCIS0;                   // Re-enable capture on CCR1


    LPM0_EXIT;     // exit LPM
    __no_operation();

}
