/** @file   TI_USBKBD_DKS_public.h
 *  @brief  Digital Keyscan header file
 *  @author Luis Reynoso
 *  @date   July 2011
 *  @note   History:
 *          Date        Author      Comment
 *          07/2011     LR          File created and added to GIT
 *
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
#ifndef __TI_USBKBD_DKS_public_h
#define __TI_USBKBD_DKS_public_h

#define KSO_PINS            16      // Number of KSO (row) pins
#define KSI_PINS            8       // Number of KSI (column) pins
#define DELAY_DKS_KSO       5       // Delay after setting KSO output
#define DEBOUNCE_CYCLES     2       //  Debounce cycles (in tick counts)
#define INACTIVITY_TIMEOUT  8       // timeout before going to interrupt mode (in tick counts)

/*! Digital Keyscan mode */
typedef enum{
    DKS_interrupt=0,    // No key pressed, use interrupt mode
    DKS_polling        // Key detected, use polling mode
}DKS_mode_t;


//
// Public prototypes
//
void TI_USBKBD_DKS_Init(void);
void TI_USBKBD_DKS_Poll(void);
void TI_USBKBD_DKS_Disable(void);


#endif //__TI_USBKBD_DKS_public_h
