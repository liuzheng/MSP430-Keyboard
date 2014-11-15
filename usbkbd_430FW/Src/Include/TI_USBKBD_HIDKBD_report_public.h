/** @file   TI_USBKBD_HIDKBD_report_public.h
 *  @brief  HID Keyboard report builder public file
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
#ifndef __TI_USBKBD_HIDKBD_report_public_h
#define __TI_USBKBD_HIDKBD_report_public_h

#include "TI_USBKBD_public_defs.h"

//
// MACROS/DEFINES
//
/*! HID report buffer size (1 modifier, 1 unused, 6 data) */
#define REPORT_BUFFER_SIZE  8
/*! MODIFIER offset in report buffer */
#define MODIFIER_OFFSET     0
/*! Data offset in report buffer */
#define DATA_OFFSET         2
/*! Mark all data slots high */
#define ALL_SLOTS           0xFC



//
// Type definition
//
/*! type definition of key value */
typedef  uint8_t key_t;

/*! HID Keyboard buffer */
typedef struct
{
    tBool               Update;                     /**< Update report */
    USBKBD_8bitreg_t       EmptySlots;                 /**< emptyslots */
    uint8_t             Size;                       /**< Queue size */
    key_t               Buff[REPORT_BUFFER_SIZE];   /**< Circular buffer elements */
} ReportBuffer_t;

//
// Public prototypes
//
void TI_USBKBD_Report_Init( void );
tBool TI_USBKBD_Report_Addkey(key_t keyvalue);
tBool TI_USBKBD_Report_Removekey(key_t keyvalue);
void TI_USBKBD_Report_Update(void);
tBool TI_USBKBD_Report_CanSleep(void);


#endif //__TI_USBKBD_HIDKBD_report_public_h
