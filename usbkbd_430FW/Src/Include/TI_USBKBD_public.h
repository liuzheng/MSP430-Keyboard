/** @file   TI_USBKBD_public.h
 *  @brief  USBKBD public header file
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
#ifndef __TI_USBKBD_public_h
#define __TI_USBKBD_public_h

//
// Include files
//
#include "stdint.h"
#include "TI_USBKBD_hardware.h"
#include "TI_USBKBD_public_defs.h"
#include "TI_USBKBD_HUT.h"

#include "TI_USBKBD_ticktimer_public.h"
#include "TI_USBKBD_HIDKBD_report_public.h"
#include "TI_USBKBD_comm_protocol_public.h"
#include "TI_USBKBD_DKS_public.h"

//
// Macro definitions
//
/*! USBKBD Version (BCD format: 0x0101 = 1.0.1 */
#define USBKBD_VERSION         (0x0001)


/*! USBKBD Tick clock divider (uses ACLK=32768Hz as base) \n
    I.e. for 500Hz, set to 32768/500=65.536=66 */
#define USBKBD_TICKTIMER_DIV     (66)


/*! Magic key at the beggining of Table */
#define USBKBD_MAGIC_KEY                 (0xDEADC0DE)

/*! USBKBD Config Table Segment */
#define USBKBD_CONFIGCONST_SEGMENT       "USBKBD_ConfigConst"
/*! USBKBD Config Table Start Address from linker */
extern uint16_t _USBKBD_ConfigConst_Start;
/*! USBKBD Config Table End Address from linker */
extern uint16_t _USBKBD_ConfigConst_End;

/*! USBKBD ScanCodes Table Segment */
#define USBKBD_SCANCODES_SEGMENT       "USBKBD_ScanCodes"
/*! USBKBD ScanCodes Start Address from linker */
extern uint16_t D_USBKBD_ScanCodes_Start;
/*! USBKBD ScanCodes End Address from linker */
extern uint16_t D_USBKBD_ScanCodes_Emd;


//
// Public variables
//

//
// Public Function prototypes
//

//
// Type definitions
//
/*! HID interfaces */
typedef enum{
    HID_Keyboard=0,    // Keyboard HID interface
    HID_Custom          // HID custom interface
}HID_Intf_t;

/*! key value size */
typedef uint8_t USBKBD_dks_keyval_t;
/*! key value and modifier size */
typedef uint16_t USBKBD_dks_keyvalmod_t;


/*! Constants defining configuration of USBKBD */
typedef struct {
    uint32_t        MagicKey;           /*!< Magic Key */
    uint16_t        Version;            /*!< USBKBD version */
    uint16_t        ticktimer_div;      /*!< Tick timer divider(over ACLK) */
    uint16_t        debounce_cycles;    /*!< debounce cycles in tick counts */
    uint16_t        inactive_timeout;   /*!< number of tick counts before going to int mode*/
} USBKBD_config_const_t;

/*! ScanCode table for USBKBD */
typedef struct {
    uint32_t                MagicKey;           /*!< Magic Key    */
    USBKBD_dks_keyval_t        keycode[128];       /*!< All Keycodes */
} USBKBD_scancodest_t;


//
// Public variables
//
/*! USBKBD configuration table */
extern const USBKBD_config_const_t USBKBD_configconst_s;

#endif //__TI_USBKBD_public_h
