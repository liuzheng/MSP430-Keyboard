/** @file   TI_USBKBD_SharedTables.c
 *  @brief  Configuration and scancode tables
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

//
// Local function prototypes
//

//
// Public variables
//

/*! Configuration constant table :
 *  Located in a reserved memory location.
 *  Can  be used to adjust keyboard settings without affecting the rest of the code
**/
#ifdef __IAR_SYSTEMS_ICC__
    #pragma location=USBKBD_CONFIGCONST_SEGMENT
    #pragma data_alignment=512
#elif defined (__TI_COMPILER_VERSION__)
    #pragma DATA_SECTION(USBKBD_configconst_s, ".USBKBD_CONFIGCONST_SEGMENT")
    #pragma DATA_ALIGN(USBKBD_configconst_s, 512)
#endif
const USBKBD_config_const_t USBKBD_configconst_s = {
    USBKBD_MAGIC_KEY,           /*!< MagicKey */
    USBKBD_VERSION,             /*!< Version */
    USBKBD_TICKTIMER_DIV,       /*!< ticktimer_div */
    DEBOUNCE_CYCLES,            /*!< debounce_cycles */
    INACTIVITY_TIMEOUT,         /*!< inactive_timeout */
};

/*! Scancodes constant table
 *  Located in a reserved memory location
 *  Can be optimized according to different key maps
**/
#ifdef __IAR_SYSTEMS_ICC__
    #pragma location=USBKBD_SCANCODES_SEGMENT
    #pragma data_alignment=512
#endif
#ifdef __TI_COMPILER_VERSION__
    #pragma DATA_SECTION(USBKBD_scancodes_s, ".USBKBD_SCANCODES_SEGMENT")
    #pragma DATA_ALIGN(USBKBD_scancodes_s, 512)
#endif
const USBKBD_scancodest_t USBKBD_scancodes_s = {
    USBKBD_MAGIC_KEY,           /*!< MagicKey */
    {usbUsageKeypadEnter,      /* 0 , 0 */
    usbUsageKeypadMinus,       /* 0 , 1 */
    usbUsageKeypadPlus,        /* 0 , 2 */
    usbUsageKeypad2,           /* 0 , 3 */
    usbUsageKeypad3,           /* 0 , 4 */
    usbUsageKeypadPeriod,      /* 0 , 5 */
    usbUsageKeypad1,           /* 0 , 6 */
    usbUsageKeypadSlash,       /* 0 , 7 */
    usbUsageReserved,          /* 1 , 0 */
    usbUsageReserved,          /* 1 , 1 */
    usbUsageKeypad9,           /* 1 , 2 */
    usbUsageLeftGUI,           /* 1 , 3 */
    usbUsageKeypad7,           /* 1 , 4 */
    usbUsageHome,              /* 1 , 5 */
    usbUsagePageUp,            /* 1 , 6 */
    usbUsageKeypadNumlock,     /* 1 , 7 */
    usbUsageReserved,          /* 2 , 0 */
    usbUsageKeypad0,           /* 2 , 1 */
    usbUsageReserved,          /* 2 , 2 */
    usbUsageTab,               /* 2 , 3 */
    usbUsageTilde,             /* 2 , 4 */
    usbUsage1,                 /* 2 , 5 */
    usbUsageQ,                 /* 2 , 6 */
    usbUsageA,                 /* 2 , 7 */
    usbUsageRightAlt,          /* 3 , 0 */
    usbUsageLeftAlt,           /* 3 , 1 */
    usbUsageReserved,          /* 3 , 2 */
    usbUsageReserved,          /* 3 , 3 */
    usbUsageReserved,          /* 3 , 4 */
    usbUsageReserved,          /* 3 , 5 */
    usbUsageReserved,          /* 3 , 6 */
    usbUsageReserved,          /* 3 , 7 */
    usbUsageC,                 /* 4 , 0 */
    usbUsageSpacebar,          /* 4 , 1 */
    usbUsageF3,                /* 4 , 2 */
    usbUsageF4,                /* 4 , 3 */
    usbUsageCapsLock,          /* 4 , 4 */
    usbUsage3,                 /* 4 , 5 */
    usbUsageE,                 /* 4 , 6 */
    usbUsageD,                 /* 4 , 7 */
    usbUsageX,                 /* 5 , 0 */
    usbUsageZ,                 /* 5 , 1 */
    usbUsageF2,                /* 5 , 2 */
    usbUsageF1,                /* 5 , 3 */
    usbUsageEscape,            /* 5 , 4 */
    usbUsage2,                 /* 5 , 5 */
    usbUsageW,                 /* 5 , 6 */
    usbUsageS,                 /* 5 , 7 */
    usbUsageV,                 /* 6 , 0 */
    usbUsageB,                 /* 6 , 1 */
    usbUsageG,                 /* 6 , 2 */
    usbUsageT,                 /* 6 , 3 */
    usbUsage5,                 /* 6 , 4 */
    usbUsage4,                 /* 6 , 5 */
    usbUsageR,                 /* 6 , 6 */
    usbUsageF,                 /* 6 , 7 */
    usbUsageM,                 /* 7 , 0 */
    usbUsageN,                 /* 7 , 1 */
    usbUsageH,                 /* 7 , 2 */
    usbUsageY,                 /* 7 , 3 */
    usbUsage6,                 /* 7 , 4 */
    usbUsage7,                 /* 7 , 5 */
    usbUsageU,                 /* 7 , 6 */
    usbUsageJ,                 /* 7 , 7 */
    usbUsagePeriod,            /* 8 , 0 */
    usbUsageDownArrow,         /* 8 , 1 */
    usbUsageBackslash,         /* 8 , 2 */
    usbUsageF11,               /* 8 , 3 */
    usbUsageF10,               /* 8 , 4 */
    usbUsage9,                 /* 8 , 5 */
    usbUsageO,                 /* 8 , 6 */
    usbUsageL,                 /* 8 , 7 */
    usbUsageRightShift,        /* 9 , 0 */
    usbUsageLeftShift,         /* 9 , 1 */
    usbUsageReserved,          /* 9 , 2 */
    usbUsageReserved,          /* 9 , 3 */
    usbUsageReserved,          /* 9 , 4 */
    usbUsageReserved,          /* 9 , 5 */
    usbUsageReserved,          /* 9 , 6 */
    usbUsageReserved,          /* 9 , 7 */
    usbUsageComma,             /* 10 , 0 */
    usbUsageKeypadAsterisk,    /* 10 , 1 */
    usbUsageF7,                /* 10 , 2 */
    usbUsageF6,                /* 10 , 3 */
    usbUsageF5,                /* 10 , 4 */
    usbUsage8,                 /* 10 , 5 */
    usbUsageI,                 /* 10 , 6 */
    usbUsageK,                 /* 10 , 7 */
    usbUsageReserved,          /* 11 , 0 */
    usbUsageReserved,          /* 11 , 1 */
    usbUsageKeypad8,           /* 11 , 2 */
    usbUsageF9,                /* 11 , 3 */
    usbUsageReserved,          /* 11 , 4 */
    usbUsageReserved,          /* 11 , 5 */
    usbUsageReserved,          /* 11 , 6 */
    usbUsageLeftArrow,         /* 11 , 7 */
    usbUsageRightControl,      /* 12 , 0 */
    usbUsageLeftControl,       /* 12 , 1 */
    usbUsageReserved,          /* 12 , 2 */
    usbUsageReserved,          /* 12 , 3 */
    usbUsageReserved,          /* 12 , 4 */
    usbUsageReserved,          /* 12 , 5 */
    usbUsageReserved,          /* 12 , 6 */
    usbUsageReserved,          /* 12 , 7 */
    usbUsageSlash,             /* 13 , 0 */
    usbUsageUpArrow,           /* 13 , 1 */
    usbUsageMinus,             /* 13 , 2 */
    usbUsageF12,               /* 13 , 3 */
    usbUsage0,                 /* 13 , 4 */
    usbUsageP,                 /* 13 , 5 */
    usbUsageLeftBracket,       /* 13 , 6 */
    usbUsageSemicolon,         /* 13 , 7 */
    usbUsageApostrophe,        /* 14 , 0 */
    usbUsageEnter,             /* 14 , 1 */
    usbUsagePrintScreen,       /* 14 , 2 */
    usbUsageEnd,               /* 14 , 3 */
    usbUsageEqual,             /* 14 , 4 */
    usbUsageBackspace,         /* 14 , 5 */
    usbUsageRightBracket,      /* 14 , 6 */
    usbUsagePageDown,          /* 14 , 7 */
    usbUsageReserved,          /* 15 , 0 */
    usbUsageRightArrow,        /* 15 , 1 */
    usbUsageF8,                /* 15 , 2 */
    usbUsagePause,             /* 15 , 3 */
    usbUsageScrollLock,        /* 15 , 4 */
    usbUsageKeypad4,           /* 15 , 5 */
    usbUsageKeypad5,           /* 15 , 6 */
    usbUsageKeypad6,           /* 15 , 7 */
 }
};

//
// Public functions
//
