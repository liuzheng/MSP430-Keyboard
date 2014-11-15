/** @file    TI_USBKBD_public_defs.h
 *  @brief   USBKBD public definitions
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


#ifndef __TI_USBKBD_public_defs_h
#define __TI_USBKBD_public_defs_h

//
// Functions common return values
//
/*! Succesful return */
#define RET_OK              (1)
/*! Succesful return with return value */
#define RET_VAL_OK          (2)
/*! Parameter error */
#define RET_PARAM_ERR       (-1)
/*! Critical error */
#define RET_CRITICAL_ERR    (-2)
/*! Function/bus is busy */
#define RET_BUSY            (-3)

/*! Null pointer */
#define NULL                (0)

/*! Boolean type */
typedef enum
{
    FALSE_t =0,
    TRUE_t
} tBool;


/*! 8-bit register definition */
typedef union
{
        uint8_t byte;
        struct
        {
                uint8_t  b0  :1;
                uint8_t  b1  :1;
                uint8_t  b2  :1;
                uint8_t  b3  :1;
                uint8_t  b4  :1;
                uint8_t  b5  :1;
                uint8_t  b6  :1;
                uint8_t  b7  :1;
        }bit;
}USBKBD_8bitreg_t;

/*! Port+pin definition */
typedef struct {
    /*!< Port pointer */
    volatile uint16_t  portw;
    /*!< Port mask */
    uint8_t         port_mask;
} USBKBD_Port_t;


#endif //__TI_USBKBD_public_defs_h
