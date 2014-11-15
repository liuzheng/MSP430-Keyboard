/** @file   TI_USBKBD_hardware.h
 *  @brief   USBKBD Hardware definition
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
#ifndef __TI_USBKBD_hardware_h
#define __TI_USBKBD_hardware_h

//
//  Header files
//
#include "msp430.h"

//
//  Configuration definitions
//
#ifndef USB_MCLK_FREQ
#define MCLK_FREQ   8000000
#else
#define MCLK_FREQ   USB_MCLK_FREQ
#endif

//
//  Constant HW definitions
//
/*! Port1 KSI[0:7] */
#define KSI_PORTR_1    (P1IN)       /*! KSI Read Port */
#define KSI_PORTW_1    (P1OUT)      /*! KSI Write Port */
#define KSI_PORTDIR_1  (P1DIR)      /*! KSI Dir */
#define KSI_IES_1      (P1IES)      /*! Interrupt Edge select */
#define KSI_IFG_1      (P1IFG)      /*! Interrupt Flag */
#define KSI_IE_1       (P1IE)       /*! Interrupt Enable */
#define KSI0_1_0        BIT0
#define KSI1_1_1        BIT1
#define KSI2_1_2        BIT2
#define KSI3_1_3        BIT3
#define KSI4_1_4        BIT4
#define KSI5_1_5        BIT5
#define KSI6_1_6        BIT6
#define KSI7_1_7        BIT7
#define KSI_ALL        (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7)
#define KSI_VECTOR      PORT1_VECTOR

/*! Port2 KS0[0:7] */
#define KSO0_PORT_2     ((uint16_t) &P2OUT)    /*! KSO0 Port */
#define KSO0_2_0        BIT0        /*! KSO0 Pin */
#define KSO1_PORT_2     ((uint16_t) &P2OUT)    /*! KSO1 Port */
#define KSO1_2_1        BIT1        /*! KSO1 Pin */
#define KSO2_PORT_2     ((uint16_t) &P2OUT)    /*! KSO2 Port */
#define KSO2_2_2        BIT2        /*! KSO2 Pin */
#define KSO3_PORT_2     ((uint16_t) &P2OUT)    /*! KSO3 Port */
#define KSO3_2_3        BIT3        /*! KSO3 Pin */
#define KSO4_PORT_2     ((uint16_t) &P2OUT)    /*! KSO4 Port */
#define KSO4_2_4        BIT4        /*! KSO4 Pin */
#define KSO5_PORT_2     ((uint16_t) &P2OUT)    /*! KSO5 Port */
#define KSO5_2_5        BIT5        /*! KSO5 Pin */
#define KSO6_PORT_2     ((uint16_t) &P2OUT)    /*! KSO6 Port */
#define KSO6_2_6        BIT6        /*! KSO6 Pin */
#define KSO7_PORT_2     ((uint16_t) &P2OUT)    /*! KSO7 Port */
#define KSO7_2_7        BIT7        /*! KSO7 Pin */
#define KSO_PORT2_ALL      (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7)

/*! Port3 KS0[8:12] */
#define KSO8_PORT_3     ((uint16_t) &P3OUT)    /*! KSO8 Port */
#define KSO8_3_0        BIT0        /*! KSO8 Pin */
#define KSO9_PORT_3     ((uint16_t) &P3OUT)    /*! KSO9 Port */
#define KSO9_3_1        BIT1        /*! KSO9 Pin */
#define KSO10_PORT_3    ((uint16_t) &P3OUT)   /*! KSO10 Port */
#define KSO10_3_2       BIT2        /*! KSO10 Pin */
#define KSO11_PORT_3    ((uint16_t) &P3OUT)   /*! KSO11 Port */
#define KSO11_3_3       BIT3        /*! KSO11 Pin */
#define KSO12_PORT_3    ((uint16_t) &P3OUT)   /*! KSO12 Port */
#define KSO12_3_4       BIT4        /*! KSO12 Pin */
#define KSO_PORT3_ALL   (BIT0|BIT1|BIT2|BIT3|BIT4)

/*! Port4 */
#define LED_PORT_W      P4OUT           /*! LED Output Port */
#define UNUSED_4_0      BIT0
#define I2C_SDA_4_1     BIT1            /*! I2C SDA */
#define I2C_SCL_4_2     BIT2            /*! I2C SCL */
#define UNUSED_4_3      BIT3
#define UNUSED_4_4      BIT4
#define UNUSED_4_5      BIT5
#define LEDNUM_4_6      BIT6            /*! LED2 pin */
#define LEDCAPS_4_7     BIT7            /*! LED1 pin */

/*! Port5 */
#define KSO13_PORT_5    ((uint16_t) &P5OUT)    /*! KSO13 Port */
#define KSO13_5_0       BIT0        /*! KSO13 Pin */
#define KSO14_PORT_5    ((uint16_t) &P5OUT)   /*! KSO14 Port */
#define KSO14_5_1       BIT1        /*! KSO14 Pin */
#define XT2IN_5_2       BIT2
#define XT2OUT_5_3      BIT3
#define KSO15_PORT_5    ((uint16_t) &P5OUT)   /*! KSO15 Port */
#define KSO15_5_4       BIT4        /*! KSO15 Pin */
#define KBD_GP_5_5      BIT5
#define KSO_PORT5_ALL   (BIT0|BIT1|BIT4)

/*! Port6 (ADC Channels) */
#define UNUSED_6_0      BIT0
#define UNUSED_6_1      BIT1
#define UNUSED_6_2      BIT2
#define UNUSED_6_3      BIT3
#define UNUSED_6_4      BIT4
#define UNUSED_6_5      BIT5
#define UNUSED_6_6      BIT6
#define UNUSED_6_7      BIT7

/*! PortJ */
#define UNUSED_J_0      BIT0
#define UNUSED_J_1      BIT1
#define UNUSED_J_2      BIT2
#define UNUSED_J_3      BIT3

// Keyboard macros
/*! Set all KSO pins as output high to detect interrupt on KSI pins */
#define SET_KSO_INTERRUPT() {P2OUT |= KSO_PORT2_ALL; P2DIR |= KSO_PORT2_ALL;\
                            P3OUT |= KSO_PORT3_ALL; P3DIR |= KSO_PORT3_ALL;\
                            P5OUT |= KSO_PORT5_ALL; P5DIR |= KSO_PORT5_ALL;}
/*! Set KSO pins as input with pull-down to poll each row */
#define SET_KSO_POLL()      {P2OUT &= ~KSO_PORT2_ALL; P2DIR &= ~KSO_PORT2_ALL; P2REN |= KSO_PORT2_ALL;\
                            P3OUT &= ~KSO_PORT3_ALL; P3DIR &= ~KSO_PORT3_ALL; P3REN |= KSO_PORT3_ALL;\
                            P5OUT &= ~KSO_PORT5_ALL; P5DIR &= ~KSO_PORT5_ALL; P5REN |= KSO_PORT5_ALL; }

#endif //__TI_USBKBD_hardware_h
