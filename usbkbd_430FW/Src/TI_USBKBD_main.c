/** @file   TI_USBKBD_main.c
 *  @brief  USB Keyboard main file
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
#include "pmm.h"
#include "ucs.h"

#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"          // Basic Type declarations

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/usb.h"        // USB-specific functions

#include "usbConstructs.h"

#ifdef _HID_
    #include "USB_API/USB_HID_API/UsbHid.h"
#endif

//
// Local function prototypes
//
static void Init_Clock(void);
static void Init_Ports(void);
static volatile uint8_t RxBuffKbd[MAX_PACKET_SIZE];

//
// Global variables
//
volatile tBool bDataReceived_kbd;// data received event

//
// Public functions
//
/******************************************************************************
 * @brief   Main USB Keyboard function
 *  Initializes the MCU and then jumps to a main loop routine checking the
 *  analog and digital keys
 *
 * @return  none
 *****************************************************************************/
void main( void )
{
    tBool   FirstTime;

    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;

    // Setup clock and core voltage
    PMM_setVCore(__MSP430_BASEADDRESS_PMM__, PMMCOREV_0);   // Initialize VCore

    Init_Clock();               // Init clocks
    Init_Ports();               // Init GPIOs

    PMM_disableSvsL (__MSP430_BASEADDRESS_PMM__);   // SVS Low side is turned off
    PMM_disableSvmL (__MSP430_BASEADDRESS_PMM__);   // Monitor low side is turned off
    PMM_disableSvmH (__MSP430_BASEADDRESS_PMM__);   // Monitor high side is turned off
    PMM_enableSvsH (__MSP430_BASEADDRESS_PMM__);    // SVS High side is turned on

    PMM_enableSvsHReset(__MSP430_BASEADDRESS_PMM__);// Enable POR on SVS Event
    PMM_SvsHEnabledInLPMFullPerf(__MSP430_BASEADDRESS_PMM__);   // SVS high side Full perf mode,
                                           // stays on in LPM3,enhanced protect
    // Wait until high side, low side settled
    while (((PMMIFG & SVSMLDLYIFG) == 0)&&((PMMIFG & SVSMHDLYIFG) == 0));
    PMM_clearPMMIFGS(__MSP430_BASEADDRESS_PMM__);

    USB_init();               // Init USB

    // Enable all USB-level events
    USB_setEnabledEvents(kUSB_VbusOnEvent+kUSB_VbusOffEvent+kUSB_dataReceivedEvent+
                        kUSB_receiveCompletedEvent+kUSB_UsbSuspendEvent+kUSB_UsbResumeEvent);

    TI_USBKBD_CommProtocol_Init();     // Initialize comm protocol

    TI_USBKBD_DKS_Init();          // Initialize Digital Keyscan

    TI_USBKBD_TickTimer_Init();    // Initialize the tick timer


    // Clear global HID receive flags
    bDataReceived_kbd = FALSE_t;

    // Set to signal first loop
    FirstTime = TRUE_t;

    __enable_interrupt();    // Enable interrupts

    // In case USB is already attached (meaning no VBUS event will
    // occur), manually start the connection
    if (USB_connectionInfo() & kUSB_vbusPresent)
      USB_handleVbusOnEvent();


    for (;;)
    {

       switch(USB_connectionState())
        {
         case (ST_USB_DISCONNECTED | ST_NOENUM_SUSPENDED):
         // Enter LPM3 w/interrupt.  Nothing for us to do while disconnected.
                FirstTime = TRUE_t;
                TI_USBKBD_DKS_Disable();
            __bis_SR_register(LPM3_bits + GIE); 	
            break;

         case ST_USB_CONNECTED_NO_ENUM:
                FirstTime = TRUE_t;
                TI_USBKBD_DKS_Disable();
            break;

         case ST_ENUM_ACTIVE:
            // Poll the USB Custom interface
            TI_USBKBD_CommProtocol_Poll();

            if (bDataReceived_kbd == TRUE_t)
            {
                // This flag is true if the Standard keyboard interface got data
                if (USBHID_receiveReport((uint8_t *)RxBuffKbd, HID_Keyboard) >= 1)
                {
                    // Check for LED usage data and turn on/off NUM and CAPS LEDs
                    if (RxBuffKbd[0] & 0x01)
                        LED_PORT_W |= (LEDNUM_4_6);
                    else
                        LED_PORT_W &= ~(LEDNUM_4_6);

                    if (RxBuffKbd[0] & 0x02)
                        LED_PORT_W |= (LEDCAPS_4_7);
                    else
                        LED_PORT_W &= ~(LEDCAPS_4_7);
                }
                bDataReceived_kbd = FALSE_t;
            }

            if (FirstTime == TRUE_t)
            {
                // Toggle LEDs just to signal that keyboard is ready
                LED_PORT_W &= ~(LEDNUM_4_6|LEDCAPS_4_7);
                __delay_cycles(2000000);
                LED_PORT_W |= (LEDNUM_4_6);
                __delay_cycles(2000000);
                LED_PORT_W |= (LEDNUM_4_6|LEDCAPS_4_7);
                __delay_cycles(2000000);
                LED_PORT_W &= ~(LEDNUM_4_6|LEDCAPS_4_7);

                // Re-initialize modules
                TI_USBKBD_DKS_Init();          // Initalize Digital Keyscan
                TI_USBKBD_Report_Init();    // Initialize USB Keyboard report

                FirstTime = FALSE_t;
            }

            // Scan the Digital keyboard
            PJDIR|= BIT3;
            PJOUT |= BIT3;
            TI_USBKBD_DKS_Poll();
            PJOUT &= ~BIT3;

            // Update USB Keyboard report
            TI_USBKBD_Report_Update();

            // Disable interrupts before checking if module can sleep
            __disable_interrupt();

            // Signal Loop completion to check for main loop time overflow
            TI_USBKBD_TickTimer_LoopComplete();


            // Enter LPM0 w/interrupt if nothing is pending
            if ((TI_USBKBD_Report_CanSleep() == TRUE_t))
                {
                    __bis_SR_register(LPM0_bits + GIE); 	
                    __no_operation();
                }
            else
                __enable_interrupt();

            break;

         case ST_ENUM_SUSPENDED:
                FirstTime = TRUE_t;
                TI_USBKBD_DKS_Disable();
             // Enter LPM3 w/interrupt.  Nothing for us to do while
              __bis_SR_register(LPM3_bits + GIE); 	
              break;
              // suspended.  (Remote wakeup isn't enabled in this example.)

         case ST_ENUM_IN_PROGRESS:
                FirstTime = TRUE_t;
                TI_USBKBD_DKS_Disable();
              break;

         case ST_ERROR:
                FirstTime = TRUE_t;
                TI_USBKBD_DKS_Disable();
              break;

         default:;
        }

    }
}


//
// Private functions
//
/******************************************************************************
*
 * @brief   Clock initialization
 *  Initializes the clocks for the USB and rest of the system
 *
 * @return  none
 *****************************************************************************/
static void Init_Clock(void)
{
  #if defined (__MSP430F563x_F663x)
  while(BAKCTL & LOCKIO)                    // Unlock XT1 pins for operation
    BAKCTL &= ~(LOCKIO);                  // enable XT1 pins
  // Workaround for USB7
  UCSCTL6 &= ~XT1OFF;
  #endif
  //Initialization of clock module
  if (USB_PLL_XT == 2)
  {
    #if defined (__MSP430F552x) || defined (__MSP430F550x)
    P5SEL |= 0x0C;                        // enable XT2 pins for F5529
    #elif defined (__MSP430F563x_F663x)
    P7SEL |= 0x0C;
    #endif

    // use REFO for FLL and ACLK
    UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (SELREF__REFOCLK);
    UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);
    // Set FLL (DCOCLK)
    UCS_initFLLSettle( __MSP430_BASEADDRESS_UCS__,
                       USB_MCLK_FREQ/1000,
                       USB_MCLK_FREQ/32768);
  }
  else
  {
    #if defined (__MSP430F552x) || defined (__MSP430F550x)
    P5SEL |= 0x10;                    // enable XT1 pins
    #endif
    UCSCTL3 = SELREF__REFOCLK;            // run FLL mit REF_O clock
    UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK); // set ACLK = REFO
    // Set FLL (DCOCLK)
    UCS_initFLLSettle( __MSP430_BASEADDRESS_UCS__,
                       USB_MCLK_FREQ/1000,
                       USB_MCLK_FREQ/32768);
  }
}

/******************************************************************************
*
 * @brief   GPIO initialization
 *  Initializes all the GPIOs
 *
 * @return  none
 *****************************************************************************/
static void Init_Ports(void)
{
    // KSI pins have external pull-down
    P1DIR &= ~(KSI0_1_0 | KSI1_1_1 | KSI2_1_2 | KSI3_1_3|
             KSI4_1_4 |  KSI5_1_5 | KSI6_1_6 | KSI7_1_7);

    // KSO pins as outputs
    P2OUT &= ~(KSO0_2_0|KSO1_2_1|KSO2_2_2|KSO3_2_3|
               KSO4_2_4|KSO5_2_5|KSO6_2_6|KSO7_2_7);
    P2DIR = (KSO0_2_0|KSO1_2_1|KSO2_2_2|KSO3_2_3|
               KSO4_2_4|KSO5_2_5|KSO6_2_6|KSO7_2_7);

    P3OUT &= ~(KSO8_3_0|KSO9_3_1|KSO10_3_2|KSO11_3_3|KSO12_3_4);
    P3DIR = (KSO8_3_0|KSO9_3_1|KSO10_3_2|KSO11_3_3|KSO12_3_4);

    P4OUT &= ~(UNUSED_4_0|UNUSED_4_4|UNUSED_4_5|LEDNUM_4_6|LEDCAPS_4_7);
    P4DIR = (UNUSED_4_0|UNUSED_4_4|UNUSED_4_5|LEDNUM_4_6|LEDCAPS_4_7);
    //P4DIR &= ~(I2C_SDA_4_1|I2C_SCL_4_2|UNUSED_4_3);

    P5OUT &= ~(KSO13_5_0|KSO14_5_1|XT2IN_5_2|XT2OUT_5_3|KSO15_5_4|KBD_GP_5_5);
    P5DIR = (KSO13_5_0|KSO14_5_1|XT2IN_5_2|XT2OUT_5_3|KSO15_5_4|KBD_GP_5_5);

    P6OUT &= ~(UNUSED_6_0|UNUSED_6_1|UNUSED_6_2|UNUSED_6_3|
               UNUSED_6_4|UNUSED_6_5|UNUSED_6_6|UNUSED_6_7);
    P6DIR = (UNUSED_6_0|UNUSED_6_1|UNUSED_6_2|UNUSED_6_3|
               UNUSED_6_4|UNUSED_6_5|UNUSED_6_6|UNUSED_6_7);

    PJOUT &= ~(UNUSED_J_0|UNUSED_J_1|UNUSED_J_2|UNUSED_J_3);
    PJDIR = (UNUSED_J_0|UNUSED_J_1|UNUSED_J_2|UNUSED_J_3);
}

/******************************************************************************
 *
 * @brief   System Non-Maskable interrupt fault ISR
 *  Handlees system interrupts
 *
 * @return  none
 *****************************************************************************/
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR(void)
{
  switch (__even_in_range(SYSUNIV, SYSUNIV_SYSBUSIV))
  {
    case SYSUNIV_NONE:
      __no_operation();
      break;
    case SYSUNIV_NMIIFG:
      __no_operation();
      break;
    case SYSUNIV_OFIFG:
      UCSCTL7 &= ~(DCOFFG+0+0+XT2OFFG); // Clear OSC flaut Flags fault flags
      SFRIFG1 &= ~OFIFG;                        // Clear OFIFG fault flag
      break;
    case SYSUNIV_ACCVIFG:
      __no_operation();
      break;
    case SYSUNIV_SYSBUSIV:
     // If bus error occured - the cleaning of flag and re-initializing of USB is required.
      SYSBERRIV = 0;            // clear bus error flag
      USB_disable();            // Disable
  }
}
