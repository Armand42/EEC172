//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Blinky
// Application Overview - The objective of this application is to showcase the 
//                        GPIO control using Driverlib api calls. The LEDs 
//                        connected to the GPIOs on the LP are used to indicate 
//                        the GPIO output. The GPIOs are driven high-low 
//                        periodically in order to turn on-off the LEDs.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_Blinky_Application
// or
// docs\examples\CC32xx_Blinky_Application.pdf
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup blinky
//! @{
//
//****************************************************************************

// Standard includes
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "utils.h"
#include "uart.h"
// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"
#include "pinmux.h"
//#include "string.h"
#define APPLICATION_VERSION     "1.1.1"

//*****************************************************************************
//                          MACROS
//*****************************************************************************
#define APPLICATION_VERSION  "1.1.1"
#define APP_NAME             "UART Echo"
#define CONSOLE              UARTA0_BASE
#define UartGetChar()        MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)       MAP_UARTCharPut(CONSOLE,c)
#define MAX_STRING_LENGTH    80
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile int g_iCounter = 0;
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************
void LEDBlinkyRoutine();
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                         
//*****************************************************************************

//*****************************************************************************
//
//! Configures the pins as GPIOs and peroidically toggles the lines
//!
//! \param None
//! 
//! This function  
//!    1. Configures 3 lines connected to LEDs as GPIO
//!    2. Sets up the GPIO pins as output
//!    3. Periodically toggles each LED one by one by toggling the GPIO line
//!
//! \return None
//
//*****************************************************************************
void LEDBlinkyRoutine()
{
    //
    // Toggle the lines initially to turn off the LEDs.
    // The values driven are as required by the LEDs on the LP.
    //
    GPIO_IF_LedOff(MCU_ALL_LED_IND);
    while(1)
    {
        //
        // Alternately toggle hi-low each of the GPIOs
        // to switch the corresponding LED on/off.
        //
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOff(MCU_RED_LED_GPIO);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
    }

}
//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//! This function  
//!    1. Invokes the LEDBlinkyTask
//!
//! \return None.
//
//****************************************************************************
void poll(int blink){
    int SW_2 = GPIOPinRead(0x40006000, 0x40);
    int first = 0;
    while (SW_2 > 0){
            if (first == 0)
                Message("SW2 Pressed\n\r");
                first = first + 1;
            if (blink == 0){
                GPIO_IF_LedOn(9);
                GPIO_IF_LedOn(10);
                GPIO_IF_LedOn(11);
                blink = 1;
                MAP_UtilsDelay(16000000);
            }
            else{
                GPIO_IF_LedOff(9);
                GPIO_IF_LedOff(10);
                GPIO_IF_LedOff(11);
                blink = 0;
                MAP_UtilsDelay(16000000);
            }
            SW_2 = GPIOPinRead(0x40006000, 0x40);
        }

}
int
main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();
    
    //
    // Power on the corresponding GPIO port B for 9,10,11.
    // Set up the GPIO lines to mode 0 (GPIO)
    //
    PinMuxConfig();
    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    GPIO_IF_LedOff(MCU_ALL_LED_IND);
    InitTerm();
    ClearTerm();
    Message("*************************************************\n\r");
    Message("\tCC3200 GPIO Application\n\r");
    Message("*************************************************\n\n\n\r");
    Message("****************************************************\n\r");
    Message("\tPush SW3 to start LED binary counting\n\r");
    Message("\tPush SW2 to blink LEDs on and off\n\r");
    Message("****************************************************\n\r");
    //
    // Start the LEDBlinkyRoutine
    //
    //GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    char * word = "check is: ";
    char number[5];
    int SW_2;
    int SW_3;
    int blink = 0;
    while(1){
    int check = 0;
    SW_2 = GPIOPinRead(0x40006000, 0x40);
    //check 64 is SW2 ON
    SW_3= GPIOPinRead(0x40005000, 0x20);
    //check 32 is SW3 ON
    //Message(word);
    //sprintf(number, "%d", check);
    //Message(number);
    MAP_UtilsDelay(8000000);
    if (SW_3 > 0){
        Message("SW3 Pressed\n\r");
        GPIO_IF_LedOn(9);
        poll(blink);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOff(9);
        GPIO_IF_LedOn(10);
        poll(blink);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOn(9);
        poll(blink);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOff(9);
        GPIO_IF_LedOff(10);
        GPIO_IF_LedOn(11);
        poll(blink);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOn(9);
        poll(blink);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOff(9);
        GPIO_IF_LedOn(10);
        poll(blink);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOn(9);
        poll(blink);
        MAP_UtilsDelay(16000000);
        GPIO_IF_LedOff(MCU_ALL_LED_IND);
    }
    if (SW_2 > 0){
        Message("SW2 Pressed\n\r");
        if (blink == 0){
        GPIO_IF_LedOn(9);
        GPIO_IF_LedOn(10);
        GPIO_IF_LedOn(11);
        blink = 1;
        MAP_UtilsDelay(8000000);
        }
        else{
            GPIO_IF_LedOff(9);
            GPIO_IF_LedOff(10);
            GPIO_IF_LedOff(11);
            blink = 0;
            MAP_UtilsDelay(8000000);
        }
    }
    //LEDBlinkyRoutine();
    }
    return 0;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
