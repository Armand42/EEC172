
//*****************************************************************************
//
// Application Name     - int_sw
// Application Overview - The objective of this application is to demonstrate
//                          GPIO interrupts using SW2 and SW3.
//                          NOTE: the switches are not debounced!
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup int_sw
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
#include "prcm.h"
#include "gpio.h"
#include "timer.h"
#include "utils.h"

// Common interface includes
#include "timer_if.h"
//#include "gpio_if.h"

// Common interface includes
#include "uart_if.h"

#include "pinmux.h"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);

volatile unsigned long SW2_intcount;
volatile unsigned long SW3_intcount;
volatile unsigned char SW2_intflag;
volatile unsigned char SW3_intflag;
volatile unsigned char Code;
volatile unsigned char Edge;
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;
volatile int buf[32];
volatile int index;
volatile int score;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

static PinSetting switch2 = { .port = GPIOA2_BASE, .pin = 0x40};

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************
void
TimerBaseIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(g_ulBase);

    g_ulTimerInts ++;
}

//*****************************************************************************
//
//! The interrupt handler for the second timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
void
TimerRefIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(g_ulRefBase);

    g_ulRefTimerInts ++;
}

static void GPIOA1IntHandler(void) { // SW3 handler
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (GPIOA1_BASE, true);
    MAP_GPIOIntClear(GPIOA1_BASE, ulStatus);        // clear interrupts on GPIOA1
    SW3_intcount++;
    SW3_intflag=1;
}


static void BothEdgeIntHandler(void) { // SW3 handler
    //Message("here\n");
    unsigned long ulStatus;
    int value = MAP_GPIOPinRead(GPIOA1_BASE, 0x10);
    //Report("%d\r\n", value);
    //Rising Edge
    if (value > 0){
        g_ulTimerInts = 0;
        Timer_IF_Start(g_ulBase, TIMER_A, 2000);
        //Edge = 1;
    }
    //Falling Edge
    else{
        //Report("Length: %d\r\n", g_ulTimerInts);
        if (index < 32){
            if (g_ulTimerInts <= 43 && g_ulTimerInts >= 15){
                buf[index] = 0;
                index++;
                score = (score << 1) | (0);
            }
            else if (g_ulTimerInts >= 44 && g_ulTimerInts < 70){
                buf[index] = 1;
                index++;
                score = (score << 1) | (1);
            }
            if (index == 32){
                Edge = 1;
            }
        }
        else{
            Timer_IF_Stop(g_ulBase, TIMER_A);
        }
        if (g_ulTimerInts >= 70){
            index = 0;
            score = 0;
        }
    }
    //Edge = 1;
    ulStatus = MAP_GPIOIntStatus (GPIOA1_BASE, true);
    MAP_GPIOIntClear(GPIOA1_BASE, ulStatus);
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
BoardInit(void) {
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);

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
//!
//! \return None.
//
//****************************************************************************
int main() {
    unsigned long ulStatus;

    BoardInit();

    PinMuxConfig();

    InitTerm();

    ClearTerm();
    //Message("HESFSDFSF\r\n");
    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(GPIOA1_BASE, BothEdgeIntHandler);
    //Register for pin 21 GPIOA3_BASE, 0x2
    //GPIOA1_BASE, 0x20
   // MAP_GPIOIntRegister(GPIOA2_BASE, BothEdgeIntHandler);
    //MAP_GPIOIntRegister(switch2.port, GPIOA2IntHandler);

    //
    // Configure rising edge interrupts on SW2 and SW3
    //
    MAP_GPIOIntTypeSet(GPIOA1_BASE, 0x10, GPIO_BOTH_EDGES);    // SW3
    //MAP_GPIOIntTypeSet(GPIOA2_BASE, 0x1, GPIO_BOTH_EDGES);    // IR GPIO
    //MAP_GPIOIntTypeSet(switch2.port, switch2.pin, GPIO_RISING_EDGE);    // SW2

    //ulStatus = MAP_GPIOIntStatus (GPIOA2_BASE, false);
    //MAP_GPIOIntClear(GPIOA2_BASE, ulStatus);
    ulStatus = MAP_GPIOIntStatus (GPIOA1_BASE, false);
    MAP_GPIOIntClear(GPIOA1_BASE, ulStatus); // clear interrupts on GPIOA1
    //ulStatus = MAP_GPIOIntStatus (switch2.port, false);

   // MAP_GPIOIntClear(switch2.port, ulStatus);           // clear interrupts on GPIOA2

    // clear global variables
    SW2_intcount=0;
    SW3_intcount=0;
    SW2_intflag=0;
    SW3_intflag=0;
    Edge = 0;
    Code = 0;
    index = 0;
    score = 0;
    // Enable SW2 and SW3 interrupts
    //MAP_GPIOIntEnable(GPIOA2_BASE, 0x1);
    MAP_GPIOIntEnable(GPIOA1_BASE, 0x10);
   // MAP_GPIOIntEnable(switch2.port, switch2.pin);

    //
    // Base address for first timer
    //
    g_ulBase = TIMERA0_BASE;
    //
    // Base address for second timer
    //
    g_ulRefBase = TIMERA1_BASE;
    //
    // Configuring the timers
    //
    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_Init(PRCM_TIMERA1, g_ulRefBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

    //
    // Setup the interrupts for the timer timeouts.
    //
    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);
    Timer_IF_IntSetup(g_ulRefBase, TIMER_A, TimerRefIntHandler);

    //
    // Turn on the timers feeding values in mSec
    //
    Timer_IF_Start(g_ulBase, TIMER_A, 2000);
    //Timer_IF_Start(g_ulRefBase, TIMER_A, 1000);

    Message("\t\t****************************************************\n\r");
    Message("\t\t\tPush SW3 or SW2 to generate an interrupt\n\r");
    Message("\t\t ****************************************************\n\r");
    Message("\n\n\n\r");
    //Report("SW2 ints = %d\tSW3 ints = %d\r\n",SW2_intcount,SW3_intcount);
    int i = 0;
    while (1) {
        while (Edge == 0) {;}
        //Report("Timer: %d\r\n", g_ulRefTimerInts);
        if (Edge){
            //Message("g_ulBase: %d\n, g_ulBase");
            /*
            for (i = 0; i < 32; i++){
                Report("%d", buf[i]);
            }
            Report("  Score: %d", score);
            Message("\r\n");
            */
            switch (score){
            case 79370471: Message("0\r\n"); break;
            case 79396991: Message("1\r\n"); break;
            case 79380671: Message("2\r\n"); break;
            case 79413311: Message("3\r\n"); break;
            case 79405151: Message("4\r\n"); break;
            case 79388831: Message("5\r\n"); break;
            case 79421471: Message("6\r\n"); break;
            case 79401071: Message("7\r\n"); break;
            case 79384751: Message("8\r\n"); break;
            case 79417391: Message("9\r\n"); break;
            case 79373021: Message("MUTE\r\n"); break;
            case 79401581: Message("LAST\r\n"); break;
            default: Message("KEY NOT FOUND \r\n");
            }
            Edge = 0;
            index = 0;
            score = 0;
        }

    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************