
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
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
//#include <SPI.h>
#include <string.h>
#include "spi.h"
#include "uart.h"
// Common interface includes
#include "timer_if.h"
//#include "gpio_if.h"
#include "uart_if.h"
// Common interface includes
#include "uart_if.h"

#include "pinmux.h"
#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define APPLICATION_VERSION "1.1.1"
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE 0xFFFF
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
volatile int msgrcv;
volatile char rcv_arr[16];
volatile char tran_arr[16];
volatile int rcv_idx;
volatile int msg_length;
static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

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

//UARTA1_RX interrupt
static void  RecieverIntHandler(void){
    Message("RECIEVING MESSAGE\r\n");
    unsigned long ulStatus;
    msgrcv = 1;
    //int index = 0;
    msg_length = 0;
    long character;
    while(UARTCharsAvail(UARTA1_BASE)){
        character = UARTCharGet(UARTA1_BASE);
        rcv_arr[msg_length++] = (char) character;
    }
    ulStatus = MAP_UARTIntStatus (UARTA1_BASE, true);
    MAP_UARTIntClear(UARTA1_BASE, ulStatus);
}



//IR Reciever interrupt
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
    /*
#define UART_BAUD_RATE  115200
#define SYSCLK          80000000
#define CONSOLE         UARTA0_BASE
#define CONSOLE_PERIPH  PRCM_UARTA0
//*/
    InitTerm();
    MAP_UARTConfigSetExpClk(UARTA1_BASE,MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                     115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                      UART_CONFIG_PAR_NONE));



    ClearTerm();
    MAP_GPIOIntRegister(GPIOA1_BASE, BothEdgeIntHandler);
    MAP_GPIOIntTypeSet(GPIOA1_BASE, 0x10, GPIO_BOTH_EDGES);    // IR GPIO
    MAP_UARTIntRegister(UARTA1_BASE, RecieverIntHandler);

    //ulStatus = MAP_GPIOIntStatus (GPIOA2_BASE, false);
    //MAP_GPIOIntClear(GPIOA2_BASE, ulStatus);
    ulStatus = MAP_GPIOIntStatus (GPIOA1_BASE, false);
    MAP_GPIOIntClear(GPIOA1_BASE, ulStatus); // clear interrupts on GPIOA1
    //ulStatus = MAP_GPIOIntStatus (switch2.port, false);

   // MAP_GPIOIntClear(switch2.port, ulStatus);           // clear interrupts on GPIOA2
    //SPI Initialization
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIReset(GSPI_BASE);
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                         SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                         (SPI_SW_CTRL_CS |
                         SPI_4PIN_MODE |
                         SPI_TURBO_OFF |
                         SPI_CS_ACTIVEHIGH |
                         SPI_WL_8));

    MAP_SPIEnable(GSPI_BASE);
    // clear global variables
    SW2_intcount=0;
    SW3_intcount=0;
    SW2_intflag=0;
    SW3_intflag=0;
    Edge = 0; //flag of IR_int_handler
    //Code = 0;
    index = 0; //index of IR buffer
    score = 0; //32-bit interger value of buffer
    msgrcv = 0; //if we have recieved a msg
    rcv_idx = 0; //char index for the recieving character buffer
    msg_length = 0; //length of recieving message
    //Enable Interrupts
    MAP_GPIOIntEnable(GPIOA1_BASE, 0x10);
    MAP_UARTIntEnable(UARTA1_BASE, UART_INT_RX);


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

    GPIOPinWrite(GPIOA2_BASE, 0x40, 0x40);

    Adafruit_Init();
    fillScreen(RED);
    int rcvtxt = 0;
    long character;
    int current = -1;
    char cr = 'a';
    //int i = 0;
    while (1) {
        while (Edge == 0 && msgrcv == 0) {;}
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
            case 79370471: Message("0\r\n"); current = 0; break;
            case 79396991: Message("1\r\n"); current = 1; break;
            case 79380671: Message("2\r\n"); current = 2; break;
            case 79413311: Message("3\r\n"); current = 3; break;
            case 79405151: Message("4\r\n"); current = 4; break;
            case 79388831: Message("5\r\n"); current = 5; break;
            case 79421471: Message("6\r\n"); current = 6; break;
            case 79401071: Message("7\r\n"); current = 7; break;
            case 79384751: Message("8\r\n"); current = 8; break;
            case 79417391: Message("9\r\n"); current = 9; break;
            case 79373021: Message("MUTE\r\n"); current = 10; break;
            case 79401581: Message("LAST\r\n"); current = 11; break;
            default: Message("KEY NOT FOUND \r\n"); current = -1; break;
            }
            if (current = 11){
                Message("Send message\r\n");
            }
            Edge = 0;
            index = 0;
            score = 0;
        }
        if (msgrcv){
            int i;
            for (i = 0; i < msg_length; i++){
                drawChar(rcvtxt*4, 60, rcv_arr[i], GREEN, BLACK, 1);
                rcvtxt++;
            }
            msgrcv = 0;
        }

    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
