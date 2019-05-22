//EEC 172 LAB 4
//Griffin Kimura
//Armand Nasseri


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
#include "inc/hw_apps_rcm.h"
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
#define TR_BUFF_SIZE     2

#define APPLICATION_VERSION "1.1.1"
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE 0xFFFF
#define M_PI 3.14159265358979323846
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);

volatile unsigned long SW2_intcount;
volatile unsigned long SW3_intcount;
volatile unsigned char SW2_intflag;
volatile unsigned char SW3_intflag;
volatile unsigned char Edge; // flag for GPIO interruput
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0; //increment counter for Key Press Delay
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts; //increment counter for IR reciever pulse width
volatile int buf[32]; //stores 32 bit decoder of IR
volatile int index; //index of the buffer
volatile int score; //32 bit int value of decoder (buf)
volatile int msgrcv; //flag for if we have recieved a printf
volatile char rcv_arr[16]; //store characters from UART1 RX
volatile char tran_arr[16]; //store characters to send to UART1 TX
volatile int rcv_idx; //index for rcv_arr
volatile int msg_length; //length of the sending printf
//static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

//Lab 4 Stuff
//unsigned char samples[10];
//unsigned char
unsigned long s1;
unsigned long s2;
volatile int count;

//Algorithm Globals
long int goertzel (volatile int sample[], long int coeff, int N);
char post_test (void);

//-------Global variables--------//

int N = 410;                 // block size
volatile int samples[410];   // buffer to store N samples
volatile int count;         // samples count
volatile int flag;         // flag set when the samples buffer is full with N samples
volatile int new_dig;      // flag set when inter-digit interval (pause) is detected

int power_all[8];       // array to store calculated power of 8 frequencies

//int coeff[8];           // array to store the calculated coefficients
int f_tone[8] = { 697, 770, 852, 941, 1209, 1336, 1477, 1633 }; // frequencies of rows & columns
int coeff[8] = { 31548,  31281, 30951, 30556, 29144,  28361,  27409,  26258};



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

//This will sample 16 kHz
//410 samples
//P3 is our CS
void
TimerBaseIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    //printf("HERE\n");
    //printf("HERE\n");
    if (count < 410){
    SPICSEnable(GSPI_BASE);
    //GPIOPinWrite(GPIOA2_BASE, 0x40, 0x00);
    GPIOPinWrite(GPIOA1_BASE, 0x10, 0x00);
    //might need to move pointer
    //SPIDataGet(GSPI_BASE, samples);
    SPITransfer(GSPI_BASE, 0,  g_ucRxBuff, 1, SPI_CS_ENABLE);
    //SPIDataGet(GSPI_BASE, &s2);
    SPITransfer(GSPI_BASE, 0, &(g_ucRxBuff[1]), 1, SPI_CS_ENABLE);
    //printf("%d  ", g_ucRxBuff[0]);
    //printf("%d\r\n",  g_ucRxBuff[1]);
    SPICSDisable(GSPI_BASE);
    GPIOPinWrite(GPIOA1_BASE, 0x10, 0x10);
    //GPIOPinWrite(GPIOA2_BASE, 0x40, 0x40);
    Timer_IF_InterruptClear(g_ulBase);
    //samples[count++] =  ((0x000003E0 & (((int) g_ucRxBuff[0]) << 5))  |  (0x0000001F & (((int)g_ucRxBuff[1]) >> 3)));
    samples[count++] = (((int)(g_ucRxBuff[0] & 0x1F) << 5) | (((int) g_ucRxBuff[1]) >> 3)) - 430 ;
    }
    else if (count == 410){
        //printf("Full Samples");
        //count = 0;
        flag = 1;
        Timer_IF_InterruptClear(g_ulBase);
        Timer_IF_Stop(g_ulBase, TIMER_A);
    }

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
    printf("RECIEVING printf\r\n");
    unsigned long ulStatus;
    msgrcv = 1;
    msg_length = 0;
    long character;
    while(UARTCharsAvail(UARTA1_BASE)){ //look for available characters in UART1
        character = UARTCharGet(UARTA1_BASE);
        rcv_arr[msg_length++] = (char) character; //add current character to buffer
    }
    int i = 0;
    for (i = 0; i < msg_length; i++){
        printf("%c", rcv_arr[i]);
    }
    ulStatus = MAP_UARTIntStatus (UARTA1_BASE, true);
    MAP_UARTIntClear(UARTA1_BASE, ulStatus);
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

long int
goertzel (volatile int sample[], long int coeff, int N)
//---------------------------------------------------------------//
{
//initialize variables to be used in the function
  int Q, Q_prev, Q_prev2, i;
  long prod1, prod2, prod3, power;

  Q_prev = 0;           //set delay element1 Q_prev as zero
  Q_prev2 = 0;          //set delay element2 Q_prev2 as zero
  power = 0;            //set power as zero

  for (i = 0; i < N; i++)   // loop N times and calculate Q, Q_prev, Q_prev2 at each iteration
    {
      Q = (sample[i]) + ((coeff * Q_prev) >> 14) - (Q_prev2);   // >>14 used as the coeff was used in Q15 format
      Q_prev2 = Q_prev;     // shuffle delay elements
      Q_prev = Q;
    }

  //calculate the three products used to calculate power
  prod1 = ((long) Q_prev * Q_prev);
  prod2 = ((long) Q_prev2 * Q_prev2);
  prod3 = ((long) Q_prev * coeff) >> 14;
  prod3 = (prod3 * Q_prev2);

  power = ((prod1 + prod2 - prod3)) >> 8;   //calculate power using the three products and scale the result down

  return power;
}

char
post_test (void)
//---------------------------------------------------------------//
{
//initialize variables to be used in the function
  int i, row, col, max_power;

  char row_col[4][4] =      // array with the order of the digits in the DTMF system
  {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
  };

// find the maximum power in the row frequencies and the row number

  max_power = 0;                //initialize max_power=0

  for (i = 0; i < 4; i++)       //loop 4 times from 0>3 (the indecies of the rows)
    {
      if (power_all[i] > max_power) //if power of the current row frequency > max_power
    {
      max_power = power_all[i]; //set max_power as the current row frequency
      row = i;      //update row number
    }
    }


// find the maximum power in the column frequencies and the column number

  max_power = 0;        //initialize max_power=0

  for (i = 4; i < 8; i++)   //loop 4 times from 4>7 (the indecies of the columns)
    {
      if (power_all[i] > max_power) //if power of the current column frequency > max_power
    {
      max_power = power_all[i]; //set max_power as the current column frequency
      col = i;      //update column number
    }
    }


  if (power_all[col] == 0 && power_all[row] == 0)   //if the maximum powers equal zero > this means no signal or inter-digit pause
    new_dig = 1;        //set new_dig to 1 to display the next decoded digit


  if ((power_all[col] > 200000 && power_all[row] > 200000))   // check if maximum powers of row & column exceed certain threshold AND new_dig flag is set to 1
    {
      //if (new_dig == 1)
      //write_lcd (1, row_col[row][col - 4]); // display the digit on the LCD
      //dis_7seg (8, row_col[row][col - 4]);  // display the digit on 7-seg
      return row_col[row][col - 4];
      //new_dig = 0;      // set new_dig to 0 to avoid displaying the same digit again.
    }
  else
      return 'Q';
}










int main() {
    unsigned long ulStatus;

    BoardInit();

    PinMuxConfig();
    /*
#define UART_BAUD_RATE  115200
#define SYSCLK          80000000
#define CONSOLE         UARTA0_BASE
#define CONSOLE_PERIPH  PRCM_UARTA0
*/
    InitTerm();
    MAP_UARTConfigSetExpClk(UARTA1_BASE,MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                     115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                      UART_CONFIG_PAR_NONE));



    ClearTerm();
    //MAP_GPIOIntRegister(GPIOA1_BASE, BothEdgeIntHandler);
    MAP_GPIOIntTypeSet(GPIOA1_BASE, 0x10, GPIO_BOTH_EDGES);    // IR GPIO
    MAP_UARTIntRegister(UARTA1_BASE, RecieverIntHandler); //UARTA1 RX int handler
    ulStatus = MAP_UARTIntStatus (UARTA1_BASE, false);
    MAP_UARTIntClear(UARTA1_BASE, ulStatus); //clear ints on UARTA1
    ulStatus = MAP_GPIOIntStatus (GPIOA1_BASE, false);
    MAP_GPIOIntClear(GPIOA1_BASE, ulStatus); // clear interrupts on GPIOA1

    //Enable SPI port for OLED
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIReset(GSPI_BASE);
    //SPI_IF_BIT_RATE = 400000
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                         400000,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                         (SPI_SW_CTRL_CS |
                         SPI_4PIN_MODE |
                         SPI_TURBO_OFF |
                         SPI_CS_ACTIVEHIGH |
                         SPI_WL_8));

    MAP_SPIEnable(GSPI_BASE);


    Adafruit_Init();
    fillScreen(RED);
    // clear global variables
    SW2_intcount=0;
    SW3_intcount=0;
    SW2_intflag=0;
    SW3_intflag=0;
    count = 0;
    Edge = 0; //flag of IR_int_handler
    index = 0; //index of IR buffer
    score = 0; //32-bit interger value of buffer
    msgrcv = 0; //if we have recieved a msg
    rcv_idx = 0; //char index for the recieving character buffer
    msg_length = 0; //length of recieving printf

   //Enable Interrupts
    MAP_GPIOIntEnable(GPIOA1_BASE, 0x10);
    MAP_UARTIntEnable(UARTA1_BASE, UART_INT_RX); //set interrupt for RX

    UARTFIFOEnable(UARTA1_BASE);
    UARTFIFOLevelSet(UARTA1_BASE,UART_FIFO_TX6_8,  UART_FIFO_RX6_8); //How many characters until interrupt triggered
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
    //Timer_IF_Start(g_ulBase, TIMER_A, 5000); //Sample Rate of 16 kHz
    //Timer_IF_Start(g_ulRefBase, TIMER_A, 1000);
    MAP_TimerLoadSet(g_ulBase, TIMER_A, 5000);
    //MAP_TimerEnable(g_ulBase, TIMER_A);
    MAP_TimerLoadSet(g_ulRefBase, TIMER_A, 1000);
    MAP_TimerEnable(g_ulRefBase, TIMER_A);
        //
        // Enable the GPT
        //
    //MAP_TimerEnable(ulBase,ulTimer);
    //GPIOPinWrite(GPIOA2_BASE, 0x40, 0x40); // CS for OLED
    //GPIOPinWrite(GPIOA1_BASE, 0x10, 0x10); // Complement of CS for ADC


    //fillScreen(BLUE);
    MAP_UARTIntEnable(UARTA1_BASE, UART_INT_RX);
    int rcvtxt = 0;
    long character;
    int key_press = -1; //which key is pressed
    int old_key = -1;
    int replace = 0; //flag if we were working with duplicate keypresses
    int size = 0; //where we are on transmit array //same as size
    char current = 0; //which initial character was pressed (key_press)
    int dup = 0; //Number of times we have cycled through the digit
    int i = 0;
    MAP_TimerEnable(g_ulBase, TIMER_A);
    while (1) {
        while (msgrcv == 0 && flag == 0) {;}
        if (flag){
            for (i = 0; i < 8; i++){
              power_all[i] = goertzel (samples, coeff[i], N);   // call goertzel to calculate the power at each frequency and store it in the power_all array
              printf("%d  ", power_all[i]);
            }
        char status = post_test ();
        /*
        if (status != 'Q'){
            printf("%c\r\n", status);
        }
        else{
            printf("Nope\r\n");
        }*/
        switch (status){ //score is the 32 bit value for decoder
            case '0': printf("0\r\n"); current = ' '; key_press = 0; break;
            //case 79396991: printf("1\r\n"); current = ; break;
            case '2': printf("2\r\n"); current = 'a'; key_press = 2; break;
            case '3': printf("3\r\n"); current = 'd'; key_press = 3; break;
            case '4': printf("4\r\n"); current = 'g'; key_press = 4; break;
            case '5': printf("5\r\n"); current = 'j'; key_press = 5; break;
            case '6': printf("6\r\n"); current = 'm'; key_press = 6; break;
            case '7': printf("7\r\n"); current = 'p'; key_press = 7; break;
            case '8': printf("8\r\n"); current = 't'; key_press = 8; break;
            case '9': printf("9\r\n"); current = 'w'; key_press = 9; break;
            case '*': printf("* (ENTER)\r\n"); current = 0; key_press = 10; break; //ENTER
            case '#': printf("# (DELETE)\r\n"); current = 0; key_press = 11; break; //DELETE
            default: printf("KEY NOT FOUND \r\n"); current = -1; key_press = -1; break;
            }
            if (key_press > -1) // valid key press
            {

            if (key_press == 10 && size){ //SEND
                printf("size: %d\r\n", size);
                //send printf (index size)
                //delete old printf on screen
                int j = 0;

                for (j = 0; j < size; j++){
                    printf("j: %d\r\n", j);
                    printf("tran_arr[%d]: %c\r\n",j, tran_arr[j]);
                    MAP_UARTCharPut(UARTA1_BASE, tran_arr[j]);
                }
                for (j = size - 1; j <= 10; j++){
                    MAP_UARTCharPut(UARTA1_BASE, ' ');
                }
                drawRect(0, 0, 128, 16, BLACK);
                fillRect(0, 0, 128, 16, BLACK);
                //set cursor back to starting position
                size = 0;
            }
            else if (key_press == 0){ //SPACE
                //add a space to the char array
                //Probably just put a rectangle
                //drawRect(10*size, 0, 10, 20, BLACK);
                tran_arr[size++] = ' ';
                drawChar(size*10, 0, ' ', GREEN, BLACK, 2);
                //move index over one position
            }
            else if (key_press == 11 && size){ //BACKSPACE/DELETE
                //If there are characters to delete
                //delete element from char array
                size--;
                drawRect(10*size, 0, 128, 16, BLACK);
                fillRect(10*size, 0, 128, 16, BLACK);
                //drawRect(10*size, 0, 10, 20, BLACK);
                //move index to the left
            }
            else if (g_ulRefTimerInts < 60000 && (key_press == old_key) && size){ //If duplicate
                //need to change letter being displayed
                //alter in display
                //alter in character array
                //no change in size

                current = tran_arr[(size - 1)];
                if (key_press == 7 || key_press == 9){ // 4 options
                    if (dup == 3){ //wrap around
                        current = current - 3;
                        dup = 0;
                    }
                    else{
                        current++;
                        dup++;
                    }
                }
                else{ // 3 options
                    if (dup == 2){ //wrap around
                        current = current - 2;
                        dup = 0;
                    }
                    else{
                        current++;
                        dup++;
                    }
                }
                //insert
                drawChar((size-1)*10, 0, current, GREEN, BLACK, 2);
                tran_arr[(size - 1)] = current;
                replace = 1;
            }
            else if (key_press != 10 && key_press != 11){ //Regular operation
                if (g_ulRefTimerInts >= 60000 && replace){
                    drawChar((size)*10, 0, current, GREEN, BLACK, 2);
                    replace = 0;

                }
                else{
                    drawChar(size*10, 0, current, GREEN, BLACK, 2);
                }
                //add character to array
                //add character to display
                //Move cursor to next line
                //We are using 16 characters
                //So one line would be 128/16 = 8 pixels per character
                //Could do two lines with 16 pixels, we will see
                tran_arr[size++] = current;
                old_key = key_press;
                dup = 0;
            }

            g_ulRefTimerInts = 0;
            }
            //One key press read multiple times
            MAP_UtilsDelay(1000000);
            flag = 0;
            count = 0;
           //Timer_IF_Start(g_ulBase, TIMER_A, 1);
            //MAP_TimerLoadSet(g_ulBase, TIMER_A, 5000);
            MAP_TimerEnable(g_ulBase, TIMER_A);

        }
        if (msgrcv){ // If we have recieved a printf, draw from rcv_arr[i]
            drawRect(0, 60, 128, 16, BLACK);
            fillRect(0, 60, 128, 16, BLACK);
            int i;
            for (i = 0; i < msg_length; i++){
                drawChar(i*10, 60, rcv_arr[i], GREEN, BLACK, 2);
                rcvtxt++;
            }
            msgrcv = 0;
            msg_length = 0;
        }

    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
