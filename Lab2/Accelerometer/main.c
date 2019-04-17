/***************************************************
  This is a example sketch demonstrating graphic drawing
  capabilities of the SSD1351 library for the 1.5"
  and 1.27" 16-bit Color OLEDs with SSD1351 driver chip
  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1431
  ------> http://www.adafruit.com/products/1673
  If you're using a 1.27" OLED, change SCREEN_HEIGHT to 96 instead of 128.
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  The Adafruit GFX Graphics core library is also required
  https://github.com/adafruit/Adafruit-GFX-Library
  Be sure to install it!
 ****************************************************/

// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128 // Change this to 96 for 1.27" OLED.

// You can use any (4 or) 5 pins
#define SCLK_PIN 2
#define MOSI_PIN 3
#define DC_PIN   4
#define CS_PIN   5
#define RST_PIN  6

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
//#include <SPI.h>
#include <string.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
// Common interface includes
#include "uart_if.h"
#include "pinmux.h"
#include "gpio.h"
#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100
#include "i2c_if.h"



//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION     "1.1.1"
#define APP_NAME                "I2C Demo"
#define UART_PRINT              Report
#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************




// Option 1: use any pins but a little slower
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN, RST_PIN);

// Option 2: must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN);


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


void main()
{
    int iRetVal;
       char acCmdStore[512];

       //
       // Initialize board configurations
       //
       BoardInit();

       //
       // Configure the pinmux settings for the peripherals exercised
       //
       PinMuxConfig();

       //
       // Configuring UART
       //
       InitTerm();

       //
       // Clearing the Terminal.
       //
       ClearTerm();

       //
       // I2C Init


    //
    // Enable the SPI module clock
    //

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);


    MAP_PRCMPeripheralReset(PRCM_GSPI);


//Start of MasterMain()
    //unsigned long ulUserData;
    //unsigned long ulDummy;

    //
    // Initialize the message
    //
    //memcpy(g_ucTxBuff,MASTER_MSG,sizeof(MASTER_MSG));

    //
    // Set Tx buffer index
    //
   // ucTxBuffNdx = 0;
    //ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);
    // Reset the peripheral
    //
    //Set reset low to initialize chip
    //GPIOPinWrite(GPIOA3_BASE, 0x10, 0x00); //P18

    //Chip select set to high initially
    GPIOPinWrite(GPIOA2_BASE, 0x40, 0x40);

    Adafruit_Init();


    fillScreen(RED);
    int x = 63;
    int y = 63;
    fillCircle(x,y, 4, BLUE);
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);
    unsigned char ucDevAddr, ucRegOffset, ucRdLen;
      unsigned char aucRdDataBuf[6];


      //
      // Get the device address
      //

      ucDevAddr = 0x18;
      //
      // Get the register offset address
      //

      ucRegOffset = 0x2;

      //
      // Get the length of data to be read
      ucRdLen = 6;
      //RETERR_IF_TRUE(ucLen > sizeof(aucDataBuf));

      //
      // Write the register address to be read from.
      // Stop bit implicitly assumed to be 0.
      //
      char x_acc;
      char y_acc;
      int old_x = 0;
      int old_y = 0;
      int radius = 6;
      while(1){
          fillCircle(old_x, old_y, radius, RED);
      RET_IF_ERR(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0));

      //
      // Read the specified length of data
      //
      RET_IF_ERR(I2C_IF_Read(ucDevAddr, &aucRdDataBuf[0], ucRdLen));

      //initial position of ball
      x_acc = aucRdDataBuf[1];
      y_acc = aucRdDataBuf[3];
      //range of x_acc and y_acc is 0-255
      //8-bit two's complement
      //two's complement so the first bit is positive or negative
      fillCircle(y, x , radius, BLUE);
      old_x = y;
      old_y = x;
      if (x_acc > 127){
          x_acc = x_acc ^ 0xFF;
          x_acc = x_acc + 1;
          x_acc = -x_acc;
      }
      if (y_acc > 127){
          y_acc = y_acc ^ 0xFF;
          y_acc = y_acc + 1;
          y_acc = -y_acc;
      }
      //the circle should go from left to right in two seconds
      //We do not entirely know entire pipeline length
      //Assume updates 10 times a second or something
      //0-128 in 2 seconds
      //64 pixels per second
      //10 updates per second
      //at max speed 127/20
      //6.35 pixels per update
      int speed = 200;
      x = x - x_acc/speed;
      y = y + y_acc/speed;
      if (x < 3)
          x = 3;
      if (y < 3)
          y = 3;
      if (x > 124)
          x = 124;
      if (y > 124)
          y = 124;
      }
}
