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

#define APPLICATION_VERSION     "1.1.1"
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
    //p18 = RESET
    //p15 = OC
    //p21 = DC
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();
    //
    // Enable the SPI module clock
    //\

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
    fillScreen(CYAN);
   // delay(3000);
    testdrawrects(RED);
    //float p = 3.1415926;

      // You can optionally rotate the display by running the line below.
      // Note that a value of 0 means no rotation, 1 means 90 clockwise,
      // 2 means 180 degrees clockwise, and 3 means 270 degrees clockwise.
      //tft.setRotation(1);
      // NOTE: The test pattern at the start will NOT be rotated!  The code
      // for rendering the test pattern talks directly to the display and
      // ignores any rotation.
    /*
      fillRect(0, 0, 128, 128, BLUE);
      MAP_UtilsDelay(8000000);
      char c = 0;
      int x = 0;
      int y = 0;
      for (c; c < 128; c++){
          x++;
          y++;
          drawChar(x, y, c, RED, BLACK, 1);
      }*/
      /*
      lcdTestPattern();
      delay(500);

      tft.invert(true);
      delay(100);
      tft.invert(false);
      delay(100);

      tft.fillScreen(BLACK);
      testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", WHITE);
      delay(500);

      // tft print function!
      tftPrintTest();
      delay(500);

      //a single pixel
      tft.drawPixel(tft.width()/2, tft.height()/2, GREEN);
      delay(500);

      // line draw test
      testlines(YELLOW);
      delay(500);

      // optimized lines
      testfastlines(RED, BLUE);
      delay(500);


      testdrawrects(GREEN);
      delay(1000);

      testfillrects(YELLOW, MAGENTA);
      delay(1000);

      tft.fillScreen(BLACK);
      testfillcircles(10, BLUE);
      testdrawcircles(10, WHITE);
      delay(1000);

      testroundrects();
      delay(500);

      testtriangles();
      delay(500);

      Serial.println("done");
      delay(1000);
    */
    }

