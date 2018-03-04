//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
//                                                                                      //
//                     Numkang's code for Microcontroller                               //
//                      SOFTWARE WRITTEN BY NUMKANG_AE20                                //
//          DEPARTMENT OF AEROSPACE ENGINEERING, KASETSART UNIVERSITY                   //
//                                                                                      //
//      This program is free software: you can redistribute it and/or modify            //
//      it under the terms of the GNU General Public License as published by            //
//      the Free Software Foundation, either version 3 of the License, or               //
//      any later version. see <http://www.gnu.org/licenses/>                           //
//                                                                                      //  
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 Time function                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint16_t cycleTime = 0;
static uint32_t taskTime50 = 0, taskTime4 = 0, taskTime2 = 0, taskTime16 = 0, taskTime500 = 0;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                              AVR Microchips Setup                        //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define UNO
  #define PROMINI
  #define NUMBER_MOTORS 4 //for QUAD
#endif

#if defined(__AVR_ATmega32U4__)
  #define NANO
  #define NUMBER_MOTORS 2 //for MAVion
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define MEGA
  #define NUMBER_MOTORS 6 //for QUAD or HEX
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           General function constants                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define THR 0

enum rc {
  ROLL,
  PITCH,
  YAW,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                        Interrupt Constant & Variables                    //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined (UNO) || defined (PROMINI)
  #define RC_CHANS 5
  #define PCINT_PIN_COUNT 5
  #define PCINT_RX_BITS (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
  #define RX_PCINT_PIN_PORT PIND
#endif

#if defined (NANO)
  #define RC_CHANS 5
  #define PCINT_PIN_COUNT 5
  #define PCINT_RX_BITS (1<<4),(1<<2),(1<<3),(1<<1)
  #define RX_PCINT_PIN_PORT PINB
#endif

#if defined (MEGA)
  #define RC_CHANS 8
  #define PCINT_PIN_COUNT 8
  #define PCINT_RX_BITS (1<<0),(1<<1),(1<<2),(1<<3),(1<<4),(1<<5),(1<<6),(1<<7)
  #define RX_PCINT_PIN_PORT PINK
#endif

static int16_t rcData[RC_CHANS]; //interval [1000;2000]
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};
volatile uint16_t rcValue[RC_CHANS];
volatile int16_t RC_Command[RC_CHANS];

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                              AVR Microchips Setup                        //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define UNO
  #define PROMINI
  #define UART_NUMBER 1
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define MEGA
  #define UART_NUMBER 4
#endif

#define RX_BUFFER_SIZE 256 //256 if GPS, 64 if not
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

static volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER],serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];

static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];
