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

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define UNO
  #define PROMINI
  #define NUMBER_MOTORS 2
  #define UART_NUMBER 1
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 Time function                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint32_t cycleTime = 0;
static uint32_t taskTime = 0;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                       Serial UART Constant & Variables                   //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

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
static uint8_t cmdISC[UART_NUMBER];

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                Step Motor                                //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define dirPin 2
#define stepPin 9
#define potenPin A0
float potenMin = 317;
float potenMax = 999;
int16_t potenVal;
float angle;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                Step Motor                                //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

int16_t input = 100; //100Hz
int16_t output, p_output;
float error, p_error, pp_error;

