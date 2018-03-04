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

enum rc {
  ROLL,
  PITCH,
  AUX1,
  AUX2,
};

enum vec {
  X,Y,Z
};

struct function_struct {  
  uint8_t DISARM :1 ;
  uint8_t ARM :1 ;
  uint8_t c;
} f;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 Time function                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

static uint32_t currentTime = 0, cTime = 0;
static uint32_t previousTime = 0, pTime = 0;
static uint32_t cycleTime = 0, sTime = 0;
static uint32_t taskTime50 = 0, taskTime400 = 0, taskTime500 = 0;
float dt = 0;
uint8_t tc = 0;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                       Serial UART Constant & Variables                   //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

enum _COMMAND{
  ISC_KEEP_ALIVE = 100,
  ISC_RC_PWM_IN,
  ISC_RC_PWM_OUT,
  ISC_IMG_DIST
};

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
//                        Interrupt Constant & Variables                    //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define RC_CHANS 5
#define PCINT_PIN_COUNT 5
#define PCINT_RX_BITS (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
#define RX_PCINT_PIN_PORT PIND

static int16_t rcData[RC_CHANS]; //interval [1000;2000]
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};
volatile uint16_t rcValue[RC_CHANS];
volatile int16_t RC_Command[RC_CHANS];

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                       PWM & PID Constant & Variables                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define MINCOMMAND 950
#define MAXCOMMAND 1900
#define PID_MIX(X,Y,Z) RC_Command[THR] + X*PID_CMD[ROLL] + Y*PID_CMD[PITCH] + Z*PID_CMD[YAW];
static uint16_t PWM_Command[2];
