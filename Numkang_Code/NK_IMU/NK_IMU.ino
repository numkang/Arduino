#include <Wire.h>

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

enum MotorServo {
  M1,
  M2,
  M3,
  M4,
  M5,
  M6,
  M7,
  M8
};

unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long cycleTime = 0;
unsigned long tasktime = 0;
unsigned long tasktime50 = 0;
float dt = 0;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                          Sensor Constant & Variables                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

static uint8_t rawADC[6];

#define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] = X; gyroRAW[PITCH] = Y; gyroRAW[YAW] = Z;}
#define L3G4200D_Address 0x69
uint8_t gyroCalibrated = 0;
int gyroRAW[3];
float gyroLast[3];
float dPS[3];
float gyroAngle[3];
int gyroHigh[3];
int gyroLow[3];

#define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL] = X; accRAW[PITCH] = Y; accRAW[YAW] = Z;}
#define ADXL345_ADDRESS 0x53
#define ACC_TO_SAMPLE 400
#define ACC_1G 265
uint8_t accCalibrated = 0;
static int16_t ACC_SAMPLE = ACC_TO_SAMPLE;
static int16_t acc_zero_offset[3];
int accRAW[3];

float gyroVectorX = 0;
float gyroVectorY = 0;
float gyroVectorZ = 1;
float Gyro_AngX;
float Gyro_AngY;
float Gyro_AngZ;

float accR;
float inv_accR;
float n_accX;
float n_accY;
float n_accZ;

float trueVectorX = 0;
float trueVectorY = 0;
float trueVectorZ = 1;
float trueVectorR = 1;

float ComplementAlpha = 0.9;

float trueAng[2];

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
//                            PWM Constant & Variables                      //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define MINCOMMAND 950
#define MAXCOMMAND 1900

static unsigned int PWM_Command[9];
