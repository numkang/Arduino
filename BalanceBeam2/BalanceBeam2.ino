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
static uint32_t taskLED = 0, taskTime400 = 0, taskTime500 = 0;

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

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                Serial Setup                              //
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

#define ISC_KEEP_ALIVE 100
#define ISC_ANGLE 101
#define ISC_SETPOINT 102
#define ISC_GAIN 103
#define ISC_ON 104
#define ISC_OFF 105
#define ISC_START 106
#define ISC_STOP 107

byte isAlive = 0;
bool isConnect = false;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           IMU Constant & Variables                       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define MPU6050

#if defined(MPU6050)
  #define ACCELEROMETER
  #define GYROSCOPE
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -X; accRAW[PITCH]  = -Y; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
#endif

#if defined(MPU6050)
  #define MPU6050_ADDRESS 0x68  
  #define MPU6050_DLPF_CFG 2 //0-256 1-188 2-98 3-42 4-20 5-10 6-5
#endif

//I2C
uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
static int16_t i2c_errors_count = 0;

#if defined(ACCELEROMETER)
  #define ACC_TO_SAMPLE 400
  static uint16_t calibratingA = 400;
  static int16_t accRAW[3], accSmooth[3];
  static int16_t ACC_Zero[3];
#endif

#if defined(GYROSCOPE)
  #define GYRO_TO_SAMPLE 400
  static int16_t gyroRAW[3];
  static uint16_t calibratingG = 400;
  static int16_t GYRO_Zero[3];
  static int16_t gyroData[3] = {0,0,0};
#endif

static int16_t magRAW[3];

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                  Complementary Filter Constant & Variables               //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

struct function_struct {
  uint8_t gyroCalibrated :1 ;
  uint8_t accCalibrated :1 ;
  uint8_t motorCalibrated :1 ;
  uint8_t SMALL_ANGLES_25 :1 ; 
  uint8_t ARM :1 ;
} f;

static int16_t angle[3] = {0,0,0};
static float angle_f[2] = {0,0};
static int16_t heading;

#define ROLL 0
#define PITCH 1
#define YAW 2

#define I2C_SPEED 400000L
#define ACC_LPF_FACTOR 4 //100

#if defined(MMA7361)
  #define ACC_1G 64
#elif defined(ADXL345)
  #define ACC_1G 265
#elif defined(BMA180)
  #define ACC_1G 255
#elif defined(MPU6050)
  #define ACC_1G 512
#elif defined(LSM330)
  #define ACC_1G 256
#elif defined(LSM303DLHC)
  #define ACC_1G 265 //256
#else
  #define ACC_1G 256
#endif

#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)

#define GYR_CMPFA_FACTOR 600.0f //400
#define GYR_CMPFM_FACTOR 250.0f //200,300
#define INV_GYR_CMPFA_FACTOR (1.0f / (GYR_CMPFA_FACTOR + 1.0f))
#define INV_GYR_CMPFM_FACTOR (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

#if defined(ITG3200)
  #define GYRO_SCALE (4 / 14.375 * PI / 180.0 / 1000000.0) //ITG3200   14.375 LSB/(deg/s) and we ignore the last 2 bits
#elif defined(L3G4200D)
  #define GYRO_SCALE ((4.0f * PI * 70.0f)/(1000.0f * 180.0f * 1000000.0f))
#elif defined(MPU6050)
  #define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits
#elif defined(LSM330)
  #define GYRO_SCALE ((4.0f * PI * 70.0f)/(1000.0f * 180.0f * 1000000.0f)) // like L3G4200D
#elif defined(L3GD20)
  #define GYRO_SCALE ((4.0f * PI * 70.0f)/(1000.0f * 180.0f * 1000000.0f)) // like L3G4200D (not sure)
#elif defined(MPU3050)
  #define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits
#else
  #define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //GYRO_SCALE unit: radian/microsecond
#endif

static int16_t gyroADCprevious[3] = {0,0,0};
static int16_t gyroADCp[3];
static int16_t gyroADCinter[3];

static float invG;
static int32_t accLPF32[3] = {0, 0, 1};

typedef struct fp_vector {    
  float X,Y,Z;    
} t_fp_vector_def;

typedef union {   
  float A[3];   
  t_fp_vector_def V;    
} t_fp_vector;

typedef struct int32_t_vector {
  int32_t X,Y,Z;
} t_int32_t_vector_def;

typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} t_int32_t_vector;

static t_fp_vector EstG;
static t_int32_t_vector EstG32;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                       PWM & PID Constant & Variables                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define P 0
#define I 1
#define D 2
#define MINCOMMAND 900
#define MAXCOMMAND 2200
#define PID_MIX(X,Y,Z) 1200 + X*PID_CMD[ROLL] + Y*PID_CMD[PITCH] + Z*PID_CMD[YAW];

static uint16_t PWM_Command[9];

volatile int setpoint = 0;
float PID_CMD[3];
float error, errorI, errorD, previous_error = 0;
float Pterm, Iterm, Dterm;
float gyro_D;

float K[3] = {0,0,0};
/*float KP = 0;//5.0f; //5
float KI = 0;//5.0f;
float KD = 0;//3.0f; //0.7*/
