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
  YAW,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};

enum vec {
  X,Y,Z
};

struct function_struct {
  uint8_t gyroCalibrated :1 ;
  uint8_t accCalibrated :1 ;
  uint8_t magCalibrated :1 ;
  
  uint8_t motorCalibrated :1 ;
  
  uint8_t SMALL_ANGLES_25 :1 ;
  
  uint8_t DISARM :1 ;
  uint8_t ARM :1 ;
  uint8_t c;
} f;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 Time function                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint32_t cycleTime = 0;
static uint32_t taskTime50 = 0, taskTime400 = 0, taskTime500 = 0;
float dt = 0;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           IMU Constant & Variables                       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
#define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -X; accRAW[PITCH]  = -Y; accRAW[YAW]  =  Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magRAW[ROLL]  =  X; magRAW[PITCH]  =  Y; magRAW[YAW]  = -Z;}

//I2C
#define I2C_SPEED 400000L
uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
static int16_t i2c_errors_count = 0;

uint8_t gyroCalibrated = 0;
int16_t gyroHigh[3];
int16_t gyroLow[3];
float gyroLast[3];
float dPS[3];
float gyroAngle[3];

#define MPU6050_ADDRESS 0x68  
#define MPU6050_DLPF_CFG 2 //0-256 1-188 2-98 3-42 4-20 5-10 6-5
#define GYRO_TO_SAMPLE 400
static int16_t gyroRAW[3];
static uint16_t calibratingG = 400;
static int16_t GYRO_Zero[3];
static int16_t gyroData[3] = {0,0,0};


#define ACC_TO_SAMPLE 400
static uint16_t calibratingA = 400;
static int16_t accRAW[3], accSmooth[3];
static int16_t ACC_Zero[3], ACC_Zero2[3];

#define HMC5883_ADDRESS 0x1E
#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03
#define MAG_DECLINIATION 0.0f
static int16_t magSmooth[3];
static float magScale[3] = {1.0,1.0,1.0};
static uint8_t magInit = 0;
static int16_t MAG_Zero[3];
static int16_t magRAW[3];

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                  Complementary Filter Constant & Variables               //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
static int32_t angle[3] = {0,0,0};
static float heading;

#define ACC_LPF_FACTOR 4 //100
#define ACC_1G 512
#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)

#define GYR_CMPFA_FACTOR 0.0f //400
#define GYR_CMPFM_FACTOR 10.0f //200,300
#define INV_GYR_CMPFA_FACTOR (1.0f / (GYR_CMPFA_FACTOR + 1.0f))
#define INV_GYR_CMPFM_FACTOR (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

#define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)

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

static t_fp_vector EstM;
static t_int32_t_vector EstM32;

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

uint8_t dirPin_elev = 8; //LOW-CW
uint8_t stepperPin_elev = 9; 
uint8_t dirPin_azim = 11;
uint8_t stepperPin_azim = 10;

uint8_t step_c;
int16_t step_cc;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                  Vector                                  //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

//Magnetometer
  float mbx, mby, mbz, mcal[3];

  float m1 = 659.1603;
  float m2 = 2.0909;
  float m3 = -4.4645;
  float m4 = 2.0909;
  float m5 = 679.4839;
  float m6 = 1.2232;
  float m7 = -4.4645;
  float m8 = 1.2232;
  float m9 = 660.0949;

  float detm  = m1*m5*m9 - m1*m6*m8 - m2*m4*m9+m2*m6*m7 + m3*m4*m8 - m3*m5*m7;
  float invm1 = (m5*m9-m6*m8)  / detm;
  float invm2 = -(m2*m9-m3*m8) / detm;
  float invm3 = (m2*m6-m3*m5)  / detm;
  float invm4 = -(m4*m9-m6*m7) / detm;
  float invm5 = (m1*m9-m3*m7)  / detm;
  float invm6 = -(m1*m6-m3*m4) / detm;
  float invm7 = (m4*m8-m5*m7)  / detm;
  float invm8 = -(m1*m8-m2*m7) / detm;
  float invm9 = (m1*m5-m2*m4)  / detm;
  
//Accelerometer  
  float abx, aby,abz, acal[3];

  float a1 = 297.7445;
  float a2 = 1.2799;
  float a3 = -0.5950;
  float a4 = 1.2799;
  float a5 = 299.4024;
  float a6 = 0.2521;
  float a7 = -0.5950;
  float a8 = 0.2521;
  float a9 = 300.7052;

  float deta  = a1*a5*a9 - a1*a6*a8 - a2*a4*a9 + a2*a6*a7 + a3*a4*a8 - a3*a5*a7;
  float inva1 = (a5*a9-a6*a8)  / deta;
  float inva2 = -(a2*a9-a3*a8) / deta;
  float inva3 = (a2*a6-a3*a5)  / deta;
  float inva4 = -(a4*a9-a6*a7) / deta;
  float inva5 = (a1*a9-a3*a7)  / deta;
  float inva6 = -(a1*a6-a3*a4) / deta;
  float inva7 = (a4*a8-a5*a7)  / deta;
  float inva8 = -(a1*a8-a2*a7) / deta;
  float inva9 = (a1*a5-a2*a4)  / deta;
  
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                             Antenna Tracking                             //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

//for Test Math
/*float xx[] = {1,2,3};
float yy[] = {4,5,6};
float zz[3], vec[3][3], invvec[3][3], multi[3];
float xxx[] = {11,12,13};
float yyy[] = {14,15,16};
float zzz[3], vvec[3][3], multi3[3][3];*/

float m_project[3], acalxmp[3], m_level[3];
float acal_mp_acalxmp[3][3], acal_mp_acalxmp_inv[3][3];
float idealVec[3][3] = {{0,1,0},{0,0,-1},{-1,0,0}};
float idealVec_inv[3][3] = {{0,0,-1},{1,0,0},{0,-1,0}};
float TransformMartix[3][3];
float idealAcc[3] = {0,0,-1}, idealAccxmlevel[3];
float idealAcc_mlevel_idealAccxmlevel[3][3], idealAcc_mlevel_idealAccxmlevel_inv[3][3];
float acalxmcal[3], acal_mcal_acalxmcal[3][3];

float initAzim = 0.0;
float initElev = 45.0 * (-1.0);
float initLat = 0.0;
float initLon = 0.0;
float newAzim, newElev, xyz_init[3], xyz_new[3];
