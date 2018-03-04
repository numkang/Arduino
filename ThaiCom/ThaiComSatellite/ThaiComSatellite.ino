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

#define MEGA

#define X 0
#define Y 1
#define Z 2

#define VER1 1
#define VER2 0
#define VER3 0

static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint16_t cycleTime = 0;
static uint32_t taskTime50 = 0, taskTime4 = 0, taskTime2 = 0, taskTime16 = 0, taskTime500 = 0;
float dt = 0;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                Step Motor                                //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

uint8_t dirPin_azim = 4; 
uint8_t dirPin_elev = 5;
uint8_t stepperPin_azim = 2; 
uint8_t stepperPin_elev = 3;

int16_t potenPin_azim = A0;
int16_t potenPin_elev = A15;
float potenVal_azim; // Left +, Right -
float potenVal_elev; // Down +, Up -
float poten_azim_min = 266;
float poten_azim_max = 960;
float poten_elev_min = 176;
float poten_elev_max = 798;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                            Analog Devices IMU                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
/*
Hardware Setup
--------------

      ADIS 16405    Arduino Mega    Comment
      ----------    ------------    -------
        SCLK           Pin 52
        MOSI / DIN     Pin 51
        MISO / DOUT    Pin 50
        Reset (RST)    Pin 6
        SS / CS        Pin 7
        5V             5V           common 5V power line
        GND            GND          common Ground
*/

#include <SPI.h>
/// IMU Data Structure
#define g 9.814 // [m/s^2]
#define	R2D 57.295779513082323 // radians to degrees 
#define D2R 0.017453292519943  // degrees to radians

struct imu_adis16405 {
  // All variables are stored in Output Data Register Format
  //  See Table 9 or data sheet for more details.  
  //
  // Previously, the data was converted into SI units.  However,
  // memory became a problem and the arduino was hanging.  Hence,
  // it was rolled back to use the unscalled outputs.
    int p;       ///< [0.05 deg/s], body X axis angular rate (roll)
    int q;       ///< [0.05 deg/s], body Y axis angular rate (pitch)
    int r;       ///< [0.05 deg/s], body Z axis angular rate (yaw)
    int ax;      ///< [3.33 mg], body X axis acceleration
    int ay;      ///< [3.33 mg], body Y axis acceleration
    int az;      ///< [3.33 mg], body Z axis acceleration
    int mx;      ///< [0.5 mgauss], body X axis magnetic field
    int my;      ///< [0.5 mgauss], body Y axis magnetic field
    int mz;      ///< [0.5 mgauss], body Z axis magnetic field
    int  T;      ///< [0.14 deg C, wrt 25 deg C], temperature of IMU sensor
    int  Vs;     ///< [2.418 mV], supply voltage of IMU sensor
    int adc;     ///< [806 microV], ADC reading
    bool VALID_DATA; // set flag as part of parsing
} ;

// ADIS16405 SPI Pins
const int reset_pin = 6;
const int chipSelectPin = 7;

imu_adis16405 isense_data;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                  Vector                                  //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

//Magnetometer
  float mbx, mby, mbz, mcal[3];

  /*float m1 = 67.3816;
  float m2 = 0.0573;
  float m3 = 0.0476;
  float m4 = 0.0573;
  float m5 = 66.8358;
  float m6 = -0.0290;
  float m7 = 0.0476;
  float m8 = -0.0290;
  float m9 = 67.1046;*/
  
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
  
  float heading, headingDegrees;

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

//Gyroscope
  uint8_t gyroCalibrated = 0;
  int gyroRAW[3];
  float gyroLast[3];
  float dPS[3];
  float gyroAngle[3];
  int gyroHigh[3];
  int gyroLow[3];

  float gyroVectorX = 0;
  float gyroVectorY = 0;
  float gyroVectorZ = -1;
  float Gyro_AngX;
  float Gyro_AngY;
  float Gyro_AngZ;

float trueVectorX = 0;
float trueVectorY = 0;
float trueVectorZ = -1;
float trueVectorR = 1;

float ComplementAlpha[3] = {0.0, 0.0, 0.0};

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
float newAzim, newElev, xyz_init[3], xyz_new[3];

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 WiiNunchunk                              //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
static uint8_t nunchuck_buf[6];

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                       Serial UART Constant & Variables                   //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define ISP_BUFFERING            100
#define ISP_KEEP_ALIVE           101
#define ISP_RAW_IMU              102
#define ISP_ATTITUDE             103
#define ISP_ATTITUDE_IMU         104
#define ISP_POTEN_PIN            105
#define ISP_GET_POTENMAP         106
#define ISP_SET_POTENMAP         107
#define ISP_CPU_CYCLE            108

#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

static volatile uint8_t serialHeadRX,serialTailRX;
static uint8_t serialBufferRX[RX_BUFFER_SIZE];
static volatile uint8_t serialHeadTX,serialTailTX;
static uint8_t serialBufferTX[TX_BUFFER_SIZE];
static uint8_t inBuf[INBUF_SIZE];
static uint8_t checksum;
static uint8_t indRX;
static uint8_t cmdMSP;

#define DATA_BUFF_LEN  16

static volatile uint8_t serialHeadRX1,serialTailRX1;
static volatile uint8_t serialHeadTX1,serialTailTX1;
static uint8_t serialBufferRX1[DATA_BUFF_LEN];
static uint8_t serialBufferTX1[DATA_BUFF_LEN];
static uint8_t inBuf1[DATA_BUFF_LEN];
static uint8_t checksum1;
static uint8_t indRX1;
static uint8_t cmdMSP1;
