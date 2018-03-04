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

#include <Wire.h>

//////////////////////////////////////////////////////////////////////////////////////////
static unsigned long currentTime = 0;
static unsigned long previousTime = 0;
static unsigned long cycleTime = 0;
float dt = 0;
static unsigned long taskTime400 = 0;

//////////////////////////////////////////////////////////////////////////////////////////
byte X = 0;
byte Y = 1;
byte Z = 2;

byte ROLL = 0;
byte PITCH = 1;
byte YAW = 2;

byte MPU6050_ADDRESS = 0x68; //WHO_AM_I

float gyroRAW[3];
int gyro_scale = 131;
int gyro_offset[3] = {27.45, 1.84, -1.05};

float accRAW[3];
float acc_scale = 16384/256;
float acc_offset[3] = {18.77, -7.19, 222.71};

float gyroAngle[3], gyroLast[3];
float gyroVectorX = 0;
float gyroVectorY = 0;
float gyroVectorZ = 1;
float Gyro_AngX;
float Gyro_AngY;
float Gyro_AngZ;

float accAngle[2];
float x_val, y_val, z_val, acc_result;
unsigned long x2, y2, z2;
float n_accX;
float n_accY;
float n_accZ;

float trueAngle[3], ComplementAlpha = 0.5;
float trueVectorX = 0;
float trueVectorY = 0;
float trueVectorZ = 1;
float trueVectorR = 1;

//////////////////////////////////////////////////////////////////////////////////////////

float setpoint = 0;
float PID_CMD;
float error, errorI, errorD, previous_error = 0;
float Pterm, Iterm, Dterm;

float KP = 2.0f; //5
float KI = 3.0f;
float KD = 1.0f; //0.7

static int motorCMD[2];

int rc[2];

byte incomingByte;
byte startbyte = 0;
