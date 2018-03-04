//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           calculation function                           //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#if !defined (CHR_UM6)
#define ssin(val) (val)
#define scos(val) 1.0f
#define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)

int16_t _atan2(int32_t y, int32_t x){
  float z = (float)y / x;
  int16_t a;
  if ( abs(y) < abs(x) ){
     a = 573 * z / (1.0f + 0.28f * z * z);
   if (x<0) {
     if (y<0) a -= 1800;
     else a += 1800;
   }
  } else {
   a = 900 - 573 * z / (z * z + 0.28f);
   if (y<0) a -= 1800;
  }
  return a;
}

float InvSqrt (float x){ 
  union{  
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X; 
}
#endif
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                    complementary filter initialize                       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#if !defined (CHR_UM6)
void computeIMU(){  
  uint8_t axis;
  
  #if defined(ACCELEROMETER)
    acc_getdata();
    getEstimatedAttitude();
  #endif
  
  #if defined(GYROSCOPE)
    gyro_getdata();
    
    for(axis=0; axis<3; axis++) gyroADCp[axis] = gyroRAW[axis];
    
    gyro_getdata();
    
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] = gyroRAW[axis]+gyroADCp[axis];
      gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
      gyroADCprevious[axis] = gyroADCinter[axis]>>1;
    }
  #endif
  
  /*#if defined(GYROSCOPE)
    gyro_getdata();
    
    for(axis = 0; axis < 3; axis++){
      gyroADCp[axis] = gyroADCinter[axis]; //RAW old
      gyroADCinter[axis] = gyroRAW[axis]; //RAW new (the next loop from RAW old)
      gyroData[axis] = (gyroADCinter[axis] + gyroADCp[axis] + gyroADCprevious[axis]*2)>>2;
      gyroADCprevious[axis] = (gyroADCinter[axis] + gyroADCp[axis])>>1; //mean between RAW new and RAW old
    }
  #endif*/
}
#endif
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//             accelerometer gyroscope and magnetometer filter              //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#if !defined (CHR_UM6)
void getEstimatedAttitude(){
  uint8_t axis;
  int32_t accMag = 0;
  float scale, deltaGyroAngle[3];
  uint8_t validAcc;
  static uint16_t previousT;
  uint16_t currentT = micros();

  scale = (currentT - previousT) * GYRO_SCALE; // GYRO_SCALE unit: radian/microsecond
  previousT = currentT;

  for (axis = 0; axis < 3; axis++){
    deltaGyroAngle[axis] = gyroRAW[axis] * scale;
    
    accLPF32[axis]  -= accLPF32[axis]>>ACC_LPF_FACTOR;
    accLPF32[axis]  += accRAW[axis];
    accSmooth[axis] = accLPF32[axis]>>ACC_LPF_FACTOR;
    
    accMag += (int32_t)accSmooth[axis]*accSmooth[axis] ;
  }
  
  accMag = accMag*100/((int32_t)ACC_1G*ACC_1G);

  rotateV(&EstG.V,deltaGyroAngle);
  
  #if defined(MAGNETOMETER) && defined(useMAG)
    rotateV(&EstM.V,deltaGyroAngle);
  #endif

  validAcc = 72 < (uint16_t)accMag && (uint16_t)accMag < 133;

  for (axis = 0; axis < 3; axis++) {
    if ( validAcc )
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPFA_FACTOR + accSmooth[axis]) * INV_GYR_CMPFA_FACTOR;
    EstG32.A[axis] = EstG.A[axis]; //int32_t cross calculation is a little bit faster than float	
    #if defined(MAGNETOMETER) && defined(useMAG)
      EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + magRAW[axis]) * INV_GYR_CMPFM_FACTOR;
      EstM32.A[axis] = EstM.A[axis];
    #endif
  }
  
  if ((int16_t)EstG32.A[2] > ACCZ_25deg)
    f.SMALL_ANGLES_25 = 1;
  else
    f.SMALL_ANGLES_25 = 0;

  // Attitude of the estimated vector
  int32_t sqGX_sqGZ = sq(EstG32.V.X) + sq(EstG32.V.Z);
  invG = InvSqrt(sqGX_sqGZ + sq(EstG32.V.Y));
  angle[ROLL]  = _atan2(EstG32.V.X , EstG32.V.Z);
  angle[PITCH] = _atan2(EstG32.V.Y , InvSqrt(sqGX_sqGZ)*sqGX_sqGZ);
  
  #if defined(MAGNETOMETER) && defined(useMAG)
    if(currentTime > taskTime16){ //16Hz task
      taskTime16 = currentTime + 62500;
      heading = _atan2(
      EstM32.V.Z * EstG32.V.X - EstM32.V.X * EstG32.V.Z,
      (EstM.V.Y * sqGX_sqGZ  - (EstM32.V.X * EstG32.V.X + EstM32.V.Z * EstG32.V.Z) * EstG.V.Y)*invG ); 
      heading += MAG_DECLINIATION; // Set from GUI
      heading /= 10;
    }
  #endif
  
  #if defined(THROTTLE_ANGLE_CORRECTION)
    cosZ = EstG.V.Z / ACC_1G * 100.0f;                                                         // cos(angleZ) * 100 
    throttleAngleCorrection = THROTTLE_ANGLE_CORRECTION * constrain(100 - cosZ, 0, 100) >> 3;  // 16 bit ok: 200*150 = 30000  
  #endif
}
#endif
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                              barometer filter                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#if !defined (CHR_UM6)
#if defined(BAROMETER) && defined(useBARO)

#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }

uint8_t getEstimatedAltitude(){
  static uint32_t deadLine;
  static float baroGroundTemperatureScale,logBaroGroundPressureSum;
  static float vel = 0.0f;
  static uint16_t previousT;
  uint16_t currentT = micros();
  uint16_t dTime;

  dTime = currentT - previousT;
  if (dTime < UPDATE_INTERVAL) return 0;
  previousT = currentT;

  if(calibratingB > 0) {
    logBaroGroundPressureSum = log(baroPressureSum);
    baroGroundTemperatureScale = (baroTemperature + 27315) * 29.271267f;
    calibratingB--;
  }

  BaroAlt = ( logBaroGroundPressureSum - log(baroPressureSum) ) * baroGroundTemperatureScale; // in cemtimeter 

  EstAlt = (EstAlt * 6 + BaroAlt * 2) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)
  
    //P
    int16_t error16 = constrain(AltHold - EstAlt, -300, 300);
    applyDeadband(error16, 10); //remove small P parametr to reduce noise near zero position
    BaroPID = constrain((AltKp * error16 >>7), -150, +150);

    //I
    errorAltitudeI += AltKi * error16 >>6;
    errorAltitudeI = constrain(errorAltitudeI,-30000,30000);
    BaroPID += errorAltitudeI>>9; //I in range +/-60
 
    // projection of ACC vector to global Z, with 1G subtructed
    // Math: accZ = A * G / |G| - 1G
    int16_t accZ = (accSmooth[ROLL] * EstG32.V.X + accSmooth[PITCH] * EstG32.V.Y + accSmooth[YAW] * EstG32.V.Z) * invG;

    static int16_t accZoffset = 0;
    if (!f.ARM) {
      accZoffset -= accZoffset>>3;
      accZoffset += accZ;
    }  
    accZ -= accZoffset>>3;
    applyDeadband(accZ, ACC_Z_DEADBAND);

    static int32_t lastBaroAlt;
    int16_t baroVel = (EstAlt - lastBaroAlt) * (1000000 / UPDATE_INTERVAL);
    lastBaroAlt = EstAlt;

    baroVel = constrain(baroVel, -300, 300); // constrain baro velocity +/- 300cm/s
    applyDeadband(baroVel, 10); // to reduce noise near zero

    // Integrator - velocity, cm/sec
    vel += accZ * ACC_VelScale * dTime;

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * 0.985f + baroVel * 0.015f;

    //D
    vario = vel;
    applyDeadband(vario, 5);
    BaroPID -= constrain(AltKd * vario >>4, -150, 150);
  
  return 1;
}
#endif
#endif
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           remove offset function                         //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#if !defined (CHR_UM6)
#if defined(ACCELEROMETER)
void acc_remove_offset(){
  static int32_t acc_sample[3]; 
  uint8_t axis;
  if (calibratingA>0) {
    for (axis = 0; axis < 3; axis++) {
      if (calibratingA == ACC_TO_SAMPLE) acc_sample[axis]=0;    
      acc_sample[axis] += accRAW[axis];      
      accRAW[axis]=0;
      ACC_Zero[axis]=0;
    }
    if (calibratingA == 1) {
      ACC_Zero[ROLL]  = acc_sample[ROLL]/ACC_TO_SAMPLE;
      ACC_Zero[PITCH] = acc_sample[PITCH]/ACC_TO_SAMPLE;
      ACC_Zero[YAW]   = acc_sample[YAW]/ACC_TO_SAMPLE - ACC_1G;
      writeAccConfig();
      f.accCalibrated = 1;      
    }
    calibratingA--;
  }
  for (axis = 0; axis < 3; axis++) {
  accRAW[axis] -= ACC_Zero[axis];
  }
}
#endif

#if defined(GYROSCOPE)
void gyro_remove_offset() {
  static int16_t lastGyro[3] = {0,0,0};
  static int32_t gyro_sample[3];
  uint8_t axis;

  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      if (calibratingG == GYRO_TO_SAMPLE) gyro_sample[axis]=0;
      gyro_sample[axis] += gyroRAW[axis];
      gyroRAW[axis]=0;
      GYRO_Zero[axis]=0;
      if (calibratingG == 1) {
        GYRO_Zero[axis] = gyro_sample[axis]/GYRO_TO_SAMPLE;
        f.gyroCalibrated = 1;    
      }
    }
    calibratingG--;
  }

  for (axis = 0; axis < 3; axis++) {
    gyroRAW[axis] -= GYRO_Zero[axis];
    gyroRAW[axis] = constrain(gyroRAW[axis],lastGyro[axis]-800,lastGyro[axis]+800);  
    lastGyro[axis] = gyroRAW[axis];
  }
}
#endif

#if defined(MAGNETOMETER) && defined (useMAG)
  #if !defined(MPU6050_I2C_AUX_MASTER)
    void Device_Mag_getADC(){
      mag_data();
    }
  #endif

  uint8_t mag_getdata() {
    static uint32_t t,tCal = 0;
    static int16_t magZeroTempMin[3];
    static int16_t magZeroTempMax[3];
    uint8_t axis;
    if ( currentTime < t ) return 0; //each read is spaced by 100ms
    t = currentTime + 100000;
    
    Device_Mag_getADC();
    
    magRAW[ROLL]  = magRAW[ROLL]  * magScale[ROLL];
    magRAW[PITCH] = magRAW[PITCH] * magScale[PITCH];
    magRAW[YAW]   = magRAW[YAW]   * magScale[YAW];
    
    if (f.magCalibrated) {
      tCal = t;
      for(axis=0;axis<3;axis++) {
        MAG_Zero[axis] = 0;
        magZeroTempMin[axis] = magRAW[axis];
        magZeroTempMax[axis] = magRAW[axis];
      }
      f.magCalibrated = 0;
    }
    if (magInit) { // we apply offset only once mag calibration is done
      magRAW[ROLL]  -= MAG_Zero[ROLL];
      magRAW[PITCH] -= MAG_Zero[PITCH];
      magRAW[YAW]   -= MAG_Zero[YAW];
    }
 
    if (tCal != 0) {
      if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
        for(axis=0; axis<3; axis++) {
          if (magRAW[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magRAW[axis];
          if (magRAW[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magRAW[axis];
        }
      } else {
        tCal = 0;
        for(axis=0; axis<3; axis++)
          MAG_Zero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])/2;
      }
    }
    return 1;
  }
#endif

#if defined(BAROMETER) && defined(useBARO)
  void Baro_Common() {
    static int32_t baroHistTab[BARO_TAB_SIZE];
    static uint8_t baroHistIdx;
  
    uint8_t indexplus1 = (baroHistIdx + 1);
    if (indexplus1 == BARO_TAB_SIZE) indexplus1 = 0;
    baroHistTab[baroHistIdx] = baroPressure;
    baroPressureSum += baroHistTab[baroHistIdx];
    baroPressureSum -= baroHistTab[indexplus1];
    baroHistIdx = indexplus1;  
  }
#endif
#endif
