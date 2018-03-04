//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           calculation function                           //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define ssin(val) (val)
#define scos(val) 1.0f
#define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)

int16_t _atan2(float y, float x){ 
  float z = y / x;
  int16_t zi = abs(int16_t(z * 100)); 
  int8_t y_neg = fp_is_neg(y);
  if ( zi < 100 ){
    if (zi > 10) 
     z = z / (1.0f + 0.28f * z * z);
   if (fp_is_neg(x)) {
     if (y_neg) z -= PI;
     else z += PI;
   }
  } else {
   z = (PI / 2.0f) - z / (z * z + 0.28f);
   if (y_neg) z -= PI;
  }
  z *= (180.0f / PI * 10); 
  return z;
}

void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X; 
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

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                    complementary filter initialize                       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void computeIMU(){
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  int16_t gyroADCp[3];
  int16_t gyroADCinter[3];
  
  #if defined(ACCELEROMETER)
    acc_getdata();
    getEstimatedAttitude();
  #endif
  
  #if defined(GYROSCOPE)
    gyro_getdata();
  
    for(axis = 0; axis < 3; axis++)
      gyroADCp[axis] = gyroRAW[axis];

    gyro_getdata();
    
    for(axis = 0; axis < 3; axis++){
      gyroADCinter[axis] = gyroRAW[axis] + gyroADCp[axis];
      gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis])/3;
      gyroADCprevious[axis] = gyroADCinter[axis]/2;      
    }
  #endif
  
  #if defined(useBARO)  
    baro_getdata();
    getEstimatedAltitude();
  #endif
  
  #if defined(ULTRASONIC) && defined(useSONAR)
    sonar_getdata();
  #endif 
  
  #if defined(MAGNETOMETER) && defined(useMAG)
    mag_getdata();
  #endif
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//             accelerometer gyroscope and magnetometer filter              //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void getEstimatedAttitude(){
  uint8_t axis;
  int32_t accMag = 0;
  static t_fp_vector EstG;
  static t_fp_vector EstM;
  static float accLPF[3];

  static uint16_t previousT;
  uint16_t currentT = micros();
  float scale, deltaGyroAngle[3];

  scale = (currentT - previousT) * GYRO_SCALE;
  previousT = currentT;

  for (axis = 0; axis < 3; axis++){
    deltaGyroAngle[axis] = gyroRAW[axis] * scale;
    
    accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accRAW[axis] * (1.0f/ACC_LPF_FACTOR);
    accSmooth[axis] = accLPF[axis];
    
    #define ACC_VALUE accSmooth[axis]
    accMag += (int32_t)ACC_VALUE*ACC_VALUE ;
    
    #if defined(useMAG)
      #define MAG_VALUE magRAW[axis]
    #endif
  }
  
  accMag = accMag*100/((int32_t)acc_1G*acc_1G);

  rotateV(&EstG.V,deltaGyroAngle);
  
  #if defined(useMAG)
    rotateV(&EstM.V,deltaGyroAngle);
  #endif

  if ( abs(accSmooth[ROLL])<acc_25deg && abs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0) {
    f.SMALL_ANGLES_25 = 1;
  } else {
    f.SMALL_ANGLES_25 = 0;
  }

  if ( ( 36 < accMag && accMag < 196 ) || f.SMALL_ANGLES_25 )
    for (axis = 0; axis < 3; axis++) {
      int16_t acc = ACC_VALUE;
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPFA_FACTOR + acc) * INV_GYR_CMPFA_FACTOR;
    }
    #if defined(useMAG)
      for (axis = 0; axis < 3; axis++)
        EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
    #endif

  angle[ROLL]  = _atan2(EstG.V.X , EstG.V.Z);
  angle[PITCH] = _atan2(EstG.V.Y , EstG.V.Z);
  
  #if defined(useMAG)
    angle[YAW] = _atan2( EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X , EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z  );
    angle[YAW] += MAG_DECLINIATION * 10; //add declination
    angle[YAW] = angle[YAW] /10;
    if ( angle[YAW] > 180)      angle[YAW] = angle[YAW] - 360;
    else if (angle[YAW] < -180) angle[YAW] = angle[YAW] + 360;
  #endif
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                              barometer filter                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined(BAROMETER) 

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
  static int32_t baroGroundPressure;
  static uint16_t previousT;
  uint16_t currentT = micros();
  uint16_t dTime;

  dTime = currentT - previousT;
  if (dTime < UPDATE_INTERVAL) return 0;
  previousT = currentT;

  if(calibratingB > 0) {
    baroGroundPressure = baroPressureSum/(BARO_TAB_SIZE - 1);
    calibratingB--;
  }

  BaroAlt = log( baroGroundPressure * (BARO_TAB_SIZE - 1)/ (float)baroPressureSum ) * (baroTemperature+27315) * 29.271267f; // in cemtimeter 

  EstAlt = (EstAlt * 6 + BaroAlt * 2) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)
  return 1;
}

#endif
