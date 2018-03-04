//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           calculation function                           //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                    complementary filter initialize                       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void computeIMU(){  
  uint8_t axis;
  
    acc_getdata();
    getEstimatedAttitude();
    
    gyro_getdata();
    
    for(axis=0; axis<3; axis++) gyroADCp[axis] = gyroRAW[axis];
    
    gyro_getdata();
    
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] = gyroRAW[axis]+gyroADCp[axis];
      gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
      gyroADCprevious[axis] = gyroADCinter[axis]>>1;
    }
  
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
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//             accelerometer gyroscope and magnetometer filter              //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

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

  validAcc = 72 < (uint16_t)accMag && (uint16_t)accMag < 133;

  for (axis = 0; axis < 3; axis++) {
    if ( validAcc )
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPFA_FACTOR + accSmooth[axis]) * INV_GYR_CMPFA_FACTOR;
    EstG32.A[axis] = EstG.A[axis]; //int32_t cross calculation is a little bit faster than float  
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
  angle_f[ROLL] = angle[ROLL]/10.0;
  angle_f[PITCH] = angle[PITCH]/10.0;
}
