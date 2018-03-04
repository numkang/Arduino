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
  
  //acc_getdata();
  acc_getdata2();
  mag_data();
  //getEstimatedAttitude();
  
  //gyro_getdata();
    
  /*for(axis=0; axis<3; axis++) gyroADCp[axis] = gyroRAW[axis];
    
  gyro_getdata();
    
  for (axis = 0; axis < 3; axis++) {
    gyroADCinter[axis] = gyroRAW[axis]+gyroADCp[axis];
    gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
    gyroADCprevious[axis] = gyroADCinter[axis]>>1;
  }*/
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//             accelerometer gyroscope and magnetometer filter              //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
/*void getEstimatedAttitude(){
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
  rotateV(&EstM.V,deltaGyroAngle);

  validAcc = 72 < (uint16_t)accMag && (uint16_t)accMag < 133;

  for (axis = 0; axis < 3; axis++) {
    if ( validAcc )
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPFA_FACTOR + accSmooth[axis]) * INV_GYR_CMPFA_FACTOR;
    EstG32.A[axis] = EstG.A[axis]; //int32_t cross calculation is a little bit faster than float	
    
    EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + magRAW[axis]) * INV_GYR_CMPFM_FACTOR;
    EstM32.A[axis] = EstM.A[axis];      
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
  
  for(uint8_t i = 2; i < 3; i++){
    if(gyroRAW[i] >= gyroHigh[i] || gyroRAW[i] <= gyroLow[i])
    {
      dPS[i] = (float)gyroRAW[i] * 0.00908;
      
      if (currentTime > previousTime) dt = (float) (currentTime - previousTime)/1000000.0;
      else dt = (float) ((4294967295-previousTime)+currentTime)/1000000.0;
        
      gyroAngle[i] += ((gyroLast[i] + dPS[i])/2) * dt;
      //gyroAngle[i] = ((gyroLast[i] + dPS[i])/2) * dt; //average between two values
      
      if(gyroAngle[i] > 180) gyroAngle[i] -= 360;
      else if(gyroAngle[i] < -180) gyroAngle[i] += 360;
      
      gyroLast[i] = dPS[i];
    }
    else gyroLast[i] = 0;
  }
  angle[YAW] = gyroAngle[YAW];
}*/

void calc_angles(void){
   // Using x y and z from accelerometer, calculate x and y angles
   float x_val, y_val, z_val, result;
   unsigned long x2, y2, z2; //24 bit

   // Lets get the deviations from our baseline
   x_val = (float)accRAW2[0];
   y_val = (float)accRAW2[1];
   z_val = (float)accRAW2[2];
   // Work out the squares 
   x2 = (unsigned long)(long(x_val)*long(x_val));
   y2 = (unsigned long)(long(y_val)*long(y_val));
   z2 = (unsigned long)(long(z_val)*long(z_val));
   //X Axis
   result=sqrt(y2+z2);
   result=x_val/result;
   angle[ROLL] = atan(result)*RAD_TO_DEG;
   //Y Axis
   result=sqrt(x2+z2);
   result=y_val/result;
   angle[PITCH] = atan(result)*RAD_TO_DEG;
   
   //Magnetometer Vector
  mbx = magRAW[ROLL] - (60.393773);
  mby = magRAW[PITCH] - (-468.785824);
  mbz = magRAW[YAW] - (250.562423);
  
  mcal[X] = mbx* invm1 + mby*invm2 + mbz*invm3;
  mcal[Y] = mbx*invm4 + mby*invm5 + mbz*invm6;
  mcal[Z] = mbx*invm7 + mby*invm8 + mbz*invm9;
  
  heading = atan2(mcal[X], mcal[Y]);
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
 
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
 
  // Convert radians to degrees for readability.
  angle[YAW] = heading * 180/PI;
  
  if(angle[YAW] >= 0 && angle[YAW] <= 36) angle[YAW] = map(angle[YAW],0,36,0,45);
  else if(angle[YAW] >= 37 && angle[YAW] <= 62) angle[YAW] = map(angle[YAW],37,62,46,90);
  else if(angle[YAW] >= 63 && angle[YAW] <= 102) angle[YAW] = map(angle[YAW],63,102,91,135);
  else if(angle[YAW] >= 103 && angle[YAW] <= 189) angle[YAW] = map(angle[YAW],103,189,136,180);
  else if(angle[YAW] >= 190 && angle[YAW] <= 241) angle[YAW] = map(angle[YAW],190,241,181,225);
  else if(angle[YAW] >= 242 && angle[YAW] <= 276) angle[YAW] = map(angle[YAW],242,276,226,270);
  else if(angle[YAW] >= 277 && angle[YAW] <= 321) angle[YAW] = map(angle[YAW],277,321,271,315);
  else if(angle[YAW] >= 322 && angle[YAW] <= 359) angle[YAW] = map(angle[YAW],322,359,316,359);
  
  //if(y_init < 10) yaw_init = angle[YAW];
  //y_init++;
   
   /*for(uint8_t i = 2; i < 3; i++){
    if(gyroRAW[i] >= gyroHigh[i] || gyroRAW[i] <= gyroLow[i])
    {
      dPS[i] = (float)gyroRAW[i];// * 0.00908;
      
      if (currentTime > previousTime) dt = (float) (currentTime - previousTime)/1000000.0;
      else dt = (float) ((4294967295-previousTime)+currentTime)/1000000.0;
        
      gyroAngle[i] += ((gyroLast[i] + dPS[i])/2) * dt;
      //gyroAngle[i] = ((gyroLast[i] + dPS[i])/2) * dt; //average between two values
      
      if(gyroAngle[i] > 180) gyroAngle[i] -= 360;
      else if(gyroAngle[i] < -180) gyroAngle[i] += 360;
      
      gyroLast[i] = dPS[i];
    }
    else gyroLast[i] = 0;
  }
  angle[YAW] = gyroAngle[YAW];*/
  
  /*heading = atan2(magRAW[X], magRAW[Y]);
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
 
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
 
  // Convert radians to degrees for readability.
  angle[YAW] = heading * 180/PI;*/
}

