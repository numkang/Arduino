//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                               PWM General Function                       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void PID(){
  
  /*#if defined(BAROMETER) && defined(useBARO) && !defined(IFO)
    if (f.BARO_MODE) {
      static uint8_t isAltHoldChanged = 0;
      static int16_t AltHoldCorr = 0;
      if (abs(RC_Command[THR] - initialThrottleHold) > ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
        AltHoldCorr += RC_Command[THR] - initialThrottleHold;
        if(abs(AltHoldCorr) > 512) {
          AltHold += AltHoldCorr >> 9;
          AltHoldCorr %= 512;
        }
        isAltHoldChanged = 1;
      } else if (isAltHoldChanged) {
        AltHold = EstAlt;
        isAltHoldChanged = 0;
      }
      RC_Command[THR] = initialThrottleHold + BaroPID;
      
      #if defined(THROTTLE_ANGLE_CORRECTION)
        if(f.ANGLE_MODE || f.HORIZON_MODE) {
        RC_Command[THR] += throttleAngleCorrection;
        }
      #endif      
    }
  #endif */
  
  if ( f.HORIZON_MODE ) prop = min(max(abs(RC_Command[PITCH+1]),abs(RC_Command[ROLL+1])),512);

  // PITCH & ROLL
  for(uint8_t axis=0;axis<2;axis++) {
    rc = RC_Command[axis+1]<<1;
    error = rc - gyroData[axis];
    errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);       // WindUp   16 bits is ok here
    if (abs(gyroData[axis])>640) errorGyroI[axis] = 0;

    ITerm = (errorGyroI[axis]>>7)*GKi[axis]>>6;                        // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000

    PTerm = (int32_t)rc*GKp[axis]>>6;

    if (f.ANGLE_MODE || f.HORIZON_MODE) { // axis relying on ACC
      // 50 degrees max inclination
      errorAngle         = constrain(rc,-500,+500) - angle[axis]; //16 bits is ok here
      errorAngleI[axis]  = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);                                                // WindUp     //16 bits is ok here

      PTermACC           = ((int32_t)errorAngle*AKp[axis])>>7; // 32 bits is needed for calculation: errorAngle*P8 could exceed 32768   16 bits is ok for result
      
      int16_t limit      = AKd[axis]*5;
      PTermACC           = constrain(PTermACC,-limit,+limit);

      ITermACC           = ((int32_t)errorAngleI[axis]*AKi[axis])>>12;   // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result

      ITerm              = ITermACC + ((ITerm-ITermACC)*prop>>9);
      PTerm              = PTermACC + ((PTerm-PTermACC)*prop>>9);
    }

    PTerm -= ((int32_t)gyroData[axis]*GKp[axis])>>6; // 32 bits is needed for calculation   
    
    delta          = gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = gyroData[axis];
    DTerm          = delta1[axis] + delta2[axis] + delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;
 
    DTerm = ((int32_t)DTerm*GKd[axis])>>5;        // 32 bits is needed for calculation

    PID_CMD[axis] =  PTerm + ITerm - DTerm;
  }

  //YAW
  #define GYRO_I_MAX 250

  rc = (int32_t)RC_Command[YAW+1] * (2*GKd[YAW] + 30) >> 5;

  error = rc - gyroData[YAW];
  errorGyroI[YAW] += (int32_t)error*GKi[YAW];
  errorGyroI[YAW] = constrain(errorGyroI[YAW], 2-((int32_t)1<<28), -2+((int32_t)1<<28));
  if (abs(rc) > 50) errorGyroI[YAW] = 0;
  
  PTerm = (int32_t)error*GKp[YAW]>>6;
  
  ITerm = constrain((int16_t)(errorGyroI[YAW]>>13),-GYRO_I_MAX,+GYRO_I_MAX);
  
  PID_CMD[YAW] =  PTerm + ITerm;  
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                MixTable Function                         //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined (QUADX)
void MixTable(){
  PWM_Command[M1] = PID_MIX(+1,-1,-1); //D3  Left_Front   CW
  PWM_Command[M2] = PID_MIX(-1,-1,+1); //D10 Right_Front  CCW
  PWM_Command[M3] = PID_MIX(-1,+1,-1); //D9  Right_Rear   CW
  PWM_Command[M4] = PID_MIX(+1,+1,+1); //D11 Left_Rear    CCW
  if(f.ARM && RC_Command[THR] > 1050 && !f.motorCalibrated) PWMwrite();
  //if(f.ARM && !f.motorCalibrated) PWMwrite(); 
  else PWMwriteAll(MINCOMMAND);
}
#endif

#if defined (HEXX)
void MixTable(){
  PWM_Command[M1] = PID_MIX(+1,-1,-1); //D3  Left_Front   CW
  PWM_Command[M2] = PID_MIX(-1,-1,+1); //D10 Right_Front  CCW
  PWM_Command[M3] = PID_MIX(-1,+1,-1); //D9  Right_Rear   CW
  PWM_Command[M4] = PID_MIX(+1,+1,+1); //D11 Left_Rear    CCW
  PWM_Command[M5] = PID_MIX(+1, 0,+1); //D6 Left_Center   CCW
  PWM_Command[M6] = PID_MIX(-1, 0,-1); //D8 Right_Center  CW
  if(f.ARM && RC_Command[THR] > 1050 && !f.motorCalibrated) PWMwrite();
  else PWMwriteAll(MINCOMMAND);
}
#endif

#if defined (IFO)
void MixTable(){
  PWM_Command[M1] = PID_MIX(+1,-1,-1); //D3  Left_Front   CW
  PWM_Command[M2] = PID_MIX(-1,-1,+1); //D10 Right_Front  CCW
  PWM_Command[M3] = PID_MIX(-1,+1,-1); //D9  Right_Rear   CW
  PWM_Command[M4] = PID_MIX(+1,+1,+1); //D11 Left_Rear    CCW
  if(RC_Command[AUX1+1] < 1500) {
    PWM_Command[M5] = PID_MIX( 0, 0, 0); //D6 Left_Center   CCW
    PWM_Command[M6] = PID_MIX( 0, 0, 0); //D8 Right_Center  CW
  }
  else{
    PWM_Command[M5] = initialThrottleHold;
    PWM_Command[M6] = initialThrottleHold;
  }
  if(f.ARM && RC_Command[THR] > 1050 && !f.motorCalibrated) PWMwrite();
  else PWMwriteAll(MINCOMMAND);
}
#endif

#if defined (MAVion)
void MixTable(){
  PWM_Command[M1] = PID_MIX(0,0,+1); //Left
  PWM_Command[M2] = PID_MIX(0,0,-1); //Right
  PWM_Command[S1] = 1500 - PID_CMD[ROLL] + PID_CMD[PITCH]; //Left
  PWM_Command[S2] = 1500 - PID_CMD[ROLL] - PID_CMD[PITCH]; //Right
  if(f.ARM && RC_Command[THR] > 1050 && !f.motorCalibrated) PWMwrite();   
  else PWMwriteAll(MINCOMMAND);
}
#endif
