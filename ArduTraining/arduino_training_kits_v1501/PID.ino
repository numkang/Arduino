void PIDc(){
if(f.CTRL_MODE == NORMAL){  
/*int16_t P_cmd,I_cmd,D_cmd;
static int16_t lastGyro;
int16_t delta,deltaSum,delta1,delta2;
angle_diff = angle_cmd - angle[PITCH];
P_cmd = (int32_t)angle_diff*PIDgains[P]/100;
angle_i  = constrain(angle_i+angle_diff,-10000,+10000);
I_cmd = ((int32_t)angle_i*PIDgains[I])>>10;

delta          = gyroData[PITCH] - lastGyro;                               // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
lastGyro = gyroData[PITCH];
deltaSum       = delta1+delta2+delta;
delta2   = delta1;
delta1   = delta;
 
    if (abs(deltaSum)<640) D_cmd = (deltaSum*PIDgains[D]/100)>>5;                       // 16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result 
                      else D_cmd = ((int32_t)deltaSum*PIDgains[D]/100)>>5;  */
int16_t P_cmd,I_cmd,D_cmd;
angle_diff = angle_cmd - angle[PITCH];
P_cmd = (int32_t)angle_diff*PIDgains[P]/100;
D_cmd = (int32_t)gyroData[PITCH]*PIDgains[D]/1000;
PID_cmd = P_cmd + I_cmd - D_cmd;



////////////////////////////////////
pPWM[M1] = motorCMD[M1] - PID_cmd;//
pPWM[M2] = motorCMD[M2] + PID_cmd;//
////////////////////////////////////

if(motorCMD[M1] <= 1070) pPWM[M1] = motorCMD[M1];
if(motorCMD[M2] <= 1070) pPWM[M2] = motorCMD[M2];
digitalWrite(13,LOW);
  }
  else{
  pPWM[M1] = motorCMD[M1];
  pPWM[M2] = motorCMD[M2];
  digitalWrite(13,HIGH);
  }
}

/*
 for(axis=0;axis<3;axis++) {
    if (f.ACC_MODE && axis<2 ) { //LEVEL MODE
      // 50 degrees max inclination
      errorAngle = constrain(2*rcCommand[axis] + GPS_angle[axis],-500,+500) - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
      #ifdef LEVEL_PDF
        PTerm      = -(int32_t)angle[axis]*conf.P8[PIDLEVEL]/100 ;
      #else  
        PTerm      = (int32_t)errorAngle*conf.P8[PIDLEVEL]/100 ;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
      #endif
      PTerm = constrain(PTerm,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);

      errorAngleI[axis]  = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
      ITerm              = ((int32_t)errorAngleI[axis]*conf.I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
    } 
    if (abs(gyroData[axis])<160) PTerm -=          gyroData[axis]*dynP8[axis]/10/8; // 16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
                            else PTerm -= (int32_t)gyroData[axis]*dynP8[axis]/10/8; // 32 bits is needed for calculation   

    delta          = gyroData[axis] - lastGyro[axis];                               // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = gyroData[axis];
    deltaSum       = delta1[axis]+delta2[axis]+delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;
 
    if (abs(deltaSum)<640) DTerm = (deltaSum*dynD8[axis])>>5;                       // 16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result 
                      else DTerm = ((int32_t)deltaSum*dynD8[axis])>>5;              // 32 bits is needed for calculation
                      
    axisPID[axis] =  PTerm + ITerm - DTerm;
  }

*/
