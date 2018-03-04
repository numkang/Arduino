void PID(){ //run at 500Hz so dt = 0.002
  gyro_D = (gyroData[PITCH]<<2)*0.0609756097561;
  error = setpoint - angle[PITCH]; //trim
  
  Pterm = error*KP;
  
  errorI += error*0.002;
  errorI = constrain(errorI,-150,150);
  Iterm = errorI*KI;
  Iterm = constrain(Iterm,-500,500);
  
  //errorD = (error - previous_error)/0.002;
  Dterm = gyro_D*KD;
  
  PID_CMD[0] = Pterm + Iterm - Dterm;
  
  //previous_error = error;
}

void PID_VDO(){
  error_VDO = setpoint_VDO - vdo_angle;
  
  Pterm_VDO = error_VDO*KP_VDO;
  
  errorI_VDO += error_VDO*0.002;
  errorI_VDO = constrain(errorI_VDO,-100,100);
  Iterm_VDO = errorI_VDO*KI_VDO;
  Iterm_VDO = constrain(Iterm_VDO,-250,250);
  
  errorD_VDO = (error_VDO - previous_error_VDO)/0.002;
  Dterm_VDO = errorD_VDO*KD_VDO;
  
  PID_CMD[0] = Pterm_VDO + Iterm_VDO + Dterm_VDO;
  
  previous_error_VDO = error_VDO;
}

void Mixtable(){
  PWM_Command[M1] = PID_MIX(-1,0,0); //D3  Left_Front   CW
  PWM_Command[M2] = PID_MIX(+1,0,0); //D10 Right_Front  CCW
  
  if(f.c == '1'){    
    PWMwrite();
    ON_LEDPIN_13
  }
  else{
    PWMwriteAll(MINCOMMAND);
    OFF_LEDPIN_13
  }
}

