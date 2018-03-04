void pid(){
  error = setpoint - trueAngle[ROLL];
  
  Pterm = error*KP;
  
  errorI += error*0.002;
  errorI = constrain(errorI,-150,150);
  Iterm = errorI*KI;
  Iterm = constrain(Iterm,-500,500);
  
  Dterm = gyroRAW[ROLL]*KD;
  
  PID_CMD = Pterm + Iterm - Dterm;
}
