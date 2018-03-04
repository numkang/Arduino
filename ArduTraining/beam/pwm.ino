void pwm_read(){
  //rc[0] = pulseIn(5,HIGH);
  rc[1] = pulseIn(6,HIGH);
  setpoint = (rc[1] - 1500)/20;
}

void pwm_write(){
  if(!startbyte){
    motorCMD[0] = 900;
    motorCMD[1] = 900;
  }
  
  if(incomingByte == 'z'){
    motorCMD[0] = 1200 + PID_CMD;
    motorCMD[1] = 1200 - PID_CMD;
  
    motorCMD[0] = constrain(motorCMD[0],900,1800);
    motorCMD[1] = constrain(motorCMD[1],900,1800);
    
    startbyte = 1;
  }
  else if(incomingByte == 'q'){
    motorCMD[0] = 900;
    motorCMD[1] = 900;
    
    startbyte = 0;
  }
  
  analogWrite(9, (motorCMD[0] >> 3));
  analogWrite(10, (motorCMD[1] >> 3));
}
