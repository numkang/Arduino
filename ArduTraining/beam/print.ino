static unsigned long printTime;

void SerialPrint(unsigned long ms){  
  if(currentTime > printTime){
    printTime = currentTime + ms*1000;
    SerialPrintText();
  }
}

void SerialPrintText(){
  Serial.println("gyro: ");
  Serial.println(gyroRAW[X]); 
  Serial.println(gyroRAW[Y]);
  Serial.println(gyroRAW[Z]);
  //Serial.println(gyroAngle[ROLL]);
  //Serial.println(gyroAngle[PITCH]);
  //Serial.println(gyroAngle[YAW]);
  
  Serial.println("acc: ");
  Serial.println(accRAW[X]);
  Serial.println(accRAW[Y]);
  Serial.println(accRAW[Z]);
  //Serial.println(accAngle[ROLL]);
  //Serial.println(accAngle[PITCH]);

  //Serial.println(dt);
  /*Serial.println("gyro: ");
  Serial.print(gyroVectorX);
  Serial.print(" ");
  Serial.print(gyroVectorY);
  Serial.print(" ");
  Serial.print(gyroVectorZ);
  Serial.println();
  
  Serial.println("acc: ");
  Serial.print(n_accX);
  Serial.print(" ");
  Serial.print(n_accY);
  Serial.print(" ");
  Serial.print(n_accZ);
  Serial.println();*/
  
  /*Serial.println("true: ");
  Serial.println(trueVectorX);
  Serial.println(trueVectorY);
  Serial.println(trueVectorZ);*/
  
  //Serial.println(trueAngle[ROLL]);
  //Serial.println(trueAngle[PITCH]);
  //Serial.println(PID_CMD);
  //Serial.println(motorCMD[0]);
  //Serial.println(motorCMD[1]);
  
  //Serial.println(rc[0]);
  //Serial.println(rc[1]);
  Serial.println();
}
