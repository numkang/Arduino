void setup(){
  initPWM();  
  digitalWrite(dirPin_azim, LOW);
  digitalWrite(dirPin_elev, LOW);
  delay(100);
  previousTime = micros();
  SerialOpen(0,115200);
  initIMU();
  gyro_calibrate();
}

void loop(){
  previousTime = currentTime;
  currentTime = micros();
  cycleTime = currentTime - previousTime;  
  serialCom(0);
  
  if(currentTime > taskTime400){ //400 Hz task
    taskTime400 = currentTime + 2500;
    stepDrive(step_c); 
    computeIMU();
    mag_getdata();
    calc_xy_angles();
  }  
  SerialPrint(500);
}
