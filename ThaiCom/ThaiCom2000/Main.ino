void setup(){
  initPWM();  
  digitalWrite(dirPin_azim, LOW);
  digitalWrite(dirPin_elev, LOW);
  delay(100);
  previousTime = micros();
  SerialOpen(0,115200);
  //initIMU();
}

void loop(){
  previousTime = currentTime;
  currentTime = micros();
  cycleTime = currentTime - previousTime;  
  serialCom(0);
  
  if(currentTime > taskTime400){ //400 Hz task
    taskTime400 = currentTime + 2500;
    stepDrive(step_c); 
    //acc_getdata2(); 
    //mag_getdata();  
    //computeIMU();
  }  
  //SerialPrint(100);
}
