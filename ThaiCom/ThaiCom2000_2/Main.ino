void setup(){
  initPWM();  
  digitalWrite(dirPin_azim, LOW);
  digitalWrite(dirPin_elev, LOW);
  initPCINT();
  delay(100);
  previousTime = micros();
  SerialOpen(0,115200);
  initIMU();
  //gyro_calibrate();
  for(int ii = 0; ii < 10; ii++){
    computeIMU();
    IMUCalibrate();
  }
}

void loop(){
  previousTime = currentTime;
  currentTime = micros();
  cycleTime = currentTime - previousTime;  
  serialCom(0);
  
  if(currentTime > taskTime50){ //50 Hz task
    taskTime50 = currentTime + 20000;    
    computeRC();
  }
  
  if(currentTime > taskTime400){ //400 Hz task
    taskTime400 = currentTime + 2500;
    if(step_c == 'A' || step_c == 'S' || step_c == 'D' || step_c == 'W' || step_c == 'a' || step_c == 's' || step_c == 'd' || step_c == 'w' || step_c == 'q') stepDrive(step_c); 
    else if(step_c == 'z') stepDrive_Auto_azim();
    else if(step_c == 'x') stepDrive_Auto_elev();
    //else if(step_c == 'c'){if(step_c != 'q') stepDrive_Auto();}
    else if(step_c == 'v'){pitch_check = 0; yaw_check = 0;}
    else stepDrive_Auto();
    computeIMU();
    calc_angles();
    //sensorVal = analogRead(A0);  
  } 
  sensorVal = analogRead(A0);
  sensorTime();
  SerialPrint(100);
}
