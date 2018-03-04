void setup() {
  previousTime = micros();
  //SerialOpen(0,115200);
  Serial.begin(115200);
  initPWM();
  delay(100);
  //CalibrateMotor();
  PWMwriteAll(MINCOMMAND);  
  delay(3000);
  initIMU();
  //gyro_calibrate();
  pinMode(13,OUTPUT);
}

void loop() {
  previousTime = currentTime;
  currentTime = micros();
  cycleTime = currentTime - previousTime;  
  //serialCom(0);
  
  if(currentTime > taskTime400){ //400 Hz task
    taskTime400 = currentTime + 2500;    
    computeIMU();
    //Println(0,(gyroData[PITCH]<<2)*0.0609756097561);//-0.2439024390244*2);
  }
  
  if(currentTime > taskTime500){ //500 Hz task
    taskTime500 = currentTime + 2000;
    if(f.vdo_active == 1){ PID_VDO(); }
    else{ PID(); }
    Mixtable();
  }
  SerialPrint(100);
}
