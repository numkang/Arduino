void setup() {
  // put your setup code here, to run once:
  previousTime = micros();
  SerialOpen(0,115200);
  initPWM();
  delay(100);
  //CalibrateMotor();
  PWMwriteAll(MINCOMMAND);  
  delay(3000);
  initIMU();
  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  previousTime = currentTime;
  currentTime = micros();
  cycleTime = currentTime - previousTime;  
  serialCom(0);

  if(currentTime > taskLED){
    taskLED = currentTime + 300000;    
    if(isConnect){
      if(f.ARM) digitalWrite(13,!digitalRead(13));
      else digitalWrite(13,HIGH);
    }
  }

  if(currentTime > taskTime400){ //400 Hz task
    taskTime400 = currentTime + 2500;    
    computeIMU();
  }
  
  if(currentTime > taskTime500){ //500 Hz task
    taskTime500 = currentTime + 2000;
    if(f.ARM){
      PID();
    }
    else PWMwriteAll(MINCOMMAND); 
  }
  //SerialPrint(100);
}
