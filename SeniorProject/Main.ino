void setup(){
  initPWM();
  initPCINT();
  delay(100);
  previousTime = micros();
  SerialOpen(0,115200);
  pinMode(13,OUTPUT);
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
    PID();
  } 
  
  //SerialPrint(100);
}
