void setup() {
  initPCINT();
  previousTime = micros(); 
  SerialOpen(0,115200);
}

void loop() {
  previousTime = currentTime;
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  
  if(currentTime > taskTime50){ //50 Hz task
    taskTime50 = currentTime + 20000;    
    computeRC();
  }
  
  SerialPrint(100);
}
