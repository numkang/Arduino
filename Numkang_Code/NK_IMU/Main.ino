void setup(){

  //Wire.begin();
  Serial.begin(115200);
  
  //gyro_init();
  //acc_init();
  
  //delay(1500); //wait for the sensor to be ready 
  //gyro_calibrate();
  initPWM();
  initPCINT();
}

void loop()
{
  //gyro_getdata();
  //acc_getdata();
  
  previousTime = currentTime;  
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  
  //filter();
  
  
  if(currentTime > tasktime50){ //50 Hz task
    tasktime50 = currentTime + 20000;    
    computeRC();    
  }
  
  if(currentTime > tasktime){
    tasktime = currentTime + 50000;
    Print();
  }
}
