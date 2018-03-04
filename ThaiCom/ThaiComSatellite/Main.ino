void setup() {   
  initPWM();  
  digitalWrite(dirPin_azim, LOW); //CCW - LOW, CW - HIGH
  digitalWrite(dirPin_elev, LOW);
  delay(100);
  Wire.begin(); //nunchuck  
  delay(100);
  nunchuck_init();
  delay(100);
  //initEEPROM();
  //delay(100);
  Serial.begin(115200);
  //SerialOpen(115200);
  delay(100);
  init_adis16405();
  delay(1000);
  gyro_calibrate();
  int i = 0;
  /*while(i < 500){
    read_adis16405(&isense_data);    
    IMUCalibrate();
    calibrateVector();
  
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    previousTime = currentTime;  
    //Print(100);
    i++;
  }*/
}

void loop(){
  if(currentTime > taskTime50){ //50 Hz task
    taskTime50 = currentTime + 20000;         
  }
  
  read_adis16405(&isense_data);
  IMUCalibrate();
  
  previousTime = currentTime; 
  currentTime = micros();
  cycleTime = currentTime - previousTime;    
  
  potenVal_azim = mapfloat(analogRead(potenPin_azim),poten_azim_min,poten_azim_max,-90.0,90.0);
  potenVal_elev = mapfloat(analogRead(potenPin_elev),poten_elev_min,poten_elev_max,-180.0,0.0);  
  
  //calibrateVector();
  TransformVector();
  if(nunchuck_zbutton() == 0) stepDrive(newAzim, newElev);
  
  if(currentTime > taskTime500){ //500 Hz task
    taskTime500 = currentTime + 2000;
    nunchunk_update();
  }
  
  if(nunchuck_zbutton() == 1) WiistepDrive();
  
  //if(nunchuck_cbutton() == 0) stepDrive(0,-90); //azim, elev
  
  Print(100);
}
