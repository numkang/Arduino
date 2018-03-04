//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 Main function                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
    
void setup() {
  initPWM();
  //CalibrateMotor(); //cannot fly
  PWMwriteAll(MINCOMMAND); //comment this if calibrate motors
  initPCINT();
  previousTime = micros();   
  //Serial.begin(115200);
  SerialOpen(115200);
  initEEPROM();
  #if !defined (CHR_UM6)
    initIMU();
  #endif
  initConfig();
}

void loop() {  
  if(currentTime > taskTime50){ //50 Hz task
    taskTime50 = currentTime + 20000;    
    computeRC();
    ModeConfig();
    serialCom();
  }
  #if !defined (CHR_UM6)
  /*else {
    static uint8_t taskOrder=0; // never call all functions in the same loop, to avoid high delay spikes
    if(taskOrder>2) taskOrder-=3;
    switch (taskOrder) {
      case 0:
        taskOrder++;
        #if defined(MAGNETOMETER) && defined(useMAG)
          if (mag_getdata()) break;
        #endif
      case 1:
        taskOrder++;
        #if defined(BAROMETER) && defined(useBARO)          
          if (baro_getdata() != 0 ) break;
        #endif
      case 2:
        taskOrder++;
        #if defined(BAROMETER) && defined(useBARO)
          if (getEstimatedAltitude() !=0 ) break;
        #endif
    }
  }  */
  computeIMU();
  #else  
  CHR_UM6_getdata();
  #endif
  
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;  
  
  if(currentTime > taskTime500){ //500 Hz task
    taskTime500 = currentTime + 2000;
    PID();
    MixTable();
  }
  
  if(currentTime > taskTime4){ //4Hz task
    taskTime4 = currentTime + 250000;
    failSafe();    
  }    
  //Print(250);
}

void initConfig(){
  pinMode(13, OUTPUT);
  //pinMode(30, OUTPUT);
  //pinMode(31, OUTPUT);
  f.DISARM = 1;
  f.ARM = 0;
  f.motorCalibrated = 0;
  f.BARO_MODE = 0;
  f.MOUSE_MODE = 0;
  f.KINECT_MODE = 0;
  ModeConfig();
  ALL_LED(3,100)
  ALL_LED(3,100)
  ALL_LED(3,100)
  delay(1000);
  OFF_LEDPIN_13
  OFF_LEDPIN_30
  OFF_LEDPIN_31
}

void ModeConfig(){
  if (RC_Command[AUX1+1] < 1450) f.Mode_Config = 2;
  else if (RC_Command[AUX1+1] > 1550) f.Mode_Config = 1;
  else f.Mode_Config = 0;
  
  if(f.Mode_Config_temp != f.Mode_Config){ TOGGLE_LEDPIN_31 }
  
  switch(f.Mode_Config){
    case 0:
      f.ANGLE_MODE = 1;
      f.HORIZON_MODE = 0;
      f.RATE_MODE = 0;
    break;
    
    case 1:
      f.ANGLE_MODE = 0;
      f.HORIZON_MODE = 1;
      f.RATE_MODE = 0;
    break;
    
    case 2:
      f.ANGLE_MODE = 0;
      f.HORIZON_MODE = 0;
      f.RATE_MODE = 1;
    break;
  }
  
  f.Mode_Config_temp = f.Mode_Config;
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                              FailSafe function                           //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void failSafe(){
  if(RC_Command[THR] <= 1050){
    if(f.ARM == 1) TOGGLE_LEDPIN_13    
    armDisarm();
  }
  else OFF_LEDPIN_13  
}

void armDisarm(){
  if(rcValue[3] < 1050 && f.ARM == 1){ //disarm
    f.DISARM = 1;
    f.ARM = 0;
    BLINK_LEDPIN_31(6,50)
    delay(200);
    OFF_LEDPIN_13  
    OFF_LEDPIN_30
    OFF_LEDPIN_31  
  }
  
  else if(rcValue[3] > 1800 && f.DISARM == 1){ //arm
    f.DISARM = 0;
    f.ARM = 1;
    BLINK_LEDPIN_31(6,50)
    #if !defined (CHR_UM6)
      calibratingA = 400;
      calibratingG = 400;    
      #if defined(BAROMETER) && defined(useBARO)
        calibratingB = 200;
      #endif  
    #endif  
    delay(200);
    OFF_LEDPIN_13
    ON_LEDPIN_30
    OFF_LEDPIN_31
  }
}
