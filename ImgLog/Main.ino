void setup(void)
{
  Serial.begin(9600);
  pinMode(chipSelect, OUTPUT);
  SD.begin(chipSelect);
  Wire.begin();  
  RTC.begin();
  initPCINT();
  previousTime = micros();

  filename_setup();

  /*if (! logfile) {
    Serial.println("couldnt create file");
  }*/
  
  delay(100);
  now = RTC.now();
  logfile.print(now.year(), DEC);
  logfile.print("-");
  logfile.print(now.month(), DEC);
  logfile.print("-");
  logfile.print(now.day(), DEC);
  logfile.print("_");
  logfile.print(now.hour(), DEC);
  logfile.print("-");
  logfile.print(now.minute(), DEC);
  logfile.print("-");
  logfile.print(now.second(), DEC);
  logfile.println();
  logfile.flush();
  delay(100);
}

void loop(void)
{
  if(currentTime > task50Hz){ //50 Hz task
    task50Hz = currentTime + 20;    
    computeRC();      
  }
  
  currentTime = millis();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;
  serialCom();
  
  /*if (currentTime > Interval2s){
    Interval2s = currentTime + 200;
    fetch_time();
    logfile.flush();
  }*/

  if (RC_Command[4] > 1750 && !isTrig){
    Interval2s = currentTime + 2000;
    fetch_time();
    logfile.flush();
    isTrig = true;    
  }  
  else if(RC_Command[4] < 1750 && isTrig){
    isTrig = false;
  }

  if (currentTime > Interval1s){
    Interval1s = currentTime + 1000;
    //print_print();
  }
}
