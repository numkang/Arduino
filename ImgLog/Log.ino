void filename_setup(){
  for (uint16_t i = 0; i < 10000; i++) {
    filename[4] = i/1000 + '0';
    filename[5] = i/100 + '0';
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
}

void fetch_time(){
  // fetch the time
  now = RTC.now();
  //log time
  //logfile.print(now.unixtime()); // seconds since 1/1/1970
  //logfile.print(", ");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print("/");
  logfile.print(now.year(), DEC);
  logfile.print(" ");
  if(now.hour() >= 13 && now.hour() < 24){
    logfile.print(now.hour()-12, DEC);
  }
  else if(now.hour() == 0) logfile.print(now.hour()+12, DEC);
  else if(now.hour() >= 1 && now.hour() <= 12){
    logfile.print(now.hour(), DEC);
  }
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  if(now.hour() >= 12 && now.hour() < 24){
    logfile.print(" PM");
  }
  else if(now.hour() >= 0 && now.hour() < 12){
    logfile.print(" AM");
  }
  logfile.println();
  logfile.print(",");
  logfile.print(GPS_coord[LAT]*0.0000001);
  logfile.print(",");
  logfile.print(GPS_coord[LON]*0.0000001);
  logfile.print(",");
  logfile.print(GPS_altitude);
  logfile.println();
}

