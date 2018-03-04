/*void  print_print(){
  Serial.print("DEG : ");
  Serial.print(GPS_coord[LAT]);
  Serial.print(" , ");
  Serial.print(GPS_coord[LON]);
  Serial.print(" RAD : ");
  Serial.print(GPS_coord[LAT]*DEG_TO_RAD);
  Serial.print(" , ");
  Serial.print(GPS_coord[LON]*DEG_TO_RAD);
  Serial.print(" , ");
  Serial.print(GPS_altitude);  
  Serial.print("\r\n");
}*/

void serialCom(){
  n = Serial.available();
  if (n > 0){
    c = Serial.read();
    if(c != 0) GPS_NMEA_newFrame(c);    
    //Serial.print(c);
  }   
}

bool GPS_NMEA_newFrame(char c){     
    if (c == '$') {
      param = 0; 
      offset = 0; 
      parity = 0;       
      //GPS_payload_GGA = 0; 
      //GPS_payload_RMC = 0; 
      frame = 0; 
      frameOK = 0;
      //offset2 = 0;
    } 
    else if (c == ',' || c == '*'){
      string[offset] = 0;
      if (param == 0){ //frame identification
        if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A'){ frame = FRAME_GGA;  stop_GGA = 0;}
        if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C'){ frame = FRAME_RMC;  stop_RMC = 0;}
      } 
      else if (frame == FRAME_GGA){
        if (param == 1){
          for(uint8_t j=0; j<9; j++) time[j] = string[j];
        }
        else if (param == 2) { 
          GPS_coord[LAT] = GPS_coord_to_degrees(string); 
          //for(uint8_t j=0; j<sizeof(string); j++) lat[j] = string[j];
          isSouth = 0;
        }
        else if (param == 3 && string[0] == 'S') { GPS_coord[LAT] = -GPS_coord[LAT]; isSouth = 1;}
        else if (param == 4) {
          GPS_coord[LON] = GPS_coord_to_degrees(string); 
          //for(uint8_t j=0; j<sizeof(string); j++) lon[j] = string[j];
          isWest = 0;
        }
        else if (param == 5 && string[0] == 'W') { GPS_coord[LON] = -GPS_coord[LON]; isWest = 1;}
        else if (param == 6)                     { GPS_FIX = (string[0]  > '0'); }
        else if (param == 7)                     { GPS_numSat = grab_fields(string,0); }
        else if (param == 9)                     { GPS_altitude = grab_fields(string,1); }  //Altitude, Meters, above mean sea level
      }
      else if (frame == FRAME_RMC){
        /*if (param == 7){ 
          ground_speed = grab_fields(string,1)*0.1;          
        }//{GPS_speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //gps speed in cm/s will be used for navigation
        else if (param == 8) { GPS_ground_course = grab_fields(string,1); } //ground course deg*10 */
      }
      
      param++; 
      offset = 0;
      
      if (c == '*') checksum_param=1;
      else parity ^= c;
    } 
    else if (c == '\r' || c == '\n'){
      if (checksum_param) { //parity checksum
        uint8_t checksum = hex_c(string[0]);
        checksum <<= 4;
        checksum += hex_c(string[1]);
        if (checksum == parity) frameOK = 1;
      }
      checksum_param=0;
    }
    else{
       if (offset < 15)     string[offset++] = c;
       if (!checksum_param) parity ^= c;
    }
    //string2[offset2++] = c;
    
    /*if(GPS_payload_GGA < 128 && frame == FRAME_GGA && param >= 6 && !stop_GGA){ 
      gps_serial_GGA[GPS_payload_GGA++] = c; 
      if(c == '*') stop_GGA = 1;
    }
    
    if(GPS_payload_RMC < 128 && frame == FRAME_RMC && param >= 7 && !stop_RMC){ 
      gps_serial_RMC[GPS_payload_RMC++] = string2[offset2-1]; 
      if(c == '*') stop_RMC = 1;
    }*/
    
    if (frame) GPS_Present = 1;
    return frameOK && (frame==FRAME_GGA);
}
