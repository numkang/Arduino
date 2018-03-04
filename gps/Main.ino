void setup(){
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
}

void loop(){
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;   
  
  serialCom();
  
  if(currentTime >= taskTime){ //1Hz task
    taskTime = currentTime + 1000000;
        
    Serial.print("$GPGGA,");
    for(uint8_t j=0; j<9; j++) Serial.print(time[j]);
    Serial.print(",");
    Serial.print(lat);
    Serial.print(",");
    if(isSouth == 1) Serial.print('S');
    else Serial.print('N');
    Serial.print(",");
    Serial.print(lon);
    Serial.print(",");
    if(isWest == 1) Serial.print('W');
    else Serial.print('E');
    Serial.print(gps_serial_GGA); 
    
    parity_GGA = 0;
    parity_GGA ^= 'G';
    parity_GGA ^= 'P';
    parity_GGA ^= 'G';
    parity_GGA ^= 'G';
    parity_GGA ^= 'A';
    parity_GGA ^= ',';
    for(uint8_t j=0; j<sizeof(time); j++) parity_GGA ^= time[j];
    parity_GGA ^= ',';
    for(uint8_t j=0; j<sizeof(lat); j++) parity_GGA ^= lat[j];
    parity_GGA ^= ',';  
    if(isSouth == 1) parity_GGA ^= 'S';
    else parity_GGA ^= 'N';
    parity_GGA ^= ','; 
    for(uint8_t j=0; j<sizeof(lon); j++) parity_GGA ^= lon[j];
    parity_GGA ^= ',';
    if(isWest == 1) parity_GGA ^= 'W';
    else parity_GGA ^= 'E';
    for(uint8_t j=0; j<sizeof(gps_serial_GGA); j++){ if(gps_serial_GGA[j] != '*') parity_GGA ^= gps_serial_GGA[j]; }
    
    hextochar(parity_GGA);   
    Serial.print(checksum[0]);
    Serial.print(checksum[1]);
    Serial.print("\r\n");
    
    Serial.print("$GPRMC,");
    for(uint8_t j=0; j<9; j++) Serial.print(time[j]);
    Serial.print(",A,");
    Serial.print(lat);
    Serial.print(",");
    if(isSouth == 1) Serial.print('S');
    else Serial.print('N');
    Serial.print(",");
    Serial.print(lon);
    Serial.print(",");
    if(isWest == 1) Serial.print('W');
    else Serial.print('E');
    Serial.print(gps_serial_RMC);
    
    parity_RMC = 0;
    parity_RMC ^= 'G';
    parity_RMC ^= 'P';
    parity_RMC ^= 'R';
    parity_RMC ^= 'M';
    parity_RMC ^= 'C';
    parity_RMC ^= ',';
    for(uint8_t j=0; j<sizeof(time); j++) parity_RMC ^= time[j];
    parity_RMC ^= ',';
    parity_RMC ^= 'A';
    parity_RMC ^= ',';
    for(uint8_t j=0; j<sizeof(lat); j++) parity_RMC ^= lat[j];
    parity_RMC ^= ',';  
    if(isSouth == 1) parity_RMC ^= 'S';
    else parity_RMC ^= 'N';
    parity_RMC ^= ','; 
    for(uint8_t j=0; j<sizeof(lon); j++) parity_RMC ^= lon[j];
    parity_RMC ^= ',';
    if(isWest == 1) parity_RMC ^= 'W';
    else parity_RMC ^= 'E';
    for(uint8_t j=0; j<sizeof(gps_serial_RMC); j++){ if(gps_serial_RMC[j] != '*') parity_RMC ^= gps_serial_RMC[j]; }
    
    hextochar(parity_RMC);
    Serial.print(checksum[0]);
    Serial.print(checksum[1]); 
    Serial.print("\r\n");
    //print_print();  
    
    for(uint8_t i=0; i<128; i++)
    {
    gps_serial_GGA[i] = 0;
    gps_serial_RMC[i] = 0;
    }    
  }
}
