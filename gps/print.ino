void  print_print(){
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
}
