void SerialRead(){
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
  }
}
