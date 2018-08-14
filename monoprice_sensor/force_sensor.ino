void force_setup(){
  
  pinMode(Pin_slk, OUTPUT);
  pinMode(Pin_dout, INPUT);

  digitalWrite(Pin_slk, HIGH);
  delayMicroseconds(100);
  digitalWrite(Pin_slk, LOW);

  for(int i = 0; i < avg_num; i++){
//    Serial.println("aaa");
  while (digitalRead(Pin_dout))
    ;

  for (byte j = 3; j--;)
  {
    for (char i = 8; i--;)
    {
      digitalWrite(Pin_slk, HIGH);
      bitWrite(data[j], i, digitalRead(Pin_dout));
      digitalWrite(Pin_slk, LOW);
    }
  }

  digitalWrite(Pin_slk, HIGH);
  digitalWrite(Pin_slk, LOW);

  data[2] ^= 0x80;

  val = ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8)
      | (uint32_t) data[0];

  val2 = val/1000;
  avg = avg + val2;
  }
  force_offset = avg/avg_num;

  //force_ratio = (w1-force_offset)/w2;
  avg = 0;
}

void sensor_loop() {

 while (digitalRead(Pin_dout))
    ;

  for (byte j = 3; j--;)
  {
    for (char i = 8; i--;)
    {
      digitalWrite(Pin_slk, HIGH);
      bitWrite(data[j], i, digitalRead(Pin_dout));
      digitalWrite(Pin_slk, LOW);
    }
  }

  digitalWrite(Pin_slk, HIGH);
  digitalWrite(Pin_slk, LOW);

  data[2] ^= 0x80;

  val = ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8)
      | (uint32_t) data[0];

  val2 = val/1000;
  //gram = (val2 - force_offset)/force_ratio;

  gram = val2 - force_offset;
}
