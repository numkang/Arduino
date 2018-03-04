#if defined (MEGA)

void PWMinit(){
  DDRL |= (_BV(DDL3) | _BV(DDL4) | _BV(DDL5)); //set PORTL (D46, D45, D44) as an OUTPUT (SERVO) 50 hz
  
  TCCR5A |= (_BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1)); //set CompareOutputMode to D46, D45, D44
  TCCR5A |= _BV(WGM51); TCCR5A &= ~(_BV(WGM50)); TCCR5B |= _BV(WGM53); //use Phase Correct 16 bits
  TCCR5B &= ~(_BV(CS50)); //set Prescaler to 8 bits
  ICR5 = 0x4E20; //20000
  //for SERVO
}

void PWMwrite(){
  
  //SERVO
  OCR5A = 1400; //D46
  //OCR5B = ; //D45
  //OCR5C = ; //D44
}

#endif
