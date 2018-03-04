//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                            PWM General Function                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void PWMwriteAll(int16_t pwm) {   // Sends commands to all motors
  for (uint8_t i = 0; i<4; i++){
    PWM_Command[i] = pwm;
  }
  PWMwrite();
}

void CalibrateMotor(){ //run only once
  PWMwriteAll(MAXCOMMAND);
  delay(3000);
  BLINK_LEDPIN_31(4,150)
  PWMwriteAll(MINCOMMAND);
  delay(3000);
  BLINK_LEDPIN_31(4,150)  
  f.motorCalibrated = 1;
}

void initServo(){  
  PWM_Command[S1] = 1800;
  PWM_Command[S2] = 1800;
  PWM_Command[S3] = 1800;
}
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                     ***only for Arduino Mega 2560***                     //
//          MOTOR : D2  D3  D5  D6  D7  D8   SERVO : D44  D45  D46          //
//                  3B  3C  3A  4A  4B  4C           5C   5B   5A           //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#if defined (MEGA)

void initPWM(){
  DDRE |= (_BV(DDE3) | _BV(DDE4) | _BV(DDE5)); //set PORTE (D5,  D2,  D3)  as an OUTPUT (MOTOR) 490 hz
  DDRH |= (_BV(DDH3) | _BV(DDH4) | _BV(DDH5)); //set PORTH (D6,  D7,  D8)  as an OUTPUT (MOTOR) 490 hz
  DDRL |= (_BV(DDL3) | _BV(DDL4) | _BV(DDL5)); //set PORTL (D46, D45, D44) as an OUTPUT (SERVO)  50 hz
  
  TCCR3A |= (_BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1)); //set CompareOutputMode to D5, D2, D3 
  TCCR3A |= _BV(WGM31); TCCR3A &= ~(_BV(WGM30)); TCCR3B |= _BV(WGM33); //use Phase Correct 16 bits
  TCCR3B &= ~(_BV(CS31)); //set Prescaler to 1 bits
  ICR3 |= 0x3FC5; //16325
  //for MOTORS
  
  TCCR4A |= (_BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1)); //set CompareOutputMode to D6, D7, D8
  TCCR4A |= _BV(WGM41); TCCR4A &= ~(_BV(WGM40)); TCCR4B |= _BV(WGM43); //use Phase Correct 16 bits
  TCCR4B &= ~(_BV(CS41)); //set Prescaler to 1 bits
  ICR4 = 0x3FC5; //16325
  //for MOTORS
  
  TCCR5A |= (_BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1)); //set CompareOutputMode to D46, D45, D44
  TCCR5A |= _BV(WGM51); TCCR5A &= ~(_BV(WGM50)); TCCR5B |= _BV(WGM53); //use Phase Correct 16 bits
  TCCR5B &= ~(_BV(CS50)); //set Prescaler to 8 bits
  ICR5 = 0x4E20; //20000
  //for SERVO
  
  #if defined(useSERVO)
    initServo();
  #endif 
}

void PWMwrite(){
  for(uint8_t i=0; i<NUMBER_MOTORS; i++) PWM_Command[i] = constrain(PWM_Command[i],MINCOMMAND,MAXCOMMAND);
  
  //Motor
  OCR3B = PWM_Command[M1] << 3; //D2 Left_Front   CW
  OCR3A = PWM_Command[M2] << 3; //D5 Right_Front  CCW
  OCR3C = PWM_Command[M3] << 3; //D3 Right_Rear   CW
  OCR4A = PWM_Command[M4] << 3; //D6 Left_Rear    CCW
  /*#if defined(HEXX) || defined(IFO)
    OCR4B = PWM_Command[M5] << 3; //D7 Left
    OCR4C = PWM_Command[M6] << 3; //D8 Right
  #endif*/
  
  //SERVO
  //OCR5A = PWM_Command[S1]; //D46 Servo Left
  //OCR5B = PWM_Command[S2]; //D45 Servo Right
  //OCR5C = PWM_Command[S3]; //D44
}

#endif
