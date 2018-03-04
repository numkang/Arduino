//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                            PWM General Function                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void PWMwriteAll(int16_t pwm) {   // Sends commands to all motors
  for (uint8_t i = 0; i<8; i++){
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
  PWM_Command[S1] = 1500;
  PWM_Command[S2] = 1500;
  PWM_Command[S3] = 1500;
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                    *** only for Arduino 168/328P ***                     //
//                                                                          //
//                            D11 D3            D9  D10                     //
//                            2A  2B            1A  1B                 `    //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#if defined (UNO) || defined (PROMINI)

void initPWM(){
  DDRB |= (_BV(DDB1) | _BV(DDB2) | _BV(DDB3)); //set PORTB (D9, D10, D11) as an OUTPUT (MOTOR) 490 hz
  DDRD |= (_BV(DDD3)); //set PORTD (D3) as an OUTPUT (MOTOR) 490 hz
  
  TCCR2A |= (_BV(COM2A1) | _BV(COM2B1)); //set CompareOutputMode to D11, D3 
  //for MOTOR use default WaveformGenerationMode and Prescaler (Phase Correct 8 bits and 64 respectively)
  
  TCCR1A |= (_BV(COM1A1) | _BV(COM1B1)); //set CompareOutputMode to D9, D10
  /*TCCR1A |= _BV(WGM11); TCCR1A &= ~(_BV(WGM10)); TCCR1B |= _BV(WGM13); //use Phase Correct 16 bits
  TCCR1B &= ~(_BV(CS10)); //set Prescaler to 8 bits
  ICR1 = 0x4E20; //20000*/
  //for SERVO
  
  #if defined(useSERVO)
    initServo();
  #endif  
}

void PWMwrite(){  
  for(uint8_t i=0; i<NUMBER_MOTORS; i++) PWM_Command[i] = constrain(PWM_Command[i],MINCOMMAND,MAXCOMMAND);
  
  //Motor
  OCR2B = PWM_Command[M1] >> 3; //D3  Left_Front  CW
  OCR1B = PWM_Command[M2] >> 3; //D10 Right_Front CCW
  OCR1A = PWM_Command[M3] >> 3; //D9  Right_Rear  CW
  OCR2A = PWM_Command[M4] >> 3; //D11 Left_Rear   CCW
  
  //Servo
  //OCR1A = PWM_Command[S1];
  //OCR1B = PWM_Command[S2];
}

#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                    *** only for Arduino 16/32U4 ***                      //
//                                                                          //
//                  MOTOR : D5  D6       SERVO : D9 D11                     //
//                          3A  4D               1A  1C                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#if defined (NANO)

void initPWM(){
  DDRB |= (_BV(DDB5) | _BV(DDB7)); //set PORTB (D9, D11) as an OUTPUT (SERVO) 50 hz
  DDRC |= (_BV(DDC6)); //set PORTD (D5) as an OUTPUT (MOTOR) 490 hz
  DDRD |= (_BV(DDD7)); //set PORTD (D6) as an OUTPUT (MOTOR) 490 hz
  
  TCCR1A |= (_BV(COM1A1) | _BV(COM1C1)); //set CompareOutputMode to D9, D11
  TCCR1A |= _BV(WGM11); TCCR1A &= ~(_BV(WGM10)); TCCR1B |= _BV(WGM13); //use Phase Correct 16 bits
  TCCR1B &= ~(_BV(CS10)); //set Prescaler to 8 bits
  ICR1 = 0x4E20; //20000 50Hz
  //for SERVO 
  
  TCCR3A |= (_BV(COM3A1)); //set CompareOutputMode to D5
  TCCR3A |= _BV(WGM31); TCCR3A &= ~(_BV(WGM30)); TCCR3B |= _BV(WGM33); //use Phase Correct 16 bits
  TCCR3B &= ~(_BV(CS31)); //set Prescaler to 1 bits
  ICR3 = 0x3FC5; //16325
  //for MOTORS
  
  TCCR4C |= (_BV(COM4D1)); //set CompareOutputMode to D6
  TCCR4B |= (_BV(CS40) | _BV(CS41) | _BV(CS42)); TCCR4B &= ~(_BV(CS43)); //set Prescaler to 64 bits
  //for MOTORS
  
  #if defined(useSERVO)
    initServo();
  #endif 
}

void PWMwrite(){  
  for(uint8_t i=0; i<NUMBER_MOTORS; i++) PWM_Command[i] = constrain(PWM_Command[i],MINCOMMAND,MAXCOMMAND);
  
  //Servo
  OCR1A = PWM_Command[S1]; //D9  Left
  OCR1C = PWM_Command[S2]; //D11 Right
  
  //Motor
  OCR3A = PWM_Command[M1] << 3; //D5 Left
  OCR4D = PWM_Command[M2] >> 3; //D6 Right
}

#endif

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
  ICR3 = 0x3FC5; //16325
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
  #if defined(HEXX) || defined(IFO)
    OCR4B = PWM_Command[M5] << 3; //D7 Left
    OCR4C = PWM_Command[M6] << 3; //D8 Right
  #endif
  
  //SERVO
  OCR5A = PWM_Command[S1]; //D46
  OCR5B = PWM_Command[S2]; //D45
  OCR5C = PWM_Command[S3]; //D44
}

#endif
