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
  PWMwriteAll(MINCOMMAND);
  delay(3000);
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                    *** only for Arduino 168/328P ***                     //
//                                                                          //
//                            D11 D3            D9  D10                     //
//                            2A  2B            1A  1B                 `    //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void initPWM(){
  DDRB |= (_BV(DDB1) | _BV(DDB2) | _BV(DDB3)); //set PORTB (D9, D10, D11) as an OUTPUT (MOTOR) 490 hz
  DDRD |= (_BV(DDD3)); //set PORTD (D3) as an OUTPUT (MOTOR) 490 hz
  
  TCCR2A |= (_BV(COM2A1) | _BV(COM2B1)); //set CompareOutputMode to D11, D3   
  TCCR1A |= (_BV(COM1A1) | _BV(COM1B1)); //set CompareOutputMode to D9, D10
  //for MOTOR use default WaveformGenerationMode and Prescaler (Phase Correct 8 bits and 64 respectively)
}

void PWMwrite(){  
  for(int i=0; i<2; i++) PWM_Command[i] = constrain(PWM_Command[i],MINCOMMAND,MAXCOMMAND);
  
  //Motor
  OCR2B = PWM_Command[M1] >> 3; //D3  Left_Front  CW
  OCR1B = PWM_Command[M2] >> 3; //D10 Right_Front CCW
  //OCR1A = PWM_Command[M3] >> 3; //D9  Right_Rear  CW
  //OCR2A = PWM_Command[M4] >> 3; //D11 Left_Rear   CCW
}
