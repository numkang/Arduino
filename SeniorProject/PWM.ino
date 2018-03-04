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

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                    *** only for Arduino 168/328P ***                     //
//                                                                          //
//                            D11 D3            D9  D10                     //
//                            2A  2B            1A  1B                 `    //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
void initPWM(){
  DDRB |= (_BV(DDB1) | _BV(DDB2)); //set PORTB (D9, D10, D11) as an OUTPUT (MOTOR) 490 hz
  
  TCCR1A |= (_BV(COM1A1) | _BV(COM1B1)); //set CompareOutputMode to D9, D10
  /*TCCR1A |= _BV(WGM11); TCCR1A &= ~(_BV(WGM10)); TCCR1B |= _BV(WGM13); //use Phase Correct 16 bits
  TCCR1B &= ~(_BV(CS10)); //set Prescaler to 8 bits
  ICR1 = 0x4E20; //20000*/
  //for 16-bits WGM
}

void PWMwrite(){  
  //for(uint8_t i=0; i<NUMBER_MOTORS; i++) PWM_Command[i] = constrain(PWM_Command[i],MINCOMMAND,MAXCOMMAND);
  
  //PWM RX
  OCR1A = PWM_Command[ROLL] >> 3; //D9  CH1
  OCR1B = PWM_Command[PITCH] >> 3; //D10 CH2
}
