void initPWM(){
  pinMode(dirPin_azim, OUTPUT);
  pinMode(dirPin_elev, OUTPUT);
  DDRB |= (_BV(DDB1) | _BV(DDB2)); //set PORTB (D9, D10) as an OUTPUT (MOTOR) 490 hz
  
  TCCR1A |= (_BV(COM1A1) | _BV(COM1B1)); //set CompareOutputMode to D9, D10
  TCCR1A |= _BV(WGM11); TCCR1A &= ~(_BV(WGM10)); TCCR1B |= _BV(WGM13); //use Phase Correct 16 bits
  //TCCR1B &= ~(_BV(CS11)); //set Prescaler to 8 bits
  ICR1 = 1250;
  //for stepMOTORS
}

void PWMwrite_azim(int16_t pwm){
  OCR1B = pwm; //azim D10
}

void PWMstop_azim(){
  OCR1B = 0; //azim D10
}

void PWMwrite_elev(int16_t pwm){
  OCR1A = pwm; //elev D9
}

void PWMstop_elev(){
  OCR1A = 0; //elev D9
}

void stepDrive(uint8_t c){   
  if(c == 'a' || c == 'd' || c == 'A' || c == 'D'){
    ICR1 = 1000;
    if(c == 'd' && step_cc > 0){ //step drive RIGHT
      digitalWrite(dirPin_azim, LOW);
      PWMwrite_azim(ICR1*0.5);
      step_cc--;
    }
    else if(c == 'a' && step_cc > 0){ //step drive LEFT
      digitalWrite(dirPin_azim, HIGH);
      PWMwrite_azim(ICR1*0.5);
      step_cc--;
    }
    else if(c == 'D'){ //continuous drive RIGHT
      digitalWrite(dirPin_azim, LOW);
      PWMwrite_azim(ICR1*0.5);
    }
    else if(c == 'A'){ //continuous drive LEFT
      digitalWrite(dirPin_azim, HIGH);
      PWMwrite_azim(ICR1*0.5);
    }
    else PWMstop_azim();
  }
  else PWMstop_azim();
  
  if(c == 'w' || c == 's' || c == 'W' || c == 'S'){
    ICR1 = 250;
    if(c == 's' && step_cc > 0){ //step drive
      digitalWrite(dirPin_elev, LOW);
      PWMwrite_elev(ICR1*0.5);
      step_cc--;
    }
    else if(c == 'w' && step_cc > 0){ //step drive
      digitalWrite(dirPin_elev, HIGH);
      PWMwrite_elev(ICR1*0.5);
      step_cc--;
    }
    else if(c == 'S'){ //continuous drive
      digitalWrite(dirPin_elev, LOW);
      PWMwrite_elev(ICR1*0.5);
    }
    else if(c == 'W'){ //continuous drive
      digitalWrite(dirPin_elev, HIGH);
      PWMwrite_elev(ICR1*0.5);
    }
    else PWMstop_elev();
  }
  else PWMstop_elev();
}
