void initPWM(){
  pinMode(dirPin_azim, OUTPUT);
  pinMode(dirPin_elev, OUTPUT);
  DDRB |= (_BV(DDB1) | _BV(DDB2)); //set PORTB (D9, D10) as an OUTPUT (MOTOR) 490 hz
  
  TCCR1A |= (_BV(COM1A1) | _BV(COM1B1)); //set CompareOutputMode to D9, D10
  TCCR1A |= _BV(WGM11); TCCR1A &= ~(_BV(WGM10)); TCCR1B |= _BV(WGM13); //use Phase Correct 16 bits
  //TCCR1B &= ~(_BV(CS11)); //set Prescaler to 8 bits
  ICR1 = 1250;
  //for stepMOTORS
  
  DDRD |= (_BV(DDD3));
  TCCR2A |= (_BV(COM2B1));
}

void PWMwrite_azim(int16_t pwm){
  OCR1B = pwm; //azim D10
}

void PWMstop_azim(){
  OCR1B = 0; //azim D10
}

void PWMwrite_elev(int16_t pwm){
  //OCR1A = pwm; //elev D9
  OCR2B = pwm;
}

void PWMstop_elev(){
  //OCR1A = 0; //elev D9
  OCR2B = 0;
}

void stepDrive(uint8_t c){   
  if(c == 'a' || c == 'd' || c == 'A' || c == 'D'){
    ICR1 = 1250;
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
    ICR1 = 125;
    if(c == 's' && step_cc > 0){ //step drive DOWN
      digitalWrite(dirPin_elev, LOW);
      PWMwrite_elev(1500>>3);
      step_cc--;
    }
    else if(c == 'w' && step_cc > 0){ //step drive UP
      digitalWrite(dirPin_elev, HIGH);
      PWMwrite_elev(1500>>3);
      step_cc--;
    }
    else if(c == 'S'){ //continuous drive DOWN
      digitalWrite(dirPin_elev, LOW);
      PWMwrite_elev(1500>>3);
    }
    else if(c == 'W'){ //continuous drive UP
      digitalWrite(dirPin_elev, HIGH);
      PWMwrite_elev(1500>>3);
    }
    else PWMstop_elev();
  }
  else PWMstop_elev();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stepDrive_Auto_azim(){   
  /*if(angle[YAW] < 97 || angle[YAW] > 98){
    ICR1 = 1000;
    if(angle[YAW] > 98){ //continuous drive RIGHT
      digitalWrite(dirPin_azim, LOW);
      PWMwrite_azim(ICR1*0.5);
    }
    else if(angle[YAW] < 97){ //continuous drive LEFT
      digitalWrite(dirPin_azim, HIGH);
      PWMwrite_azim(ICR1*0.5);
    }
    else PWMstop_azim();
  }
  else PWMstop_azim();*/
  
  if(sensorVal < 900 && yaw_check < 3){
    ICR1 = 1250;    
    digitalWrite(dirPin_azim, LOW);
    PWMwrite_azim(ICR1*0.5);    
  }
  else if(sensorVal > 900) yaw_check++;
  else PWMstop_azim();
}

void stepDrive_Auto_elev(){ 
  if(angle[PITCH] < 10 || angle[PITCH] > 11){
    //ICR1 = 125;
    if(angle[PITCH] > 11){ //continuous drive DOWN
      digitalWrite(dirPin_elev, LOW);
      PWMwrite_elev(1500>>3);
    }
    else if(angle[PITCH] < 10){ //continuous drive UP
      digitalWrite(dirPin_elev, HIGH);
      PWMwrite_elev(1500>>3);
    }
    else PWMstop_elev();
  }
  else PWMstop_elev();
}

void stepDrive_Auto(){
  if(angle[PITCH] < 10 || angle[PITCH] > 10 && pitch_check < 100){
    PWMstop_azim();
    //ICR1 = 125;
    if(angle[PITCH] >= 11){ //continuous drive DOWN
      digitalWrite(dirPin_elev, LOW);
      PWMwrite_elev(1500>>3);
    }
    else if(angle[PITCH] <= 10){ //continuous drive UP
      digitalWrite(dirPin_elev, HIGH);
      PWMwrite_elev(1500>>3);
    }
    else PWMstop_elev();
  }
  else if(angle[PITCH] >= 10 && angle[PITCH] <= 11 && pitch_check < 100) pitch_check++;
  else if(pitch_check >= 100 && angle[PITCH] < 7) pitch_check = 50;
  else if(pitch_check >= 100 && angle[PITCH] > 15) pitch_check = 50;
  else if(sensorVal < 900 && yaw_check < 3 && pitch_check >= 100){
    if(sTime > 35000 || sTime < 10000){
    PWMstop_elev();
    ICR1 = 1250;
    if(yaw_init > 90 && yaw_init < 270) digitalWrite(dirPin_azim, LOW);
    else if(yaw_init < 90 || yaw_init > 270) digitalWrite(dirPin_azim, HIGH);
    PWMwrite_azim(ICR1*0.5);
    }
  }
  else if(sensorVal > 900 && sTime > 10000 && sTime < 35000) yaw_check++;
  else{
    PWMstop_elev();
    PWMstop_azim();
  }
}
