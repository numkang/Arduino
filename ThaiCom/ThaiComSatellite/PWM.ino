//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                     ***only for Arduino Mega 2560***                     //
//          MOTOR : D2  D3  D5  D6  D7  D8   SERVO : D44  D45  D46          //
//                  3B  3C  3A  4A  4B  4C           5C   5B   5A           //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void initPWM(){
  pinMode(dirPin_azim, OUTPUT);
  pinMode(dirPin_elev, OUTPUT);
  DDRE |= (_BV(DDE4) | _BV(DDE5)); //set PORTE (D2,  D3)  as an OUTPUT (MOTOR) 5000 hz
  
  TCCR3A |= (_BV(COM3B1) | _BV(COM3C1)); //set CompareOutputMode to D5, D2, D3 
  TCCR3A |= _BV(WGM31); TCCR3A &= ~(_BV(WGM30)); TCCR3B |= _BV(WGM33); //use Phase Correct 16 bits
  TCCR3B &= ~(_BV(CS31)); //set Prescaler to 1 bits
  ICR3 = 12500; //16325,8000
  //for stepMOTORS
}

void PWMwrite_azim(int16_t pwm){
  OCR3B = pwm; //azim D2
}

void PWMstop_azim(){
  OCR3B = 0; //azim D2
}

void PWMwrite_elev(int16_t pwm){
  OCR3C = pwm; //elev D3
}

void PWMstop_elev(){
  OCR3C = 0; //elev D3
}

void stepDrive(float azimAngle, float elevAngle){
  ICR3 = 12500;
  azimAngle = constrain(azimAngle,-90.0,90.0);
  elevAngle = constrain(elevAngle,-180.0,0.0);
  
  if(potenVal_azim > (azimAngle+2) || potenVal_azim < (azimAngle-2)){
    if(azimAngle > potenVal_azim){
      digitalWrite(dirPin_azim, LOW);
      PWMwrite_azim(6250);
    }
    if(azimAngle < potenVal_azim){
      digitalWrite(dirPin_azim, HIGH);
      PWMwrite_azim(6250);
    }
  }
  else PWMstop_azim();
  
  if(potenVal_elev > (elevAngle+2) || potenVal_elev < (elevAngle-2)){
    if(elevAngle > potenVal_elev){
      digitalWrite(dirPin_elev, LOW);
      PWMwrite_elev(6250);
    }
    if(elevAngle < potenVal_elev){
      digitalWrite(dirPin_elev, HIGH);
      PWMwrite_elev(6250);
    }
  }
  else PWMstop_elev();
}

void WiistepDrive(){
  ICR3 = 16325;
  
  if(nunchuck_buf[0] < 130){
    digitalWrite(dirPin_azim, LOW);
    PWMwrite_azim(1500 << 3);
  }
  else if(nunchuck_buf[0] > 140){
    digitalWrite(dirPin_azim, HIGH);
    PWMwrite_azim(1500 << 3);
  }
  else PWMstop_azim();
  
  if(nunchuck_buf[1] < 130){
    digitalWrite(dirPin_elev, HIGH);
    PWMwrite_elev(1500 << 3);
  }
  else if(nunchuck_buf[1] > 140){
    digitalWrite(dirPin_elev, LOW);
    PWMwrite_elev(1500 << 3);
  }
  else PWMstop_elev();
  
  if(nunchuck_cbutton() == 1){
    initAzim = potenVal_azim;
    initElev = potenVal_elev;
    /*if(potenVal_azim < 0){
      EEPROM.write(1,1);
      delay(4);
      EEPROM.write(2,abs(potenVal_azim));
      delay(4);
    }
    else{
      EEPROM.write(1,0);
      delay(4);
      EEPROM.write(2,abs(potenVal_azim));
      delay(4);
    }
    EEPROM.write(3,abs(potenVal_elev));
    delay(4);*/
  }
}
