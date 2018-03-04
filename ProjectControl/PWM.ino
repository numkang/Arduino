void initPWM(){
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  
  TCCR1A |= (_BV(COM1A1)); //set CompareOutputMode to D6
  TCCR1A |= _BV(WGM11); TCCR1A &= ~(_BV(WGM10)); TCCR1B |= _BV(WGM13); //use Phase Correct 16 bits
  ICR1 = 10000;
  //for stepMOTORS (Phase Correct 16 bits and Prescaler 64)
}

void PWMwrite(int16_t pwm){
  if(angle < 180) OCR1A = pwm;
  else OCR1A = 0;
}

void PID(){
  pp_error = p_error;
  p_error = error;  
  p_output = output;
  
  error = input - angle;
  output = p_output + 95*error - 140*p_error + 45*pp_error;
  if(error > 0) digitalWrite(dirPin, HIGH);
  else if(error < 0) digitalWrite(dirPin, LOW);
  
  if(abs(error) >= 1) ICR1 = 125000/abs(int16_t(output));
  else ICR1 = 6250;
}
