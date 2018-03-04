void servoInit()
{
  TCCR1A |= (_BV(COM1A1)); //set CompareOutputMode to servoPIN
  TCCR1A |= _BV(WGM11); 
  TCCR1A &= ~(_BV(WGM10)); 
  TCCR1B |= _BV(WGM13); //for SERVO use Phase Correct 16 bits
  TCCR1B &= ~(_BV(CS10)); //set Prescaler to 8 bits
  ICR1 = 0x4E20; //20000
  OCR1A = 1000;
}

void servoDegree(int degree)
{
  int PWMcommand = constrain(map(degree, 0, 180, 600, 2300), 600, 2300);
  OCR1A = PWMcommand;
}

void servoDegreeToDegree(int degree1, int degree2, int duration)
{
  int PWMcommand1 = constrain(map(degree1, 0, 180, 600, 2300), 600, 2300);
  int PWMcommand2 = constrain(map(degree2, 0, 180, 600, 2300), 600, 2300);
  
  for(int i = PWMcommand1; i<= PWMcommand2; i++){
    OCR1A = i;
    delayMicroseconds(duration);
  }
  
  for(int i = PWMcommand2; i>= PWMcommand1; i--){
    OCR1A = i;
    delayMicroseconds(duration);
  }
}

void servoWithPotentiometer()
{
  potentiometerValue = analogRead(potentiometerPin); 
  delay(1);
  int PWMcommand = constrain(map(potentiometerValue, 0, 1023, 600, 2300), 600, 2300);
  OCR1A = PWMcommand;  
}
