void oneAfterAnother()
{
  int i;
  int delayTime = 500; //milliseconds 
  
  for(i = 0; i <= 2; i++)
  {
    digitalWrite(ledPins[i], HIGH);
    delay(delayTime);                
  }
 
  for(i = 2; i >= 0; i--)
  {
    digitalWrite(ledPins[i], LOW);
    delay(delayTime);
  }               
}

/******************************************************/

void oneOnAtATimeDirect()
{
  int i;
  int delayTime = 500; //milliseconds
  
  for(i = 0; i <= 2; i++)
  {
    digitalWrite(ledPins[i], HIGH);
    delay(delayTime);
    digitalWrite(ledPins[i], LOW);
  }
}

/******************************************************/

void oneOnAtATimeReverse()
{
  int i;
  int delayTime = 500; //milliseconds
  
  for(i = 0; i <= 2; i++)
  {
    digitalWrite(ledPins[i], HIGH);
    delay(delayTime);
    digitalWrite(ledPins[i], LOW);
  }
  
  for(i = 2; i >= 0; i--)
  {
    digitalWrite(ledPins[i], HIGH);
    delay(delayTime);
    digitalWrite(ledPins[i], LOW);
  }
}

/******************************************************/

void randomLED()
{
  int i;
  int delayTime = 500;
  
  i = random(3); //pick a random number between 0 and 2
	
  digitalWrite(ledPins[i], HIGH);
  delay(delayTime);
  digitalWrite(ledPins[i], LOW);
}
