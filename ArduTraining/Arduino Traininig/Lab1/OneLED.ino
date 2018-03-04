void TurnON()
{
  digitalWrite(ledPins[2], HIGH);
}

/******************************************************/

void TurnOFF()
{
  digitalWrite(ledPins[2], LOW);
}

/******************************************************/

void Blink()
{
  digitalWrite(ledPins[2], HIGH);
  delay(1000); 
 
  digitalWrite(ledPins[2], LOW);
  delay(1000); 
}

/******************************************************/
