//Pushing Button

int buttonPin = 2;
int ledPin = 13;

void setup()
{
  Serial.begin(57600);
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT); 
}

void loop()
{
  int buttonState = digitalRead(buttonPin);
 
  if(buttonState == HIGH)
  {
     digitalWrite(ledPin, HIGH); 
  }
  else
  {
     digitalWrite(ledPin, LOW);
  }
}
