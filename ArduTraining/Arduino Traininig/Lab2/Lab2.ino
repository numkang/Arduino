//Control the LED with a Potentiometer 

int potentiometerPin = 0;    // The potentiometer is connected to analog pin 0
int ledPin = 11;  
int potentiometerValue;

void setup()
{
  Serial.begin(57600);
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  potentiometerValue = analogRead(potentiometerPin);
  
  Blink();
  //Bright();
}

void Blink()
{
  digitalWrite(ledPin, HIGH);
  delay(potentiometerValue);
  digitalWrite(ledPin, LOW);
  delay(potentiometerValue);
}

void Bright()
{
 int brightness = map(potentiometerValue,0,1023,0,255);
 analogWrite(ledPin, brightness);
}
