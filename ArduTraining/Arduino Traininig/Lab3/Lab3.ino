//Mixing color with RGB LED

int RED_PIN = 9;
int GREEN_PIN = 10;
int BLUE_PIN = 11;

int DISPLAY_TIME = 100;  //milliseconds

int RED = 255;
int GREEN = 155;
int BLUE = 117;

void setup()
{
  Serial.begin(57600);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
}

void loop()
{
  mainColors();
  //subColors();
  //showSpectrum();
  //ownColor();
}

void ownColor()
{
  analogWrite(RED_PIN, RED);
  analogWrite(BLUE_PIN, GREEN);
  analogWrite(GREEN_PIN, BLUE);
}
