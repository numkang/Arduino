//Spinning Servo

int servoPin = 9;
int potentiometerPin = 0;
int potentiometerValue;

void setup()
{
  Serial.begin(57600);
  pinMode(servoPin, OUTPUT);
  servoInit();
}

void loop()
{
  servoDegree(0);
  //servoDegreeToDegree(0, 180, 3000); //degree1, degree2, duration in microsecond
  //servoWithPotentiometer();
}
