//BLINKING LEDs

int ledPins[] = {11,12,13};

void setup()
{
  Serial.begin(57600);
  
  pinMode(ledPins[0], OUTPUT);
  pinMode(ledPins[1], OUTPUT);
  pinMode(ledPins[2], OUTPUT);
  
  /*for(int i = 0; i <= 2; i++){
    pinMode(ledPins[i], OUTPUT);
  }*/  
}

void loop()
{
  //for OneLED
  TurnON();  
  //TurnOFF();  
  //Blink();    
  
  //for Triple LED
  //oneAfterAnother();
  //oneOnAtATimeDirect();
  //oneOnAtATimeReverse();
  //randomLED();
}
