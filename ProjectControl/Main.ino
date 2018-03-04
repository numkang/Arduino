void setup(){
  initPWM();
  previousTime = micros();
  SerialOpen(0,115200);
  pinMode(13,OUTPUT);
}

void loop(){
  previousTime = currentTime;
  currentTime = micros();
  cycleTime = currentTime - previousTime;  
  serialCom(0);  
  
  if(currentTime > taskTime){
    taskTime = currentTime + 66667;
    potenVal = analogRead(potenPin);
    angle = mapfloat(potenVal,potenMin,potenMax,0,180);
    PID();
    PWMwrite(ICR1*0.5);
    if(abs(angle)<1) digitalWrite(13,HIGH);
    else if(angle>179 && angle < 181) digitalWrite(13,HIGH);
    else digitalWrite(13,LOW);
  }
  SerialPrint(100);
}
